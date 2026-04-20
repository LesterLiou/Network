#!/usr/bin/env python3
import roslibpy
import base64
import numpy as np
import cv2
import time
import rclpy
from rclpy.clock import Clock
from rclpy.time import Time

ROBOT_IP = '192.168.0.168'
TOPIC = '/camera/camera/color/image_raw/compressed'

# 性能統計
last_fps_time = time.time()
frame_count = 0
last_bytes_time = time.time()
bytes_accum = 0
last_seq = None

# ROS2 clock 用來算 latency（不走 DDS）
rclpy.init()
clock = Clock()

def decode_compressed(msg):
    img_bytes = base64.b64decode(msg['data'])
    np_arr = np.frombuffer(img_bytes, np.uint8)
    img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    return img, len(img_bytes)

def calc_latency(stamp):
    """計算 pub → rosbridge → subscriber 的延遲（ms）"""
    try:
        t_pub = Time(seconds=stamp['sec'], nanoseconds=stamp['nanosec'])
        t_now = clock.now()
        latency_ms = (t_now - t_pub).nanoseconds / 1e6
        return latency_ms
    except:
        return -1


def main():
    global frame_count, last_fps_time, bytes_accum, last_bytes_time, last_seq

    ros = roslibpy.Ros(host=ROBOT_IP, port=9090)
    ros.run()
    print("[OK] Connected to rosbridge")

    topic = roslibpy.Topic(
        ros,
        TOPIC,
        'sensor_msgs/CompressedImage'
    )

    def callback(msg):
        global frame_count, last_fps_time, last_bytes_time, bytes_accum, last_seq

        # ---- 取出 publisher timestamp ----
        stamp = msg['header']['stamp']
        latency_ms = calc_latency(stamp)

        # ---- 計算影像大小 ----
        img, byte_size = decode_compressed(msg)
        bytes_accum += byte_size

        # ---- 檢查 sequence 是否有跳號（掉幀檢測） ----
        seq = msg['header'].get('seq', None)
        if last_seq is not None and seq is not None:
            if seq != last_seq + 1:
                print(f"[WARN] Frame drop detected! seq={last_seq} → {seq}")
        last_seq = seq if seq is not None else last_seq

        # ---- FPS 計算 ----
        frame_count += 1
        now = time.time()
        if now - last_fps_time >= 1.0:
            fps = frame_count
            print(f"[FPS] {fps} frames/sec")
            frame_count = 0
            last_fps_time = now

        # ---- 頻寬（bytes/sec → MB/s） ----
        if now - last_bytes_time >= 1.0:
            mb_per_sec = bytes_accum / 1e6
            print(f"[BW] {mb_per_sec:.2f} MB/s")
            bytes_accum = 0
            last_bytes_time = now

        # ---- 延遲 ----
        print(f"[LATENCY] {latency_ms:.2f} ms | size={byte_size} bytes")

        # 顯示影像（可關掉）
        #if img is not None:
            #cv2.imshow("ROSBridge Image", img)
            #cv2.waitKey(1)

    topic.subscribe(callback)
    print(f"[OK] Subscribed to {TOPIC}")
    print("=== Waiting for images... ===")

    try:
        while True:
            pass
    except KeyboardInterrupt:
        pass

    topic.unsubscribe()
    ros.terminate()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
