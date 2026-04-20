#!/usr/bin/env python3
import roslibpy
import base64
import numpy as np
import time
import rclpy
from rclpy.clock import Clock

ROBOT_IP = "192.168.0.168"
TOPIC = "/camera/camera/color/image_raw/compressed"
TEST_DURATION = 10   # run for 10 seconds then auto-stop

# ROS clock
rclpy.init()
clock = Clock()

# Stats
lat_list = []
jit_list = []
bw_list = []
fps_list = []
rtt_list = []

expected_fps = 30


def calc_latency(stamp):
    try:
        t_pub = stamp["sec"] * 1_000_000_000 + stamp["nanosec"]
        t_now = clock.now().nanoseconds
        return (t_now - t_pub) / 1e6
    except:
        return None


def decode_compressed(msg):
    return len(base64.b64decode(msg["data"]))


def measure_rtt(ros):
    """safe RTT measurement"""
    try:
        t0 = time.time()
        ros.get_time()
        t1 = time.time()
        return (t1 - t0) * 1000
    except:
        return None


def main():
    ros = roslibpy.Ros(host=ROBOT_IP, port=9090)

    print("[INFO] Connecting to rosbridge...")
    ros.run()
    print("[OK] Connected.")

    topic = roslibpy.Topic(ros, TOPIC, "sensor_msgs/msg/CompressedImage")

    # local state
    start_time = time.time()
    last_latency = None
    bytes_accum = 0
    frame_count = 0
    bw_ts = time.time()
    fps_ts = time.time()

    done = False   # stopper flag

    def callback(msg):
        print("[DEBUG] Got message")

        nonlocal last_latency, bytes_accum, frame_count, bw_ts, fps_ts, done

        now = time.time()

        if done:
            return

        # --------------- LATENCY -----------------
        latency = calc_latency(msg["header"]["stamp"])
        if latency is not None:
            lat_list.append(latency)

            if last_latency is not None:
                jit_list.append(abs(latency - last_latency))
            last_latency = latency

        # --------------- BANDWIDTH ---------------
        size = decode_compressed(msg)
        bytes_accum += size

        if now - bw_ts >= 1:
            bw_list.append(bytes_accum / 1e6)  # MB/s
            bytes_accum = 0
            bw_ts = now

        # --------------- FPS ---------------------
        frame_count += 1
        if now - fps_ts >= 1:
            fps_list.append(frame_count)
            frame_count = 0
            fps_ts = now

        # --------------- RTT ---------------------
        rtt = measure_rtt(ros)
        if rtt is not None:
            rtt_list.append(rtt)

        # --------------- AUTOSTOP (10 SEC) -------
        if now - start_time >= TEST_DURATION:
            done = True
            ros.close()  # non-blocking safe close
            print("[INFO] Test finished. Generating report...")

    topic.subscribe(callback)

    # Main loop with auto-stop
    while not done:
        time.sleep(0.05)

    # Prevent deadlock: do NOT call topic.unsubscribe() or ros.terminate()
    # roslibpy has known unsubscribe deadlock issues

    # -------------------- RESULTS ------------------------
    print("\n===== ROSBRIDGE TEST RESULT (10 sec) =====")
    print(f"Mean Latency : {np.mean(lat_list):.2f} ms")
    print(f"Latency Jitter(std): {np.std(lat_list):.2f} ms")
    print(f"P90 Latency : {np.percentile(lat_list,90):.2f} ms")
    print(f"P95 Latency : {np.percentile(lat_list,95):.2f} ms")

    if len(bw_list) > 0:
        print(f"Average Bandwidth : {np.mean(bw_list):.2f} MB/s")

    if len(fps_list) > 0:
        print(f"Average FPS : {np.mean(fps_list):.2f}")

    if len(rtt_list) > 0:
        print(f"Average RTT : {np.mean(rtt_list):.2f} ms")

    print("==========================================\n")


if __name__ == "__main__":
    main()
