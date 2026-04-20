#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy # [新增] 引入 QoS 模組
from sensor_msgs.msg import CompressedImage
import time
import matplotlib.pyplot as plt
import csv
import numpy as np
from collections import defaultdict
import argparse
import os
from datetime import datetime


class DDSNetworkLogger(Node):

    def __init__(self, topic_name, duration, expected_fps, output_dir):
        super().__init__('dds_network_logger')

        self.topic_name = topic_name
        self.duration = duration
        self.expected_fps = expected_fps
        self.output_dir = output_dir

        # [修正 1] 定義正確的 QoS Profile
        # Best Effort: 丟包不重傳 (測量真實頻寬)
        # Keep Last 1: 只保留最新一張 (避免 Queue 排隊導致假延遲)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # 訂閱影像
        self.subscription = self.create_subscription(
            CompressedImage,
            topic_name,
            self.listener_callback,
            sensor_qos  # [修正] 使用定義好的 QoS
        )

        # Log
        self.log = {
            "wall_time": [],
            "latency_ms": [],
            "size_bytes": [],
        }

        # 統計
        self.bytes_per_second = defaultdict(int)
        self.bytes_total = 0
        self.msg_count = 0
        self.periods = []

        # 時間控制
        self.start_wall_time = None
        self.last_recv_wall_time = None
        self.finished = False
        
        # 狀態旗標
        self.has_time_sync_issue = False

        self.get_logger().info(
            f"[START] Logging on '{topic_name}' for {duration}s (expected FPS={expected_fps})"
        )
        self.get_logger().info(f"[CONFIG] QoS set to BEST_EFFORT, Depth=1")

    def listener_callback(self, msg):
        now_wall = time.time()

        # 第一次收到 message
        if self.start_wall_time is None:
            self.start_wall_time = now_wall
            self.get_logger().info("[INFO] First message received, start timing.")

        elapsed = now_wall - self.start_wall_time

        # 到時間 → 設定 finished flag
        if elapsed > self.duration:
            self.finished = True
            return

        # [修正 2] Compute latency 與 時間同步檢查
        # 注意：msg.header.stamp 是發送端的系統時間
        t_pub_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        t_now_ns = self.get_clock().now().nanoseconds
        latency_ms = (t_now_ns - t_pub_ns) / 1e6
        
        # [新增] 負延遲檢查：如果延遲小於 -5ms，極有可能是兩台電腦時間沒同步
        if latency_ms < -5.0:
            if not self.has_time_sync_issue:
                self.get_logger().warn(f"⚠ NEGATIVE LATENCY DETECTED ({latency_ms:.2f} ms)!")
                self.get_logger().warn("⚠ Source and Receiver clocks are NOT synced. Latency data will be wrong.")
                self.has_time_sync_issue = True
            # 為了避免圖表壞掉，將負值設為 0 或標記
            # latency_ms = 0 

        size_bytes = len(msg.data)

        # 計算 period（訊息間隔）
        if self.last_recv_wall_time is not None:
            self.periods.append(now_wall - self.last_recv_wall_time)
        self.last_recv_wall_time = now_wall

        # Log
        self.log["wall_time"].append(elapsed)
        self.log["latency_ms"].append(latency_ms)
        self.log["size_bytes"].append(size_bytes)

        # 累積統計
        sec_bin = int(elapsed)
        self.bytes_per_second[sec_bin] += size_bytes
        self.msg_count += 1
        self.bytes_total += size_bytes


# -------------------------------------------------------------
# 儲存結果
# -------------------------------------------------------------
def save_results(logger):
    os.makedirs(logger.output_dir, exist_ok=True)

    times = np.array(logger.log["wall_time"])
    lat = np.array(logger.log["latency_ms"])
    sizes = np.array(logger.log["size_bytes"])

    if len(times) == 0:
        print("[ERROR] No messages received.")
        return

    duration = times[-1] - times[0]
    msg_count = logger.msg_count
    expected_fps = logger.expected_fps

    # Latency stats
    mean_latency = np.mean(lat)
    p90 = np.percentile(lat, 90)
    jitter = np.abs(np.diff(lat)) if len(lat) > 1 else np.array([])
    mean_jitter = np.mean(jitter) if len(jitter) > 0 else 0

    # FPS
    avg_fps = msg_count / duration if duration > 0 else 0

    # [修正 3] Loss Rate 計算
    # 這是「總體掉幀率」，包含了 相機沒產生 + 網路掉包
    if expected_fps > 0:
        expected_msgs = expected_fps * duration
        loss_count = max(0, expected_msgs - msg_count)
        loss_rate = loss_count / expected_msgs
    else:
        loss_rate = 0

    # Bandwidth
    avg_bw_mbps = (logger.bytes_total * 8 / duration) / 1e6

    # Bufferbloat trend
    slope = np.polyfit(times, lat, 1)[0] if len(times) > 1 else 0

    # -----------------------------
    # Print summary
    # -----------------------------
    print("\n" + "="*30)
    print("   DDS NETWORK REPORT")
    print("="*30)
    if logger.has_time_sync_issue:
        print("⚠ WARNING: CLOCK SYNC ISSUE DETECTED. LATENCY DATA IS INVALID.")
    
    print(f"Duration      : {duration:.2f} s")
    print(f"Total Msgs    : {msg_count}")
    print(f"-"*30)
    print(f"Avg Latency   : {mean_latency:.2f} ms")
    print(f"P90 Latency   : {p90:.2f} ms")
    print(f"Avg Jitter    : {mean_jitter:.2f} ms")
    print(f"-"*30)
    print(f"Expected FPS  : {expected_fps:.2f}")
    print(f"Actual FPS    : {avg_fps:.2f}")
    print(f"Frame Loss    : {loss_rate*100:.2f}% (Source+Network)")
    print(f"-"*30)
    print(f"Bandwidth     : {avg_bw_mbps:.3f} Mbps")
    print(f"Queue Slope   : {slope:.4f} ms/s (Bufferbloat)")
    print("="*30 + "\n")

    # -----------------------------
    # 1. Save CSV
    # -----------------------------
    csv_path = os.path.join(logger.output_dir, "network_log.csv")
    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["time_sec", "latency_ms", "size_bytes"])
        for t, l, s in zip(times, lat, sizes):
            writer.writerow([t, l, s])
    print(f"[SAVED] {csv_path}")

    # -----------------------------
    # 2. Latency Plot
    # -----------------------------
    plt.figure(figsize=(10, 6))
    plt.plot(times, lat, label='Latency')
    plt.xlabel("Time (sec)")
    plt.ylabel("Latency (ms)")
    plt.title(f"DDS Latency (Avg: {mean_latency:.1f}ms)")
    if logger.has_time_sync_issue:
        plt.title(f"DDS Latency (INVALID: CLOCK SYNC ERROR)")
    plt.grid(True)
    plt.legend()
    plt.savefig(os.path.join(logger.output_dir, "latency.png"))
    plt.close()

    # -----------------------------
    # 3. Jitter Plot
    # -----------------------------
    if len(jitter) > 0:
        plt.figure(figsize=(10, 6))
        plt.plot(times[1:], jitter, color='orange', label='Jitter')
        plt.xlabel("Time (sec)")
        plt.ylabel("Jitter (ms)")
        plt.title("DDS Jitter (Stability)")
        plt.grid(True)
        plt.legend()
        plt.savefig(os.path.join(logger.output_dir, "jitter.png"))
        plt.close()

    # -----------------------------
    # 4. Bandwidth / sec Plot
    # -----------------------------
    secs = sorted(logger.bytes_per_second.keys())
    bw = [logger.bytes_per_second[s] * 8 / 1e6 for s in secs]

    plt.figure(figsize=(10, 6))
    plt.plot(secs, bw, color='green', label='Bandwidth')
    plt.xlabel("Time (sec)")
    plt.ylabel("Bandwidth (Mbps)")
    plt.title(f"Bandwidth Usage (Avg: {avg_bw_mbps:.1f} Mbps)")
    plt.grid(True)
    plt.legend()
    plt.savefig(os.path.join(logger.output_dir, "bandwidth.png"))
    plt.close()

    print(f"[ALL RESULTS SAVED TO] {logger.output_dir}")


# -------------------------------------------------------------
# Main
# -------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/camera/camera/color/image_raw/compressed")
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--expected-fps", type=float, default=30.0)
    args = parser.parse_args()

    # 建立資料夾
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = f"/root/NETWORK/output/{timestamp}"

    rclpy.init()
    node = DDSNetworkLogger(args.topic, args.duration, args.expected_fps, output_dir)

    # 使用 spin_once 來精準控制
    while rclpy.ok() and not node.finished:
        rclpy.spin_once(node, timeout_sec=0.1)

    # 時間到 → 儲存結果
    save_results(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()