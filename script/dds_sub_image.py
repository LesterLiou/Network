#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import time
import matplotlib.pyplot as plt
import csv
import numpy as np


class DDSLatencyLogger(Node):

    def __init__(self):
        super().__init__('dds_latency_logger')

        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera/camera/color/image_raw/compressed',
            self.listener_callback,
            10
        )

        # Logs
        self.log = {
            "time": [],
            "latency_ms": [],
            "size_bytes": []
        }

        self.start_time = time.time()
        self.duration = 60  # seconds

        print("[START] Logging DDS latency for 60 seconds...")

    def listener_callback(self, msg):
        now = time.time()
        elapsed = now - self.start_time

        # Stop
        if elapsed > self.duration:
            print("[DONE] Time reached. Stopping logger.")
            rclpy.shutdown()
            return

        # Compute latency
        t_pub_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        t_now_ns = self.get_clock().now().nanoseconds
        latency_ms = (t_now_ns - t_pub_ns) / 1e6

        # Add to log
        self.log["time"].append(elapsed)
        self.log["latency_ms"].append(latency_ms)
        self.log["size_bytes"].append(len(msg.data))


def save_results(log):
    times = log["time"]
    lat = np.array(log["latency_ms"])
    sizes = log["size_bytes"]

    # ───────────────────────────────────────────────
    # 1. Compute statistics
    # ───────────────────────────────────────────────
    mean_latency = np.mean(lat)
    p90 = np.percentile(lat, 90)
    p95 = np.percentile(lat, 95)

    # jitter = 相鄰 latency 差值的絕對值
    jitter = np.abs(np.diff(lat))

    print("===== DDS LATENCY STATISTICS =====")
    print(f"Mean latency     : {mean_latency:.2f} ms")
    print(f"90th percentile  : {p90:.2f} ms")
    print(f"95th percentile  : {p95:.2f} ms")
    print(f"Average jitter   : {np.mean(jitter):.2f} ms")
    print("==================================")

    # ───────────────────────────────────────────────
    # 2. Save CSV
    # ───────────────────────────────────────────────
    with open("dds_latency_log.csv", "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["time_sec", "latency_ms", "size_bytes"])
        for t, l, s in zip(times, lat, sizes):
            writer.writerow([t, l, s])
    print("[SAVED] dds_latency_log.csv")

    # ───────────────────────────────────────────────
    # 3. Plot latency curve
    # ───────────────────────────────────────────────
    plt.figure(figsize=(10,5))
    plt.plot(times, lat, label="Latency (ms)")
    plt.xlabel("Time (sec)")
    plt.ylabel("Latency (ms)")
    plt.title("DDS Latency Over Time")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig("dds_latency_plot.png")
    print("[SAVED] dds_latency_plot.png")

    # ───────────────────────────────────────────────
    # 4. Plot jitter curve
    # ───────────────────────────────────────────────
    if len(jitter) > 0:
        plt.figure(figsize=(10,5))
        plt.plot(times[1:], jitter, label="Jitter (ms)", color="orange")
        plt.xlabel("Time (sec)")
        plt.ylabel("Jitter (ms)")
        plt.title("DDS Jitter Over Time")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.savefig("dds_jitter_plot.png")
        print("[SAVED] dds_jitter_plot.png")


def main(args=None):
    rclpy.init(args=args)
    node = DDSLatencyLogger()

    try:
        rclpy.spin(node)
    except:
        pass

    save_results(node.log)
    node.destroy_node()


if __name__ == "__main__":
    main()
