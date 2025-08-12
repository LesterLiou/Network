#!/usr/bin/env python3
"""
network_image.py — Unified ROS 2 network tester with per-topic bandwidth
-----------------------------------------------------------------------
Features:
 1) Ping RTT sampling (0.2s) with stats & charts
 2) iperf3 TCP throughput (JSON) with chart
 3) MTR hop-by-hop latency (JSON) with chart
 4) Flent RRUL plot (all_scaled)
 5) System-level RX/TX delta from /proc/net/dev
 6) **Per-topic ROS 2 bandwidth monitor** (Mbps/Hz/AvgBytes per window)
    - Works for 1~N image streams + any other topics (e.g., JointState)
    - Uses serialize_message(msg) to estimate payload size
 7) Outputs: structured PNG charts, meta_<ts>.json, summary.csv, ros2_bw.csv

Parameters (ros2 --ros-args -p ...):
  - target:      iperf3/Flent target host/IP
  - duration:    total seconds to measure (affects ping/iperf/flent & spin)
  - load:        A|B|C -> maps to iperf3 streams & bandwidth
  - out:         base output directory
  - topics:      comma list of topic names (e.g. "/cam1/image_raw,/cam2/image_raw,/joint_states")
  - types:       comma list of type names  (e.g. "sensor_msgs/msg/Image,...,sensor_msgs/msg/JointState")
  - bw_interval: window seconds for per-topic CSV aggregation (default 1)

Example:
ros2 run network_tester network_image \
  --ros-args \
  -p target:=192.168.0.5 \
  -p duration:=15 \
  -p load:=B \
  -p out:="../output" \
  -p topics:="/top/image_raw,/first/image_raw,/joint_states" \
  -p types:="sensor_msgs/msg/Image,sensor_msgs/msg/Image,sensor_msgs/msg/JointState" \
  -p bw_interval:=1

Notes:
- For CompressedImage streams, set the type to sensor_msgs/msg/CompressedImage
  to reflect actual network footprint after compression.
- QoS for sensors uses BEST_EFFORT by default here; adjust as needed.
"""

import time
import json
import csv
from datetime import datetime
from pathlib import Path
from subprocess import Popen, STDOUT

import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message

import psutil
import matplotlib.pyplot as plt


def io_stats():
    if psutil:
        return psutil.net_io_counters()._asdict()
    # Fallback (rarely used)
    stats = dict(bytes_recv=0, packets_recv=0, dropin=0,
                 bytes_sent=0, packets_sent=0, dropout=0)
    with open("/proc/net/dev") as f:
        for line in f.readlines()[2:]:
            p = line.split()
            stats['bytes_recv']   += int(p[1])
            stats['packets_recv'] += int(p[2])
            stats['dropin']       += int(p[4])
            stats['bytes_sent']   += int(p[9])
            stats['packets_sent'] += int(p[10])
            stats['dropout']      += int(p[12])
    return stats


def append_to_csv(csv_path: Path, row: dict, headers: list[str]):
    write_header = not csv_path.exists()
    with open(csv_path, "a", newline='') as f:
        writer = csv.DictWriter(f, fieldnames=headers)
        if write_header:
            writer.writeheader()
        writer.writerow(row)


class NetworkImageNode(Node):
    def __init__(self):
        super().__init__('network_image')
        # Core params
        self.declare_parameter('target', '--')
        self.declare_parameter('duration', 15)
        self.declare_parameter('load', 'A')
        self.declare_parameter('out', '../output')

        self.target = self.get_parameter('target').get_parameter_value().string_value
        self.duration = int(self.get_parameter('duration').get_parameter_value().integer_value)
        self.load = self.get_parameter('load').get_parameter_value().string_value
        self.out_dir = self.get_parameter('out').get_parameter_value().string_value

        # Per-topic bandwidth monitor params
        self.declare_parameter('topics', '')
        self.declare_parameter('types', '')
        self.declare_parameter('bw_interval', 1)

        self.topic_names = [t.strip() for t in self.get_parameter('topics').get_parameter_value().string_value.split(',') if t.strip()]
        self.type_names  = [t.strip() for t in self.get_parameter('types').get_parameter_value().string_value.split(',') if t.strip()]
        self.bw_interval = int(self.get_parameter('bw_interval').get_parameter_value().integer_value or 1)

        if self.topic_names and (len(self.topic_names) != len(self.type_names)):
            raise ValueError("Parameter 'topics' and 'types' must have the same length.")

        self.sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        # Runtime fields for monitor
        self._topic_bytes: dict[str, int] = {}
        self._topic_msgs: dict[str, int] = {}
        self._subs = []
        self._bw_csv_path: Path | None = None
        self._bw_timer = None

    # ---------------- Per-topic bandwidth monitor -----------------
    def _setup_topic_monitors(self, out_dir: Path):
        if not self.topic_names:
            return
        self._bw_csv_path = out_dir / 'ros2_bw.csv'

        for name in self.topic_names:
            self._topic_bytes[name] = 0
            self._topic_msgs[name] = 0

        for name, type_name in zip(self.topic_names, self.type_names):
            try:
                MsgType = get_message(type_name)
            except Exception as e:
                self.get_logger().error(f"Failed to load type {type_name} for {name}: {e}")
                continue

            def make_cb(tname: str):
                def _cb(msg):
                    try:
                        sz = len(serialize_message(msg))
                    except Exception:
                        sz = 0
                    self._topic_bytes[tname] += sz
                    self._topic_msgs[tname] += 1
                return _cb

            sub = self.create_subscription(MsgType, name, make_cb(name), self.sensor_qos)
            self._subs.append(sub)
            self.get_logger().info(f"[BW] Monitoring {name} ({type_name})")

        def _tick():
            if not self._bw_csv_path:
                return
            now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            row = {'timestamp': now}
            for t in self.topic_names:
                by = self._topic_bytes[t]
                ms = self._topic_msgs[t]
                mbps = (by * 8.0) / 1e6 / self.bw_interval if self.bw_interval > 0 else 0.0
                hz   = ms / self.bw_interval if self.bw_interval > 0 else 0.0
                avg  = (by / ms) if ms > 0 else 0.0
                row[f'{t}:Mbps'] = round(mbps, 3)
                row[f'{t}:Hz'] = round(hz, 2)
                row[f'{t}:AvgBytes'] = int(avg)
                # reset window
                self._topic_bytes[t] = 0
                self._topic_msgs[t] = 0

            write_header = not self._bw_csv_path.exists()
            with open(self._bw_csv_path, 'a', newline='') as f:
                fieldnames = list(row.keys())
                writer = csv.DictWriter(f, fieldnames=fieldnames)
                if write_header:
                    writer.writeheader()
                writer.writerow(row)

        self._bw_timer = self.create_timer(float(self.bw_interval), _tick)

    def _teardown_topic_monitors(self):
        if self._bw_timer:
            self._bw_timer.cancel()
            self._bw_timer = None
        # (Optionally destroy subscriptions)

    # ------------------------------- Main -------------------------------
    def run(self):
        # Load presets for iperf3 load
        load_map = {
            'A': {'streams': 1, 'bandwidth': '0'},
            'B': {'streams': 5, 'bandwidth': '2M'},
            'C': {'streams': 5, 'bandwidth': '24M'},
        }
        cfg = load_map.get(self.load, load_map['A'])

        ts = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        base = Path(self.out_dir) / ts
        base.mkdir(parents=True, exist_ok=True)

        start_time = time.time()
        meta = {
            'target': self.target,
            'duration': self.duration,
            'load': self.load,
            'timestamp': ts
        }
        meta['net_before'] = io_stats()

        procs = []
        # 1) Ping
        count = self.duration * 5
        ping_f = base / f"ping_{ts}.txt"
        p = Popen(['ping', '-i', '0.2', '-c', str(count), self.target], stdout=open(ping_f, 'w'), stderr=STDOUT)
        procs.append(('ping', ping_f, p))

        # 2) iperf3 TCP (JSON)
        iperf_f = base / f"iperf_tcp_{ts}.json"
        cmd = ['iperf3', '-c', self.target, '-t', str(self.duration), '-P', str(cfg['streams']), '-b', cfg['bandwidth'], '-J']
        p = Popen(cmd, stdout=open(iperf_f, 'w'), stderr=STDOUT)
        procs.append(('iperf_tcp', iperf_f, p))

        # 3) MTR JSON
        mtr_f = base / f"mtr_{ts}.json"
        p = Popen(['mtr', '-rwzc', '10', '-j', self.target], stdout=open(mtr_f, 'w'), stderr=STDOUT)
        procs.append(('mtr', mtr_f, p))

        # ---- Start ROS 2 per-topic monitor & spin for duration ----
        self._setup_topic_monitors(base)
        deadline = time.time() + self.duration
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

        # Ensure external tools finished (or wait if still running)
        for _, _, proc in procs:
            if proc.poll() is None:
                proc.wait()

        self._teardown_topic_monitors()

        # ---- Post-process ----
        net_after = io_stats()
        rx = net_after['bytes_recv'] - meta['net_before']['bytes_recv']
        tx = net_after['bytes_sent'] - meta['net_before']['bytes_sent']

        # Parse ping
        rtts = []
        with open(ping_f) as f:
            for line in f:
                if 'time=' in line:
                    try:
                        rtts.append(float(line.split('time=')[1].split()[0]))
                    except Exception:
                        pass
        received = len(rtts)
        loss = round((count - received) / count * 100, 2)
        meta['rtt_avg_ms'] = round(sum(rtts)/len(rtts), 2) if rtts else None
        meta['packet_loss%'] = loss
        meta['bufferbloat_ms'] = round(max(rtts)-min(rtts), 2) if rtts else None

        # Parse MTR
        try:
            mj = json.load(open(mtr_f))
            hubs = mj['report']['hubs']
            meta['route'] = [h.get('host', '') for h in hubs]
            meta['step_latencies_ms'] = [h.get('Last', 0.0) for h in hubs]
        except Exception:
            meta['route'] = []
            meta['step_latencies_ms'] = []

        # Parse iperf3
        try:
            ij = json.load(open(iperf_f))
            mbps = ij['end']['sum_received']['bits_per_second'] / 1e6
            meta['bandwidth_Mbps'] = round(mbps, 2)
            intervals = ij.get('intervals', [])
        except Exception:
            meta['bandwidth_Mbps'] = None
            intervals = []

        meta['rx_MB'] = round(rx/1024/1024, 2)
        meta['tx_MB'] = round(tx/1024/1024, 2)
        meta['completion_sec'] = round(time.time() - start_time, 2)

        # Flent RRUL plot
        rr = base / f"rrul_{ts}"
        try:
            Popen(['flent', 'rrul', '-p', 'all_scaled', '-l', str(self.duration), '-s', '0.2', '--socket-stats', '-H', self.target, '-o', str(rr) + '.png']).wait()
            meta['flent_plot'] = str(rr) + '.png'
        except FileNotFoundError:
            self.get_logger().warn("Flent not found; skipping RRUL plot.")

        # Charts
        if rtts:
            plt.figure(figsize=(8,4))
            plt.plot(rtts, label='RTT (ms)')
            plt.xlabel('Sample'); plt.ylabel('RTT (ms)')
            plt.title('Ping RTT Over Time'); plt.grid(); plt.legend()
            f = base / 'chart_rtt.png'; plt.savefig(f); plt.close(); meta['chart_rtt'] = str(f)

            jitter = [abs(rtts[i]-rtts[i-1]) for i in range(1,len(rtts))]
            plt.figure(figsize=(8,4))
            plt.plot(range(1,len(rtts)), jitter, label='Jitter (ms)')
            plt.xlabel('Sample'); plt.ylabel('Jitter (ms)')
            plt.title('RTT Jitter Over Time'); plt.grid(); plt.legend()
            f = base / 'chart_jitter.png'; plt.savefig(f); plt.close(); meta['chart_jitter']=str(f)

        if intervals:
            times = [(i['sum']['start']+i['sum']['end'])/2 for i in intervals]
            tps = [i['sum']['bits_per_second']/1e6 for i in intervals]
            plt.figure(figsize=(8,4))
            plt.plot(times, tps, label='Throughput (Mbps)')
            plt.xlabel('Time (s)'); plt.ylabel('Throughput (Mbps)')
            plt.title('TCP Throughput Over Time'); plt.grid(); plt.legend()
            f = base / 'chart_throughput.png'; plt.savefig(f); plt.close(); meta['chart_throughput'] = str(f)

        hops = meta.get('route', [])
        lats = meta.get('step_latencies_ms', [])
        if hops and lats:
            plt.figure(figsize=(10,5))
            plt.plot(lats, marker='o')
            plt.xticks(range(len(hops)), hops, rotation=45, ha='right')
            plt.xlabel('Hop'); plt.ylabel('Latency (ms)')
            plt.title('Step-wise Latency per Hop'); plt.grid()
            f = base / 'chart_mtr.png'; plt.tight_layout(); plt.savefig(f); plt.close(); meta['chart_mtr'] = str(f)
            meta['mtr_start_ms'] = float(lats[0]) if lats else None
            meta['mtr_end_ms'] = float(lats[-1]) if lats else None
            meta['mtr_diff_ms'] = round(float(lats[-1]) - float(lats[0]), 2) if len(lats) >= 2 else None

        # Save meta json
        (base / f'meta_{ts}.json').write_text(json.dumps(meta, indent=2))

        # Append summary.csv
        summary_csv = Path(self.out_dir) / 'summary.csv'
        row = {
            'timestamp': ts,
            'target': self.target,
            'load': self.load,
            'rtt_avg': meta.get('rtt_avg_ms'),
            'loss_%': meta.get('packet_loss%'),
            'bw_Mbps': meta.get('bandwidth_Mbps'),
            'rx_MB': meta.get('rx_MB'),
            'tx_MB': meta.get('tx_MB'),
            'bufferbloat_ms': meta.get('bufferbloat_ms'),
            'completion_s': meta.get('completion_sec'),
            'mtr_diff_ms': meta.get('mtr_diff_ms')
        }
        append_to_csv(summary_csv, row, list(row.keys()))

        # Optional: plot per-topic bandwidth from ros2_bw.csv
        try:
            import pandas as pd
            bw_csv = base / 'ros2_bw.csv'
            if bw_csv.exists():
                df = pd.read_csv(bw_csv)
                for t in self.topic_names:
                    col = f'{t}:Mbps'
                    if col in df.columns:
                        plt.figure(figsize=(8,4))
                        plt.plot(df[col])
                        plt.xlabel('Window (#)'); plt.ylabel('Mbps')
                        plt.title(f'Bandwidth over time - {t}')
                        plt.grid(True)
                        outp = base / f'bw_{t.replace("/", "_")}.png'
                        plt.tight_layout(); plt.savefig(outp); plt.close()
        except Exception as e:
            self.get_logger().warn(f"BW plot failed: {e}")

        self.get_logger().info(f"✅ All done. Results in: {base}")


def main(args=None):
    rclpy.init(args=args)
    node = NetworkImageNode()
    try:
        node.run()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
