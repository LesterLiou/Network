#!/usr/bin/env python3
"""
network_image.py — Unified network tester with per-topic bandwidth
Now supports BOTH:
  (A) ROS 2 DDS subscriptions (rclpy)
  (B) rosbridge WebSocket subscriptions (roslibpy)

Extra params for rosbridge mode:
  - use_rosbridge: 0|1 (default 0)
  - ws_host: rosbridge host/IP (default 127.0.0.1)
  - ws_port: rosbridge port (default 9090)

'types'  (sensor_msgs/msg/JointState)；
For CompressedImage, we decode base64 and use decoded bytes for Mbps.


### DDS（ROS_DOMAIN_ID / DDS ）
ros2 run network_tester network_rosbridge \
  --ros-args \
  -p target:=192.168.0.230 \
  -p duration:=15 \
  -p load:=B \
  -p out:="/root/NETWORK/output" \
  -p topics:="/left_follower/joint_states" \
  -p types:="sensor_msgs/msg/JointState" \
  -p bw_interval:=1


### rosbridge（WebSocket）
ros2 run network_tester network_rosbridge \
  --ros-args \
  -p target:=192.168.0.230 \
  -p duration:=15 \
  -p load:=B \
  -p out:="/root/NETWORK/output" \
  -p topics:="/left_follower/joint_states" \
  -p types:="sensor_msgs/msg/JointState" \
  -p bw_interval:=1 \
  -p use_rosbridge:=1 \
  -p ws_host:=192.168.0.230 \
  -p ws_port:=9090

"""
import time
import json
import csv
import base64
from datetime import datetime
from pathlib import Path
from subprocess import Popen, STDOUT

import psutil
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message

# rosbridge (lazy import; only used when enabled)
try:
    import roslibpy
except Exception:
    roslibpy = None


def annotate_series_stats(ax, series, unit="", where="upper right", show_hlines=True, show_box=False):

    if not series:
        return
    smax = max(series)
    smin = min(series)
    savg = sum(series) / len(series)

    if show_box:
        txt = f"Max: {smax:.3f} {unit}\nMin: {smin:.3f} {unit}\nAvg: {savg:.3f} {unit}"
        loc_map = {
            "upper right":  (0.98, 0.98, "right",  "top"),
            "upper left":   (0.02, 0.98, "left",   "top"),
            "lower right":  (0.98, 0.02, "right",  "bottom"),
            "lower left":   (0.02, 0.02, "left",   "bottom"),
        }
        x, y, ha, va = loc_map.get(where, loc_map["upper right"])
        ax.text(x, y, txt, transform=ax.transAxes, ha=ha, va=va, fontsize=9,
                bbox=dict(boxstyle="round", facecolor="white", alpha=0.75, edgecolor="gray"))

    if show_hlines:
        ax.axhline(savg, linestyle="--", linewidth=1, alpha=0.8, label=f"Avg {savg:.3f} {unit}")
        ax.axhline(smax, linestyle=":",  linewidth=1, alpha=0.6, label=f"Max {smax:.3f} {unit}")
        ax.axhline(smin, linestyle=":",  linewidth=1, alpha=0.6, label=f"Min {smin:.3f} {unit}")
        # show legend
        ax.legend(loc="best")   # change to 'upper left' / 'upper right' is ok

def io_stats():
    if psutil:
        return psutil.net_io_counters()._asdict()
    # Fallback
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


def ros2_to_ros1_type(t: str) -> str:
    # "sensor_msgs/msg/JointState" -> "sensor_msgs/JointState"
    return t.replace('/msg/', '/')


class NetworkImageNode(Node):
    def __init__(self):
        super().__init__('network_rosbridge')

        # Core params
        self.declare_parameter('target', '--')
        self.declare_parameter('duration', 15)
        self.declare_parameter('load', 'A')
        self.declare_parameter('out', '../output')

        self.target = self.get_parameter('target').get_parameter_value().string_value
        self.duration = int(self.get_parameter('duration').get_parameter_value().integer_value)
        self.load = self.get_parameter('load').get_parameter_value().string_value
        self.out_dir = self.get_parameter('out').get_parameter_value().string_value

        # Per-topic monitor params
        self.declare_parameter('topics', '')
        self.declare_parameter('types', '')
        self.declare_parameter('bw_interval', 1)

        self.topic_names = [t.strip() for t in self.get_parameter('topics').get_parameter_value().string_value.split(',') if t.strip()]
        self.type_names  = [t.strip() for t in self.get_parameter('types').get_parameter_value().string_value.split(',') if t.strip()]
        self.bw_interval = int(self.get_parameter('bw_interval').get_parameter_value().integer_value or 1)

        if self.topic_names and (len(self.topic_names) != len(self.type_names)):
            raise ValueError("Parameter 'topics' and 'types' must have the same length.")

        # rosbridge params
        self.declare_parameter('use_rosbridge', 0)
        self.declare_parameter('ws_host', '127.0.0.1')
        self.declare_parameter('ws_port', 9090)

        self.use_rosbridge = bool(int(self.get_parameter('use_rosbridge').get_parameter_value().integer_value))
        self.ws_host = self.get_parameter('ws_host').get_parameter_value().string_value
        self.ws_port = int(self.get_parameter('ws_port').get_parameter_value().integer_value)

        # DDS QoS for sensors
        self.sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        # Runtime fields for monitor
        self._topic_bytes: dict[str, int] = {}
        self._topic_msgs: dict[str, int] = {}
        self._subs = []             # DDS subscriptions
        self._bw_csv_path: Path | None = None
        self._bw_timer = None       # DDS timer
        self._rb_client = None      # rosbridge client
        self._rb_topics = []        # rosbridge Topic objects

    # ---------------- Per-topic bandwidth monitor (DDS) -----------------
    def _setup_topic_monitors_dds(self, out_dir: Path):
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
                self.get_logger().error(f"[DDS] Failed to load type {type_name} for {name}: {e}")
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
            self.get_logger().info(f"[DDS][BW] Monitoring {name} ({type_name})")

        def _tick():
            self._write_bw_row()

        self._bw_timer = self.create_timer(float(self.bw_interval), _tick)

    # ---------------- Per-topic bandwidth monitor (rosbridge) -----------
    def _setup_topic_monitors_rosbridge(self, out_dir: Path):
        if not self.topic_names:
            return
        if roslibpy is None:
            raise RuntimeError("roslibpy not installed. `pip install roslibpy`")

        self._bw_csv_path = out_dir / 'ros2_bw.csv'

        for name in self.topic_names:
            self._topic_bytes[name] = 0
            self._topic_msgs[name] = 0

        self._rb_client = roslibpy.Ros(host=self.ws_host, port=self.ws_port)
        self._rb_client.run()
        if not self._rb_client.is_connected:
            raise RuntimeError(f"Cannot connect to rosbridge ws://{self.ws_host}:{self.ws_port}")
        self.get_logger().info(f"[RB] Connected ws://{self.ws_host}:{self.ws_port}")

        for name, type_name in zip(self.topic_names, self.type_names):
            t_ros1 = ros2_to_ros1_type(type_name)
            topic = roslibpy.Topic(self._rb_client, name, t_ros1)

            def make_cb(tname: str, ttype: str):
                def _cb(msg: dict):
                    # payload bytes estimation
                    by = 0
                    try:
                        if ttype.endswith('CompressedImage') or ttype.endswith('/CompressedImage'):
                            by = len(base64.b64decode(msg.get('data', ''), validate=False))
                        else:
                            by = len(json.dumps(msg, separators=(',', ':')).encode('utf-8'))
                    except Exception:
                        by = 0
                    self._topic_bytes[tname] += by
                    self._topic_msgs[tname] += 1
                return _cb

            topic.subscribe(make_cb(name, t_ros1))
            self._rb_topics.append(topic)
            self.get_logger().info(f"[RB][BW] Monitoring {name} ({t_ros1})")

    # ---------------- Common writer ----------------
    def _write_bw_row(self):
        if not self._bw_csv_path:
            return
        now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        row = {'timestamp': now}
        for t in self.topic_names:
            by = self._topic_bytes.get(t, 0)
            ms = self._topic_msgs.get(t, 0)
            mbps = (by * 8.0) / 1e6 / self.bw_interval if self.bw_interval > 0 else 0.0
            hz   = ms / self.bw_interval if self.bw_interval > 0 else 0.0
            avg  = (by / ms) if ms > 0 else 0.0
            row[f'{t}:Mbps'] = round(mbps, 3)
            row[f'{t}:Hz'] = round(hz, 2)
            row[f'{t}:AvgBytes'] = int(avg)
            # reset window
            self._topic_bytes[t] = 0
            self._topic_msgs[t] = 0

        # write
        write_header = not self._bw_csv_path.exists()
        with open(self._bw_csv_path, 'a', newline='') as f:
            fieldnames = list(row.keys())
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            if write_header:
                writer.writeheader()
            writer.writerow(row)

    def _teardown_topic_monitors(self):
        # DDS timer
        if self._bw_timer:
            self._bw_timer.cancel()
            self._bw_timer = None
        # rosbridge topics
        for t in self._rb_topics:
            try:
                t.unsubscribe()
            except Exception:
                pass
        self._rb_topics.clear()
        if self._rb_client:
            try:
                self._rb_client.terminate()
            except Exception:
                pass
            self._rb_client = None

    # ------------------------------- Main -------------------------------
    def run(self):
        # Load presets for iperf3 load
        load_map = {'A': {'streams': 1, 'bandwidth': '0'},
                    'B': {'streams': 5, 'bandwidth': '2M'},
                    'C': {'streams': 5, 'bandwidth': '24M'}}
        cfg = load_map.get(self.load, load_map['A'])

        ts = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        base = Path(self.out_dir) / ts
        base.mkdir(parents=True, exist_ok=True)

        start_time = time.time()
        meta = {'target': self.target, 'duration': self.duration,
                'load': self.load, 'timestamp': ts}
        meta['net_before'] = io_stats()

        # External tools
        procs = []
        count = self.duration * 5
        ping_f = base / f"ping_{ts}.txt"
        p = Popen(['ping', '-i', '0.2', '-c', str(count), self.target], stdout=open(ping_f, 'w'), stderr=STDOUT)
        procs.append(('ping', ping_f, p))

        iperf_f = base / f"iperf_tcp_{ts}.json"
        cmd = ['iperf3', '-c', self.target, '-t', str(self.duration), '-P', str(cfg['streams']), '-b', cfg['bandwidth'], '-J']
        p = Popen(cmd, stdout=open(iperf_f, 'w'), stderr=STDOUT)
        procs.append(('iperf_tcp', iperf_f, p))

        mtr_f = base / f"mtr_{ts}.json"
        p = Popen(['mtr', '-rwzc', '10', '-j', self.target], stdout=open(mtr_f, 'w'), stderr=STDOUT)
        procs.append(('mtr', mtr_f, p))

        # ---- Start per-topic monitor ----
        if self.topic_names:
            if self.use_rosbridge:
                self._setup_topic_monitors_rosbridge(base)
            else:
                self._setup_topic_monitors_dds(base)

        # ---- Spin / tick for duration ----
        deadline = time.time() + self.duration
        next_tick = time.time() + float(self.bw_interval)
        while time.time() < deadline:
            # DDS spin (fast)
            rclpy.spin_once(self, timeout_sec=0.05)
            # Manual tick for rosbridge mode (DDS mode用timer會自己寫)
            if self.use_rosbridge and time.time() >= next_tick:
                self._write_bw_row()
                next_tick += float(self.bw_interval)

        # Wait external tools
        for _, _, proc in procs:
            if proc.poll() is None:
                proc.wait()

        self._teardown_topic_monitors()

        # ---- Post-process (same as原版) ----
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
            Popen(['flent', 'rrul', '-p', 'all_scaled', '-l', str(self.duration), '-s', '0.2',
                   '--socket-stats', '-H', self.target, '-o', str(rr) + '.png']).wait()
            meta['flent_plot'] = str(rr) + '.png'
        except FileNotFoundError:
            self.get_logger().warn("Flent not found; skipping RRUL plot.")

        # Charts
        if rtts:
            plt.figure(figsize=(8,4))
            ax = plt.gca()
            ax.plot(rtts, label='RTT (ms)')
            ax.set_xlabel('Sample'); ax.set_ylabel('RTT (ms)')
            ax.set_title('Ping RTT Over Time'); ax.grid(True)
            annotate_series_stats(ax, rtts, unit="ms", where="upper right", show_hlines=True)
            f = base / 'chart_rtt.png'
            plt.tight_layout(); plt.savefig(f); plt.close()
            meta['chart_rtt'] = str(f)

            jitter = [abs(rtts[i]-rtts[i-1]) for i in range(1,len(rtts))]
            plt.figure(figsize=(8,4))
            ax = plt.gca()
            ax.plot(range(1,len(rtts)), jitter, label='Jitter (ms)')
            ax.set_xlabel('Sample'); ax.set_ylabel('Jitter (ms)')
            ax.set_title('RTT Jitter Over Time'); ax.grid(True)
            annotate_series_stats(ax, jitter, unit="ms", where="upper right", show_hlines=True)
            f = base / 'chart_jitter.png'
            plt.tight_layout(); plt.savefig(f); plt.close()
            meta['chart_jitter']=str(f)

        if intervals:
            times = [(i['sum']['start']+i['sum']['end'])/2 for i in intervals]
            tps = [i['sum']['bits_per_second']/1e6 for i in intervals]  # Mbps
            plt.figure(figsize=(8,4))
            ax = plt.gca()
            ax.plot(times, tps, label='Throughput (Mbps)')
            ax.set_xlabel('Time (s)'); ax.set_ylabel('Throughput (Mbps)')
            ax.set_title('TCP Throughput Over Time'); ax.grid(True)
            annotate_series_stats(ax, tps, unit="Mbps", where="upper right", show_hlines=True)
            f = base / 'chart_throughput.png'
            plt.tight_layout(); plt.savefig(f); plt.close()
            meta['chart_throughput'] = str(f)

        hops = meta.get('route', [])
        lats = meta.get('step_latencies_ms', [])
        if hops and lats:
            plt.figure(figsize=(10,5))
            ax = plt.gca()
            ax.plot(lats, marker='o')
            ax.set_xticks(range(len(hops)))
            ax.set_xticklabels(hops, rotation=45, ha='right')
            ax.set_xlabel('Hop'); ax.set_ylabel('Latency (ms)')
            ax.set_title('Step-wise Latency per Hop'); ax.grid(True)
            annotate_series_stats(ax, lats, unit="ms", where="upper right", show_hlines=True)
            f = base / 'chart_mtr.png'
            plt.tight_layout(); plt.savefig(f); plt.close()

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
                        values = df[col].tolist()
                        plt.figure(figsize=(8,4))
                        ax = plt.gca()
                        ax.plot(values, label=f'{t} Mbps')
                        ax.set_xlabel('Window (#)'); ax.set_ylabel('Mbps')
                        ax.set_title(f'Bandwidth over time - {t}')
                        ax.grid(True)
                        annotate_series_stats(ax, values, unit="Mbps", where="upper right", show_hlines=True)
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
