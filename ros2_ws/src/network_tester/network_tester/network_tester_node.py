"""
ros2 run network_tester network_test \
  --ros-args \
  -p target:=<ip> \
  -p duration:=15 \
  -p load:=B \
  -p out:=\"../output\"

"""
import rclpy
from rclpy.node import Node
import time
import json
import csv
from datetime import datetime
from pathlib import Path
from subprocess import Popen, STDOUT, DEVNULL
import psutil
import matplotlib.pyplot as plt


def io_stats():
    if psutil:
        return psutil.net_io_counters()._asdict()
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


def append_to_csv(csv_path, row, headers):
    write_header = not csv_path.exists()
    with open(csv_path, "a", newline='') as f:
        writer = csv.DictWriter(f, fieldnames=headers)
        if write_header:
            writer.writeheader()
        writer.writerow(row)


class NetworkTestNode(Node):
    def __init__(self):
        super().__init__('network_test')
        # Declare ROS2 parameters
        self.declare_parameter('target', '--')
        self.declare_parameter('duration', 15)
        self.declare_parameter('load', 'A')
        self.declare_parameter('out', '../output')

        # Read parameters
        self.target = self.get_parameter('target').get_parameter_value().string_value
        self.duration = self.get_parameter('duration').get_parameter_value().integer_value
        self.load = self.get_parameter('load').get_parameter_value().string_value
        self.out_dir = self.get_parameter('out').get_parameter_value().string_value

    def run(self):
        # Map load to streams & bandwidth
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
        # 1. Ping
        count = self.duration * 5
        ping_f = base / f"ping_{ts}.txt"
        p = Popen([
            'ping', '-i', '0.2', '-c', str(count), self.target
        ], stdout=open(ping_f, 'w'), stderr=STDOUT)
        procs.append(('ping', ping_f, p))

        # 2. iperf3 TCP
        iperf_f = base / f"iperf_tcp_{ts}.json"
        cmd = [
            'iperf3', '-c', self.target,
            '-t', str(self.duration),
            '-P', str(cfg['streams']),
            '-b', cfg['bandwidth'],
            '-J'
        ]
        p = Popen(cmd, stdout=open(iperf_f, 'w'), stderr=STDOUT)
        procs.append(('iperf_tcp', iperf_f, p))

        # 3. MTR
        mtr_f = base / f"mtr_{ts}.json"
        p = Popen([
            'mtr', '-rwzc', '10', '-j', self.target
        ], stdout=open(mtr_f, 'w'), stderr=STDOUT)
        procs.append(('mtr', mtr_f, p))

        # Wait all
        for _, _, proc in procs:
            proc.wait()
        end_time = time.time()

        # Post process
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
                    except:
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
            meta['route'] = [h['host'] for h in hubs]
            meta['step_latencies_ms'] = [h['Last'] for h in hubs]
        except:
            meta['route'] = []
            meta['step_latencies_ms'] = []

        # Parse iperf
        try:
            ij = json.load(open(iperf_f))
            mbps = ij['end']['sum_received']['bits_per_second'] / 1e6
            meta['bandwidth_Mbps'] = round(mbps, 2)
            intervals = ij.get('intervals', [])
        except:
            meta['bandwidth_Mbps'] = None
            intervals = []

        meta['rx_MB'] = round(rx/1024/1024, 2)
        meta['tx_MB'] = round(tx/1024/1024, 2)
        meta['completion_sec'] = round(end_time - start_time, 2)

        # Flent
        rr = base / f"rrul_{ts}"
        Popen([
            'flent', 'rrul',
            '-p', 'all_scaled', '-l', str(self.duration),
            '-s', '0.2', '--socket-stats',
            '-H', self.target,
            '-o', str(rr) + '.png'
        ]).wait()
        meta['flent_plot'] = str(rr) + '.png'

        # Plot charts
        if rtts:
            # RTT
            plt.figure(figsize=(8,4))
            plt.plot(rtts, label='RTT (ms)')
            plt.xlabel('Sample'); plt.ylabel('RTT (ms)')
            plt.title('Ping RTT Over Time'); plt.grid(); plt.legend()
            f = base / 'chart_rtt.png'; plt.savefig(f); plt.close(); meta['chart_rtt'] = str(f)
            # Jitter
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
        hops = meta.get('route', []); lats = meta.get('step_latencies_ms', [])
        if hops and lats:
            plt.figure(figsize=(10,5))
            plt.plot(lats, marker='o'); plt.xticks(range(len(hops)), hops, rotation=45)
            plt.xlabel('Hop'); plt.ylabel('Latency (ms)')
            plt.title('Step-wise Latency per Hop'); plt.grid()
            f = base / 'chart_mtr.png'; plt.tight_layout(); plt.savefig(f); plt.close(); meta['chart_mtr'] = str(f)
            meta['mtr_start_ms'] = lats[0]; meta['mtr_end_ms'] = lats[-1]
            meta['mtr_diff_ms'] = round(lats[-1]-lats[0],2)

        # Save meta
        (base / f'meta_{ts}.json').write_text(json.dumps(meta, indent=2))
        # Update summary.csv
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

        self.get_logger().info(f"✅ All done. Results in: {base}")


def main(args=None):
    rclpy.init(args=args)
    node = NetworkTestNode()
    node.run()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
