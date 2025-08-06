import rclpy
from rclpy.node import Node
import time, json, csv
from datetime import datetime
from pathlib import Path
from subprocess import Popen, STDOUT, DEVNULL
import psutil
import matplotlib.pyplot as plt


def io_stats():
    if psutil:
        return psutil.net_io_counters()._asdict()
    stats = {'bytes_recv': 0, 'packets_recv': 0, 'dropin': 0,
             'bytes_sent': 0, 'packets_sent': 0, 'dropout': 0}
    with open('/proc/net/dev') as f:
        for line in f.readlines()[2:]:
            p = line.split()
            stats['bytes_recv']   += int(p[1])
            stats['packets_recv'] += int(p[2])
            stats['dropin']       += int(p[4])
            stats['bytes_sent']   += int(p[9])
            stats['packets_sent'] += int(p[10])
            stats['dropout']      += int(p[12])
    return stats


def launch_background_load(target_ip: str, load: str):
    if load == 'B':  # 5–10 Mbps
        return Popen(['iperf3', '-c', target_ip, '-u', '-b', '8M', '-t', '300', '-p', '5002'],
                     stdout=DEVNULL, stderr=DEVNULL)
    elif load == 'C':  # 120 Mbps
        return Popen(['iperf3', '-c', target_ip, '-u', '-b', '120M', '-t', '300', '-p', '5002'],
                     stdout=DEVNULL, stderr=DEVNULL)
    return None


def plot_summary(data_dir: Path, rtt_data: list):
    png = data_dir / 'summary_plot.png'
    plt.figure(figsize=(10, 5))
    plt.plot(rtt_data, label='RTT (ms)')
    plt.xlabel('Samples')
    plt.ylabel('RTT (ms)')
    plt.title('RTT Over Time')
    plt.legend()
    plt.grid(True)
    plt.savefig(png)
    return png


def extract_ping_rtt(ping_file: Path) -> list:
    rtts = []
    with open(ping_file) as f:
        for line in f:
            if 'time=' in line:
                try:
                    rtts.append(float(line.split('time=')[1].split()[0]))
                except ValueError:
                    pass
    return rtts


def append_to_csv(csv_path: Path, row: dict, headers: list):
    write_header = not csv_path.exists()
    with open(csv_path, 'a', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=headers)
        if write_header:
            writer.writeheader()
        writer.writerow(row)


class NetworkTester(Node):
    def __init__(self):
        super().__init__('network_tester')
        # Declare ROS parameters
        self.declare_parameter('target', '192.168.0.230')
        self.declare_parameter('duration', 10)
        self.declare_parameter('udp', False)
        self.declare_parameter('load', 'A')
        self.declare_parameter('out', '../output')

        # Read parameters
        self.target   = self.get_parameter('target').get_parameter_value().string_value
        self.duration = self.get_parameter('duration').get_parameter_value().integer_value
        self.udp      = self.get_parameter('udp').get_parameter_value().bool_value
        self.load     = self.get_parameter('load').get_parameter_value().string_value
        self.out_dir  = self.get_parameter('out').get_parameter_value().string_value

    def run(self):
        ts = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        base = Path(self.out_dir) / ts
        base.mkdir(parents=True, exist_ok=True)

        meta = {'target': self.target, 'duration': self.duration, 'timestamp': ts}
        meta['net_before'] = io_stats()

        bg_proc = launch_background_load(self.target, self.load)
        procs = []

        # 1. Ping
        cnt = self.duration * 5
        ping_f = base / f'ping_{ts}.txt'
        ping_out = open(ping_f, 'w')
        procs.append(('ping', ping_f,
                      Popen(['ping', '-i', '0.2', '-c', str(cnt), self.target],
                            stdout=ping_out, stderr=STDOUT)))
        meta['ping_file'] = str(ping_f)

        # 2. iperf3 TCP
        iperf_tcp_f = base / f'iperf_tcp_{ts}.txt'
        iperf_tcp_out = open(iperf_tcp_f, 'w')
        procs.append(('iperf_tcp', iperf_tcp_f,
                      Popen(['iperf3', '-c', self.target, '-t', str(self.duration)],
                            stdout=iperf_tcp_out, stderr=STDOUT)))
        meta['iperf_tcp'] = str(iperf_tcp_f)

        # 3. iperf3 UDP (optional)
        if self.udp:
            iperf_udp_f = base / f'iperf_udp_{ts}.txt'
            iperf_udp_out = open(iperf_udp_f, 'w')
            procs.append(('iperf_udp', iperf_udp_f,
                          Popen(['iperf3', '-u', '-b', '0', '-c', self.target, '-t', str(self.duration), '-p', '5002'],
                                stdout=iperf_udp_out, stderr=STDOUT)))
            meta['iperf_udp'] = str(iperf_udp_f)

        # 4. mtr
        mtr_f = base / f'mtr_{ts}.json'
        mtr_out = open(mtr_f, 'w')
        procs.append(('mtr', mtr_f,
                      Popen(['mtr', '-rwzc', '10', '-j', self.target],
                            stdout=mtr_out, stderr=STDOUT)))
        meta['mtr_file'] = str(mtr_f)

        # Wait for all
        for name, fpath, proc in procs:
            proc.wait()
            try:
                proc.stdout.close()
            except Exception:
                pass

        # RX/TX after
        meta['net_after'] = io_stats()

        # Flent RRUL plot
        rrul_base = base / f'rrul_{ts}'
        flent_cmd = [
            'flent', 'rrul',
            '-p', 'all_scaled', '-l', str(self.duration),
            '-s', '0.2', '--socket-stats',
            '-H', self.target,
            '-o', str(rrul_base) + '.png'
        ]
        Popen(flent_cmd).wait()
        gz = Path(str(rrul_base) + '.flent.gz')
        if gz.exists():
            gz.rename(base / gz.name)
            meta['flent_gz'] = str(base / gz.name)
        meta['flent_plot'] = str(rrul_base) + '.png'

        # Completion and meta write
        meta['completion_sec'] = round(
            time.time() - time.mktime(datetime.strptime(ts, '%Y-%m-%d_%H-%M-%S').timetuple()), 2)
        (base / f'meta_{ts}.json').write_text(json.dumps(meta, indent=2))

        if bg_proc:
            bg_proc.terminate()

        # RTT average and summary plot
        rtt_list = extract_ping_rtt(ping_f)
        meta['rtt_avg'] = round(sum(rtt_list) / len(rtt_list), 2) if rtt_list else None
        plot_summary(base, rtt_list)

        # Append to summary.csv
        summary_csv = Path(self.out_dir) / 'summary.csv'
        append_to_csv(summary_csv, {
            'timestamp': ts,
            'target': self.target,
            'duration': self.duration,
            'load': self.load,
            'rtt_avg': meta['rtt_avg'],
            'rx_MB': round((meta['net_after']['bytes_recv'] - meta['net_before']['bytes_recv']) / 1024 / 1024, 2),
            'tx_MB': round((meta['net_after']['bytes_sent'] - meta['net_before']['bytes_sent']) / 1024 / 1024, 2),
            'completion_sec': meta['completion_sec'],
            'plot': meta['flent_plot']
        }, headers=[
            'timestamp', 'target', 'duration', 'load', 'rtt_avg',
            'rx_MB', 'tx_MB', 'completion_sec', 'plot'
        ])

        self.get_logger().info(f"✅ All done. Results in: {base}")


def main(args=None):
    rclpy.init(args=args)
    node = NetworkTester()
    node.run()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
