#!/usr/bin/env python3
"""
network_test_TCP.py  
--------------------------------------------------
同步執行：
 1. Ping RTT (interval=0.2s)
 2. iperf3 TCP 吞吐 (JSON 輸出 + 多條 parallel streams 作為 load)
 3. mtr 逐跳延遲 + routing (JSON)

收集：
 4. 前後 RX/TX 快照
 5. Flent RRUL 圖
 6. RTT / Jitter / Throughput / MTR 統計線圖
 7. meta_<timestamp>.json
 8. summary.csv（摘要）
--------------------------------------------------
Usage:
python3 network_test_TCP.py \
      --target 192.168.0.230 \
      --duration 15 \
      --load B \
      --out ../output
"""
import argparse
import time
import json
import csv
from datetime import datetime
from pathlib import Path
from subprocess import Popen, STDOUT, DEVNULL
import psutil
import matplotlib.pyplot as plt


def io_stats():
    # 系統網路 IO 統計
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


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--target",   required=True)
    parser.add_argument("--duration", type=int, default=15,
                        help="測試持續時間 (秒)")
    parser.add_argument("--out",      default="../output")
    parser.add_argument("--load",     choices=['A','B','C'], default='A',
                        help="測試負載：A none；B nominal 5-10Mbps；C nominal 120Mbps")
    args = parser.parse_args()

    # 對應 load -> parallel streams & 帶寬
    load_map = {
        'A': {'streams': 1, 'bandwidth': '0'},       # 無速率限制
        'B': {'streams': 5, 'bandwidth': '2M'},      # 5 x 2 = 10 Mbps
        'C': {'streams': 5, 'bandwidth': '24M'},     # 5 x 24 = 120 Mbps
    }
    cfg = load_map[args.load]

    # 準備輸出資料夾
    ts   = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    base = Path(args.out) / ts
    base.mkdir(parents=True, exist_ok=True)

    # 開始前統計
    start_time = time.time()
    meta = dict(target=args.target,
                duration=args.duration,
                load=args.load,
                timestamp=ts)
    meta['net_before'] = io_stats()

    procs = []
    # 1. Ping
    cnt    = args.duration * 5
    ping_f = base / f"ping_{ts}.txt"
    p = Popen([
        "ping", "-i", "0.2", "-c", str(cnt), args.target
    ], stdout=open(ping_f, "w"), stderr=STDOUT)
    procs.append(("ping", ping_f, p))

    # 2. iperf3 TCP (JSON + parallel)
    iperf_tcp_f = base / f"iperf_tcp_{ts}.json"
    cmd = [
        "iperf3", "-c", args.target,
        "-t", str(args.duration),
        "-P", str(cfg['streams']),
        "-b", cfg['bandwidth'],
        "-J"
    ]
    p = Popen(cmd, stdout=open(iperf_tcp_f, "w"), stderr=STDOUT)
    procs.append(("iperf_tcp", iperf_tcp_f, p))

    # 3. MTR
    mtr_f = base / f"mtr_{ts}.json"
    p = Popen([
        "mtr", "-rwzc", "10", "-j", args.target
    ], stdout=open(mtr_f, "w"), stderr=STDOUT)
    procs.append(("mtr", mtr_f, p))

    # 等待所有任務完成
    for _, _, proc in procs:
        proc.wait()
    end_time = time.time()

    # 後處理
    net_after = io_stats()
    rx = net_after['bytes_recv'] - meta['net_before']['bytes_recv']
    tx = net_after['bytes_sent'] - meta['net_before']['bytes_sent']

    # 解析 ping
    rtts = []
    with open(ping_f) as f:
        for line in f:
            if 'time=' in line:
                try:
                    rtts.append(float(line.split('time=')[1].split()[0]))
                except:
                    pass
    received = len(rtts)
    loss = round((cnt - received) / cnt * 100, 2)
    meta['rtt_avg_ms'] = round(sum(rtts)/len(rtts),2) if rtts else None
    meta['packet_loss%'] = loss
    meta['bufferbloat_ms'] = round(max(rtts)-min(rtts),2) if rtts else None

    # 解析 mtr
    try:
        mj = json.load(open(mtr_f))
        hubs = mj['report']['hubs']
        meta['route'] = [h['host'] for h in hubs]
        meta['step_latencies_ms'] = [h['Last'] for h in hubs]
    except:
        meta['route'] = []
        meta['step_latencies_ms'] = []

    # 解析 iperf3
    try:
        ij = json.load(open(iperf_tcp_f))
        mbps = ij['end']['sum_received']['bits_per_second']/1e6
        meta['bandwidth_Mbps'] = round(mbps,2)
        intervals = ij.get('intervals', [])
    except:
        meta['bandwidth_Mbps'] = None
        intervals = []

    # net usage
    meta['rx_MB'] = round(rx/1024/1024,2)
    meta['tx_MB'] = round(tx/1024/1024,2)
    meta['completion_sec'] = round(end_time-start_time,2)

    # Flent RRUL
    rr = base/f"rrul_{ts}" 
    Popen([
        "flent","rrul",
        "-p","all_scaled","-l",str(args.duration),
        "-s","0.2","--socket-stats",
        "-H",args.target,
        "-o",str(rr)+".png"
    ]).wait()
    meta['flent_plot'] = str(rr)+'.png'

    # 繪圖：RTT, Jitter, Throughput, MTR
    # RTT
    if rtts:
        plt.figure(figsize=(8,4))
        plt.plot(rtts, label='RTT (ms)')
        plt.xlabel('Sample'); plt.ylabel('RTT (ms)')
        plt.title('Ping RTT Over Time'); plt.grid(); plt.legend()
        f = base/'chart_rtt.png'; plt.savefig(f); plt.close(); meta['chart_rtt']=str(f)
        # Jitter
        jitter = [abs(rtts[i]-rtts[i-1]) for i in range(1,len(rtts))]
        plt.figure(figsize=(8,4))
        plt.plot(range(1,len(rtts)), jitter, label='Jitter (ms)')
        plt.xlabel('Sample'); plt.ylabel('Jitter (ms)')
        plt.title('RTT Jitter Over Time'); plt.grid(); plt.legend()
        f = base/'chart_jitter.png'; plt.savefig(f); plt.close(); meta['chart_jitter']=str(f)
    # Throughput
    if intervals:
        times = [ (i['sum']['start']+i['sum']['end'])/2 for i in intervals]
        tps   = [ i['sum']['bits_per_second']/1e6 for i in intervals]
        plt.figure(figsize=(8,4))
        plt.plot(times,tps, label='Throughput (Mbps)')
        plt.xlabel('Time (s)'); plt.ylabel('Throughput (Mbps)')
        plt.title('TCP Throughput Over Time'); plt.grid(); plt.legend()
        f = base/'chart_throughput.png'; plt.savefig(f); plt.close(); meta['chart_throughput']=str(f)
    # MTR
    hops = meta['route']; lats=meta['step_latencies_ms']
    if hops and lats:
        plt.figure(figsize=(10,5))
        plt.plot(lats,marker='o'); plt.xticks(range(len(hops)),hops,rotation=45)
        plt.xlabel('Hop'); plt.ylabel('Latency (ms)')
        plt.title('Step-wise Latency per Hop'); plt.grid()
        f = base/'chart_mtr.png'; plt.tight_layout(); plt.savefig(f); plt.close(); meta['chart_mtr']=str(f)
        meta['mtr_start_ms']=lats[0]; meta['mtr_end_ms']=lats[-1]
        meta['mtr_diff_ms']=round(lats[-1]-lats[0],2)

    # 寫入 meta
    (base/f'meta_{ts}.json').write_text(json.dumps(meta,indent=2))

    # 更新 summary.csv
    summary_csv = Path(args.out)/'summary.csv'
    row={
        'timestamp':ts,'target':args.target,'load':args.load,
        'rtt_avg':meta['rtt_avg_ms'],'loss_%':meta['packet_loss%'],
        'bw_Mbps':meta['bandwidth_Mbps'],'rx_MB':meta['rx_MB'],'tx_MB':meta['tx_MB'],
        'bufferbloat_ms':meta['bufferbloat_ms'],'completion_s':meta['completion_sec'],
        'mtr_diff_ms':meta.get('mtr_diff_ms')
    }
    append_to_csv(summary_csv,row,list(row.keys()))

    print("✅ All done. Results in:", base)

if __name__=="__main__":
    main()
