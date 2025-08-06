#!/usr/bin/env python3
"""
network_test_vis2.py

一鍵收集：
 1. RTT (ping)
 2. Step-wise latency + routing (mtr)
 3. Bandwidth + Retransmission (iperf3 TCP/選 UDP)
 4. RX/TX bytes & packets (psutil 或 /proc/net/dev)
 5. Bufferbloat 圖 (.png) + 原始 .flent.gz (Flent RRUL -H)
 6. 完整測試時間
--------------------------------------------------
Usage:
  sudo python3 network_test_vis2.py \
      --target 192.168.0.230 \
      --duration 15 \
      --udp \
      --out /output
--------------------------------------------------
依賴：
  sudo apt install -y iperf3 mtr-tiny traceroute flent
  pip install psutil
"""
import argparse, subprocess as sp, time, json
from datetime import datetime
from pathlib import Path

try:
    import psutil
except ImportError:
    psutil = None

def sh(cmd, capture=False):
    print("$", *cmd)
    if capture:
        return sp.check_output(cmd, text=True)
    sp.run(cmd, check=True)

def io_stats():
    if psutil:
        return psutil.net_io_counters()._asdict()
    stats = {'bytes_recv':0,'packets_recv':0,'dropin':0,
             'bytes_sent':0,'packets_sent':0,'dropout':0}
    with open("/proc/net/dev") as f:
        for line in f.readlines()[2:]:
            p=line.split()
            stats['bytes_recv']   += int(p[1])
            stats['packets_recv'] += int(p[2])
            stats['dropin']       += int(p[4])
            stats['bytes_sent']   += int(p[9])
            stats['packets_sent'] += int(p[10])
            stats['dropout']      += int(p[12])
    return stats

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--target",   required=True, help="Netserver IP/Host")
    p.add_argument("--duration", type=int, default=10, help="iperf & RRUL 秒數")
    p.add_argument("--out",      default="/output", help="根資料夾，自動生成時間子資料夾")
    p.add_argument("--udp",      action="store_true", help="加跑 UDP iperf")
    args = p.parse_args()

    # 建立按 timestamp 分層的輸出資料夾
    ts = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    base = Path(args.out) / ts
    base.mkdir(parents=True, exist_ok=True)

    meta = {"target":args.target, "duration":args.duration, "timestamp":ts}
    t0 = time.time()

    # 1. Ping RTT
    ping_f = base / f"ping_{ts}.txt"
    with open(ping_f, "w") as f:
        sh(["ping", "-c", "20", args.target])
    meta["ping_file"] = str(ping_f)

    # 2. Step-wise latency + 路由 (mtr)
    try:
        mtr_j = sh(["mtr", "-rwzc", "10", "-J", args.target], capture=True)
        (base/f"mtr_{ts}.json").write_text(mtr_j)
        meta["mtr_file"] = str(base/f"mtr_{ts}.json")
    except Exception as e:
        print("⚠️ mtr failed:", e)

    # 3. iperf3 TCP
    iperf_tcp_f = base / f"iperf_tcp_{ts}.txt"
    with open(iperf_tcp_f, "w") as f:
        sh(["iperf3", "-c", args.target, "-t", str(args.duration)])
    meta["iperf_tcp"] = str(iperf_tcp_f)

    # 3b. Optional UDP
    if args.udp:
        iperf_udp_f = base / f"iperf_udp_{ts}.txt"
        with open(iperf_udp_f, "w") as f:
            sh(["iperf3", "-u", "-b", "0", "-c", args.target, "-t", str(args.duration)])
        meta["iperf_udp"] = str(iperf_udp_f)

    # 4. RX/TX 前後統計
    meta["net_before"] = io_stats()
    time.sleep(1)
    meta["net_after"]  = io_stats()

    # 5. Flent RRUL (Bufferbloat) 正確語法：flent rrul -p all_scaled -l <秒數> -s <步進> --socket-stats -H <host>
    rrul_base = base / f"rrul_{ts}"
    flent_cmd = [
        "flent", "rrul",
        "-p", "all_scaled",
        "-l", str(args.duration),
        "-s", "0.2",
        "--socket-stats",
        "-H", args.target,
        "-o", f"{rrul_base}.png"
    ]
    sh(flent_cmd)

    # 搬 .flent.gz
    gz = Path(f"{rrul_base}.flent.gz")
    if gz.exists():
        gz.rename(base/gz.name)
        meta["flent_gz"] = str(base/gz.name)
    meta["flent_plot"] = str(rrul_base) + ".png"

    # 結束時間 & meta 寫檔
    meta["completion_sec"] = round(time.time() - t0, 2)
    (base/f"meta_{ts}.json").write_text(json.dumps(meta, indent=2))
    print("✅ All done. Results saved to:", base)

if __name__ == "__main__":
    main()
