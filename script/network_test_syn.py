"""
network_test_syn.py  ── 並行測試模式
--------------------------------------------------
同時啟動：
 1. Ping RTT (interval=0.2s)
 2. iperf3 TCP 吞吐
 3. iperf3 UDP 吞吐 (若 --udp)
 4. mtr 逐跳延遲 + routing (JSON)
--------------------------------------------------
最後：
 5. 前後 RX/TX 快照
 6. （可選）Flent RRUL 圖
 7. meta_<timestamp>.json

Usage:
python3 network_test_syn.py \
      --target 192.168.0.230 \
      --duration 15 \
      --udp \
      --load A \
      --out ../output
"""
import argparse, time, json, csv
from datetime import datetime
from pathlib import Path
from subprocess import Popen, STDOUT
import psutil
import matplotlib.pyplot as plt



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

def launch_background_load(target_ip, load):
    from subprocess import DEVNULL
    if load == "B":  # 5–10 Mbps
        return Popen(["iperf3", "-c", target_ip, "-u", "-b", "8M", "-t", "300", "-p", "5002"], stdout=DEVNULL, stderr=DEVNULL)
    elif load == "C":  # 120 Mbps
        return Popen(["iperf3", "-c", target_ip, "-u", "-b", "120M", "-t", "300", "-p", "5002"], stdout=DEVNULL, stderr=DEVNULL)
    return None


def plot_summary(data_dir: Path, rtt_data):
    png = data_dir / "summary_plot.png"
    plt.figure(figsize=(10,5))
    plt.plot(rtt_data, label="RTT (ms)")
    plt.xlabel("Samples")
    plt.ylabel("RTT (ms)")
    plt.title("RTT Over Time")
    plt.legend()
    plt.grid(True)
    plt.savefig(png)
    return png

def extract_ping_rtt(ping_file):
    rtts = []
    with open(ping_file) as f:
        for line in f:
            if "time=" in line:
                try:
                    rtts.append(float(line.split("time=")[1].split(" ")[0]))
                except:
                    pass
    return rtts

def append_to_csv(csv_path, row, headers):
    write_header = not csv_path.exists()
    with open(csv_path, "a", newline='') as f:
        writer = csv.DictWriter(f, fieldnames=headers)
        if write_header:
            writer.writeheader()
        writer.writerow(row)

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--target",   required=True)
    p.add_argument("--duration", type=int, default=10)
    p.add_argument("--out",      default="../output")
    p.add_argument("--udp",      action="store_true")
    p.add_argument("--load", choices=['A', 'B', 'C'], default='A')
    args = p.parse_args()

    # 建立時間分層資料夾
    ts   = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    base = Path(args.out) / ts
    base.mkdir(parents=True, exist_ok=True)

    meta = {"target":args.target, "duration":args.duration, "timestamp":ts}

    # 0. RX/TX before
    meta["net_before"] = io_stats()


    bg_proc = launch_background_load(args.target, args.load)

    # 並行啟動各測試
    procs = []

    # 1. Ping (改成 interval 0.2s, count = duration*5)
    cnt = args.duration * 5
    ping_f = base / f"ping_{ts}.txt"
    ping_out = open(ping_f, "w")
    procs.append(("ping", ping_f, 
                  Popen(["ping","-i","0.2","-c",str(cnt),args.target],
                        stdout=ping_out, stderr=STDOUT)))
    meta["ping_file"] = str(ping_f)

    # 2. iperf3 TCP
    iperf_tcp_f = base / f"iperf_tcp_{ts}.txt"
    iperf_tcp_out = open(iperf_tcp_f, "w")
    procs.append(("iperf_tcp", iperf_tcp_f,
                  Popen(["iperf3","-c",args.target,"-t",str(args.duration)],
                        stdout=iperf_tcp_out, stderr=STDOUT)))
    meta["iperf_tcp"] = str(iperf_tcp_f)

    
    # 3. iperf3 UDP (選擇)
    if args.udp:
        iperf_udp_f = base / f"iperf_udp_{ts}.txt"
        iperf_udp_out = open(iperf_udp_f, "w")
        procs.append(("iperf_udp", iperf_udp_f,
                    Popen(["iperf3", "-u", "-b", "0", "-c", args.target, "-t", str(args.duration), "-p", "5002"],
                            stdout=iperf_udp_out, stderr=STDOUT)))
        meta["iperf_udp"] = str(iperf_udp_f)

    # 4. mtr (逐跳 JSON)
    mtr_f = base / f"mtr_{ts}.json"
    mtr_out = open(mtr_f, "w")
    procs.append(("mtr", mtr_f,
                  Popen(["mtr","-rwzc","10","-j",args.target],
                        stdout=mtr_out, stderr=STDOUT)))
    meta["mtr_file"] = str(mtr_f)

    # 5. 等待所有並行任務結束
    for name, fpath, proc in procs:
        proc.wait()
        # 關檔
        try: proc.stdout.close()
        except: pass

    # 6. RX/TX after
    meta["net_after"] = io_stats()

    # 7. (可選) Flent RRUL 圖
    rrul_base = base / f"rrul_{ts}"
    flent_cmd = [
        "flent","rrul",
        "-p","all_scaled","-l",str(args.duration),
        "-s","0.2","--socket-stats",
        "-H",args.target,
        "-o",str(rrul_base)+".png"
    ]
    Popen(flent_cmd).wait()
    # 搬 .flent.gz
    gz = Path(str(rrul_base)+".flent.gz")
    if gz.exists():
        gz.rename(base/gz.name)
        meta["flent_gz"] = str(base/gz.name)
    meta["flent_plot"] = str(rrul_base)+".png"

    # 8. 完成時間 & 寫 meta
    meta["completion_sec"] = round(time.time() - time.mktime(datetime.strptime(ts, "%Y-%m-%d_%H-%M-%S").timetuple()), 2)
    (base/f"meta_{ts}.json").write_text(json.dumps(meta, indent=2))

    if bg_proc:
        bg_proc.terminate()

    rtt_list = extract_ping_rtt(ping_f)
    meta["rtt_avg"] = round(sum(rtt_list)/len(rtt_list), 2) if rtt_list else None
    plot_summary(base, rtt_list)

    summary_csv = Path(args.out) / "summary.csv"
    append_to_csv(summary_csv, {
        "timestamp": ts,
        "target": args.target,
        "duration": args.duration,
        "load": args.load,
        "rtt_avg": meta["rtt_avg"],
        "rx_MB": round(meta["net_after"]["bytes_recv"] - meta["net_before"]["bytes_recv"])/1024/1024,
        "tx_MB": round(meta["net_after"]["bytes_sent"] - meta["net_before"]["bytes_sent"])/1024/1024,
        "completion_sec": meta["completion_sec"],
        "plot": meta["flent_plot"]
    }, headers=["timestamp", "target", "duration", "load", "rtt_avg", "rx_MB", "tx_MB", "completion_sec", "plot"])

    print("✅ All done. Results in:", base)


if __name__=="__main__":
    main()
