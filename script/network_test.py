import os
import subprocess
import time
from datetime import datetime

# --------- 設定參數 ---------
target_ip = "192.168.0.230"
duration_sec = 5  # 測試時間（秒）
background_load = "B"  # A: 無負載, B: 5-10 Mbps, C: 120 Mbps

# --------- 建立時間標記與 output 資料夾 ---------
timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
output_dir = os.path.join("/root/NETWORK/output", f"{timestamp}_Load-{background_load}")
os.makedirs(output_dir, exist_ok=True)

# --------- 開始記錄時間 ---------
start_time = time.time()

# --------- 啟動背景負載（若有需要） ---------
iperf_background = None
if background_load == "B":
    iperf_background = subprocess.Popen([
        "iperf3", "-c", target_ip, "-u", "-b", "8M", "-t", str(duration_sec)
    ])
elif background_load == "C":
    iperf_background = subprocess.Popen([
        "iperf3", "-c", target_ip, "-u", "-b", "120M", "-t", str(duration_sec)
    ])

# --------- 執行 Flent 測試 ---------
flent_png = os.path.join(output_dir, "rrul_result.png")
flent_data = os.path.join(output_dir, f"rrul-{timestamp}.flent.gz")

subprocess.run([
    "flent", "rrul",
    "-p", "all_scaled",
    "-l", str(duration_sec),
    "-H", target_ip,
    "-o", flent_png
])

# Flent 自動產生的 .flent.gz 檔案搬到 output 資料夾
for f in os.listdir("."):
    if f.startswith("rrul-") and f.endswith(".flent.gz"):
        os.rename(f, flent_data)
        break

# --------- Traceroute ---------
with open(os.path.join(output_dir, "traceroute.txt"), "w") as f:
    subprocess.run(["traceroute", target_ip], stdout=f)

# --------- Iperf3 測試 ---------
with open(os.path.join(output_dir, "iperf_result.txt"), "w") as f:
    subprocess.run(
        ["iperf3", "-c", target_ip, "-t", str(duration_sec)],
        stdout=f,
        stderr=subprocess.STDOUT  # 捕捉錯誤訊息
    )

# --------- 結束記錄與統計時間 ---------
end_time = time.time()
elapsed = end_time - start_time

with open(os.path.join(output_dir, "completion_time.txt"), "w") as f:
    f.write(f"Test completed in {elapsed:.2f} seconds.\n")

print(f"✅ 測試完成，結果儲存於：{output_dir}")
