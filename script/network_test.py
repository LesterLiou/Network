import os
import subprocess
import time
from datetime import datetime

# --------- 設定參數 ---------
target_ip = "192.168.0.230"
duration_sec = 10  # 5 minutes

# --------- 建立時間標記與 output 資料夾 ---------
timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
output_dir = os.path.join("/root/NETWORK/output", timestamp)
os.makedirs(output_dir, exist_ok=True)

# --------- 開始記錄時間 ---------
start_time = time.time()

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

# Flent 自動產生的 .flent.gz 會在當前目錄，把它搬到 output 資料夾
for f in os.listdir("."):
    if f.startswith("rrul-") and f.endswith(".flent.gz"):
        os.rename(f, flent_data)
        break  # 只搬第一個

# --------- Traceroute ---------
with open(os.path.join(output_dir, "traceroute.txt"), "w") as f:
    subprocess.run(["traceroute", target_ip], stdout=f)

# --------- Iperf3 測試 ---------
with open(os.path.join(output_dir, "iperf_result.txt"), "w") as f:
    subprocess.run(["iperf3", "-c", target_ip, "-t", str(duration_sec)], stdout=f)

# --------- 記錄完成時間 ---------
end_time = time.time()
elapsed = end_time - start_time
with open(os.path.join(output_dir, "completion_time.txt"), "w") as f:
    f.write(f"Test completed in {elapsed:.2f} seconds.\n")

print(f"✅ 測試完成，結果儲存於：{output_dir}")
