# 🌐 Network Performance Testing Suite for ROS 2

**完整的 ROS 2 網路性能測試工具包** - 支援 TCP 吞吐量、延遲、抖動、MTR 路由分析、Flent 緩衝區膨脹測試，以及 ROS 2 Topic 頻寬監控。

[![ROS 2 Humble](https://img.shields.io/badge/ROS_2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Ubuntu 22.04](https://img.shields.io/badge/Ubuntu-22.04-orange)](https://ubuntu.com/)
[![Python 3.10](https://img.shields.io/badge/Python-3.10-green)](https://www.python.org/)

---

## 📋 目錄

- [功能特色](#-功能特色)
- [系統需求](#-系統需求)
- [快速開始](#-快速開始)
- [詳細使用指南](#-詳細使用指南)
- [輸出檔案說明](#-輸出檔案說明)
- [結果分析](#-結果分析)
- [進階用法](#-進階用法)
- [故障排除](#-故障排除)
- [專案架構](#-專案架構)

---

## 🎯 功能特色

### 網路測試工具
- ✅ **Ping 測試**: RTT、丟包率、抖動、Bufferbloat
- ✅ **iperf3**: TCP/UDP 吞吐量、重傳次數
- ✅ **MTR**: 逐跳路由延遲分析
- ✅ **Flent RRUL**: 綜合緩衝區膨脹測試

### ROS 2 整合
- ✅ **Topic 頻寬監控**: 即時統計任意 ROS 2 topic
- ✅ **多 Topic 支援**: 同時監控多個 topic
- ✅ **自動 QoS**: Best Effort 設定適用感測器數據
- ✅ **DDS 或 ROSbridge**: 支援兩種通訊方式

### 獨立分析腳本 (位於 `script/` 目錄)
- ✅ **DDS 獨立效能分析** (`dds_network_logger.py`): 精準量測並繪製 DDS 話題的 FPS 與頻寬折線圖，並可指定預期 FPS。
- ✅ **DDS 單一節點延遲量測** (`dds_sub_image.py`): 針對影像封包等高頻寬資料的單點訂閱與延遲/抖動 (Jitter) 分析。
- ✅ **純 Python 網路壓力測試** (`network_test_TCP.py` / `network_test_ind.py` / `network_test_syn.py`): 在無需啟動 ROS 2 環境下，利用 Python 腳本快速執行平行或同步的 Ping, iperf3 (TCP/UDP), MTR 與 Bufferbloat 等連線測試，方便早期排錯。

### 自動化與分析
- ✅ **VPN 自動偵測**: 智能判斷是否透過 VPN 連線
- ✅ **圖表生成**: 自動生成 RTT、吞吐量、路由圖表
- ✅ **數據匯出**: CSV、JSON 格式完整保存
- ✅ **批次分析**: 對比多次測試結果

---

## 💻 系統需求

### 硬體需求
- CPU: 2 核心以上
- RAM: 4GB 以上
- 網路: 支援 Ethernet 或 WiFi

### 軟體需求
| 軟體 | 版本 | 必要性 |
|------|------|--------|
| Ubuntu | 22.04 LTS | ✅ 必要 |
| ROS 2 | Humble | ✅ 必要 |
| Python | 3.10+ | ✅ 必要 |
| Docker | 20.10+ | 🔶 建議 |
| iperf3 | 3.9+ | ✅ 必要 |
| mtr | 0.93+ | ✅ 必要 |
| flent | 2.0+ | 🔶 選用 |

---

## 🚀 快速開始

### 步驟 1: 克隆專案

```bash
# 克隆 repository
git clone https://github.com/LesterLiou/Network.git
cd Network

# 檢查檔案結構
ls -la
```

### 步驟 2: 環境準備（選擇一種方式）

#### 🐳 方式 A: 使用 Docker（推薦）

```bash
# 建置 Docker 映像
cd docker
./build.sh

# 啟動容器
./run.sh

# 現在你已經在容器內，可以直接跳到步驟 3
```

#### 💻 方式 B: 本機安裝

```bash
# 1. 安裝 ROS 2 Humble（如果還沒安裝）
# 參考官方文件：https://docs.ros.org/en/humble/Installation.html

# 2. 安裝必要工具
sudo apt-get update
sudo apt-get install -y \
    iperf3 \
    mtr \
    python3-pip \
    python3-matplotlib \
    python3-psutil

# 3. 安裝 flent（選用，用於 RRUL 測試）
sudo apt-get install -y flent netperf

# 4. 載入 ROS 2 環境
source /opt/ros/humble/setup.bash
```

### 步驟 3: 建置 ROS 2 套件

```bash
# 進入工作空間
cd /root/NETWORK/ros2_ws

# 建置套件
colcon build --packages-select network_tester

# 載入環境
source install/setup.bash

# 驗證安裝
ros2 pkg list | grep network_tester
```

**預期輸出**:
```
network_tester
```

### 步驟 4: 準備測試環境

#### 🖥️ 伺服器端設定（被測試機器）

```bash
# 啟動 iperf3 伺服器
iperf3 -s

# （選用）如果要測試 ROS Topic，啟動 ROS 2 節點
# 例如相機節點
ros2 run usb_cam usb_cam_node_exe
```

#### 📱 客戶端設定（測試機器）

```bash
# 測試連線（替換 <SERVER_IP> 為實際 IP）
ping -c 3 <SERVER_IP>

# 測試 iperf3 連線
iperf3 -c <SERVER_IP> -t 5
```

### 步驟 5: 執行第一次測試 🎉

```bash
# 基本網路測試（無 ROS Topic）
ros2 launch network_tester network_test.launch.py \
  target:=192.168.0.230 \
  duration:=30 \
  load:=B

# 等待測試完成（約 30-60 秒）
```

**測試進行中會看到**:
```
🚀 Network Test Configuration:
  Target: 192.168.0.230
  Duration: 30s
  Load: B
  VPN Mode: no_vpn
...
✅ Test completed successfully!
📁 Results saved to: /root/NETWORK/output/2025-10-28_14-30-15_no_vpn
```

### 步驟 6: 查看結果

```bash
# 進入結果目錄
cd /root/NETWORK/output

# 查看最新的測試結果
ls -lt | head -5

# 進入最新的測試目錄
cd $(ls -td */ | head -1)

# 查看生成的檔案
ls -lh
```

**你會看到**:
- 📊 `meta_*.json` - 測試配置和統計摘要
- 📈 `image/` 目錄 - 所有圖表
- 📄 `*.csv` - 原始數據（可用 Excel 開啟）
- 🔍 `ping_*.txt`, `iperf_tcp_*.json` - 詳細測試日誌

### 步驟 7: 查看圖表

```bash
# 查看 image 目錄中的所有圖表
ls image/

# 使用系統預設圖片檢視器開啟
xdg-open image/chart_rtt_*.png       # RTT 延遲圖
xdg-open image/chart_throughput_*.png # 吞吐量圖
xdg-open image/chart_mtr_*.png       # 路由圖
```

---

## 📘 詳細使用指南

### 測試場景 1: 基本網路性能測試

**目的**: 測試 TCP 吞吐量、延遲、抖動

```bash
ros2 launch network_tester network_test.launch.py \
  target:=192.168.0.230 \
  duration:=30 \
  load:=B
```

**參數說明**:
- `target`: 測試目標 IP
- `duration`: 測試時長（秒）
- `load`: 負載等級
  - `A`: 輕負載（1 串流，無限制）
  - `B`: 中負載（5 串流，2 Mbps）⭐ 推薦
  - `C`: 重負載（5 串流，24 Mbps）

---

### 測試場景 2: ROS 2 Topic 頻寬測試

**目的**: 測試 ROS 2 通訊效能

#### 2.1 使用 DDS 直連（推薦）

```bash
# 確保 ROS_DOMAIN_ID 一致
export ROS_DOMAIN_ID=0

# 執行測試
ros2 launch network_tester network_test.launch.py \
  target:=192.168.0.230 \
  duration:=30 \
  load:=B \
  topics:="/camera/image_raw,/robot/joint_states" \
  types:="sensor_msgs/msg/Image,sensor_msgs/msg/JointState"
```

#### 2.2 使用 ROSbridge（跨網域）

**✅ ROSbridge 支持現已可用！**

**為什麼 ROSbridge 的 topic 不會出現在 `ros2 topic list` 中**:
- ROSbridge 使用 **WebSocket (TCP)**，不是 DDS
- `ros2 topic list` 只顯示 **DDS topics**
- ROSbridge 訂閱與 **ROS 2 DDS 網路是分離的**

**伺服器端**:
```bash
# 啟動 rosbridge 伺服器
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# 驗證 rosbridge 正在運行
nc -zv <SERVER_IP> 9090
```

**客戶端**:
```bash
# 使用 launch 檔案（推薦）
ros2 launch network_tester network_test.launch.py \
  target:=10.0.0.88 \
  duration:=30 \
  load:=B \
  topics:="/camera/camera/color/image_raw/compressed" \
  types:="sensor_msgs/msg/CompressedImage" \
  use_rosbridge:=1 \
  ws_host:=10.0.0.88 \
  ws_port:=9090

# 或使用自動測試腳本
/root/NETWORK/test_rosbridge.sh 10.0.0.88 /camera/camera/color/image_raw/compressed
```

**如何驗證 ROSbridge 數據傳輸**（在伺服器端）:

```bash
# 安裝 roslibpy（如果尚未安裝）
pip3 install roslibpy

# 執行驗證腳本
cd /root/NETWORK/ros2_ws/src/network_tester/scripts
python3 verify_rosbridge.py \
  --host 127.0.0.1 \
  --port 9090 \
  --topic /camera/camera/color/image_raw/compressed \
  --type sensor_msgs/CompressedImage \
  --duration 30
```

**如果數據正在傳輸，預期輸出**:
```
✅ Connected successfully!
📡 Subscribing to /camera/camera/color/image_raw/compressed...
✅ First message received at 14:30:25
📊 Messages: 150 | Rate: 5.0 Hz | Bandwidth: 2.34 Mbps | Elapsed: 30.0s
```

**輸出檔案**:
- `rosbridge_bw.csv` - ROSbridge 頻寬時間序列
- `rosbridge_bw_summary.json` - ROSbridge 統計摘要
- `chart_rosbridge_bw_*.png` - ROSbridge 頻寬圖表

---

### 測試場景 3: VPN 效能對比

**目的**: 比較 VPN 對網路的影響

#### 3.1 無 VPN 測試
```bash
ros2 launch network_tester network_test.launch.py \
  target:=192.168.0.230 \
  duration:=30 \
  load:=B \
  vpn_mode:=no_vpn
```

#### 3.2 有 VPN 測試
```bash
# 啟動 VPN 連線（例如 OpenVPN 或 WireGuard）
# 然後執行測試

ros2 launch network_tester network_test.launch.py \
  target:=10.8.0.2 \
  duration:=30 \
  load:=B \
  vpn_mode:=with_vpn
```

#### 3.3 自動對比分析
```bash
cd /root/NETWORK/ros2_ws/src/network_tester/scripts

# 對比兩次測試
python3 analyze_results.py \
  /root/NETWORK/output/*_no_vpn \
  /root/NETWORK/output/*_with_vpn \
  --compare
```

---

### 測試場景 4: 長期監控

**目的**: 持續監控網路品質

```bash
#!/bin/bash
# 儲存為 continuous_test.sh

while true; do
    echo "=== $(date) ==="
    
    ros2 launch network_tester network_test.launch.py \
      target:=192.168.0.230 \
      duration:=60 \
      load:=B
    
    # 等待 30 分鐘後再次測試
    sleep 1800
done
```

---

## 📊 輸出檔案說明

每次測試會建立一個時間戳記目錄，結構如下:

```
output/
└── 2025-10-28_14-30-15_no_vpn/
    ├── 📄 meta_2025-10-28_14-30-15.json      # 測試摘要（最重要）
    ├── 📁 image/                              # 所有圖表
    │   ├── chart_rtt_*.png                    # RTT 延遲圖
    │   ├── chart_throughput_*.png             # 吞吐量圖
    │   ├── chart_mtr_*.png                    # 路由延遲圖
    │   ├── chart_ros2_bw_*.png                # ROS Topic 頻寬圖
    │   └── rrul_*.png                         # Flent RRUL 圖
    ├── 📊 ping_stats_*.csv                    # Ping RTT 時間序列
    ├── 📊 iperf_intervals_*.csv               # iperf 吞吐量時間序列
    ├── 📊 mtr_hops_*.csv                      # MTR 逐跳數據
    ├── 📊 ros2_bw.csv                         # ROS Topic 頻寬時間序列
    ├── 📋 ros2_bw_summary.json                # ROS 頻寬統計摘要
    ├── 🔍 ping_*.txt                          # Ping 原始輸出
    ├── 🔍 iperf_tcp_*.json                    # iperf3 原始 JSON
    ├── 🔍 mtr_*.json                          # MTR 原始 JSON
    └── 🔍 rrul_*.flent.gz                     # Flent 原始數據
```

### 核心檔案說明

#### 📄 `meta_*.json` - 測試摘要（★★★ 最重要）

包含所有關鍵指標:

```json
{
  "target": "192.168.0.230",
  "duration": 30,
  "vpn_mode": "no_vpn",
  "ping": {
    "rtt_avg_ms": 12.5,
    "rtt_max_ms": 25.3,
    "loss_percent": 0.0,
    "jitter_ms": 2.1,
    "bufferbloat_ms": 12.8
  },
  "iperf": {
    "bandwidth_mbps": 95.2,
    "total_retransmits": 5,
    "lost_percent": 0.01
  },
  "mtr": {
    "total_hops": 3,
    "avg_latency_ms": 15.2
  }
}
```

#### 📊 CSV 檔案 - 詳細時間序列

可用 Excel、Python pandas 或其他工具分析:

```bash
# 使用 pandas 分析
python3 -c "
import pandas as pd
df = pd.read_csv('ping_stats_*.csv')
print(df.describe())
"
```

#### 📁 `image/` 目錄 - 視覺化圖表

所有圖表統一存放，方便查看和分享。

---

## 🔍 結果分析

### 方式 1: 自動化分析腳本（推薦）

```bash
cd /root/NETWORK/ros2_ws/src/network_tester/scripts

# 分析單一測試
python3 analyze_results.py /root/NETWORK/output/2025-10-28_14-30-15_no_vpn

# 批次分析所有測試
python3 analyze_results.py /root/NETWORK/output --batch

# 對比兩次測試
python3 analyze_results.py \
  /root/NETWORK/output/2025-10-28_14-30-15_no_vpn \
  /root/NETWORK/output/2025-10-28_15-00-20_with_vpn \
  --compare
```

**輸出範例**:
```
================================================================================
📊 網路測試分析報告
================================================================================
測試時間: 2025-10-28T14:30:15
VPN 模式: no_vpn
--------------------------------------------------------------------------------
🏓 Ping RTT 分析
--------------------------------------------------------------------------------
平均延遲: 12.50 ms
最大延遲: 25.30 ms
抖動:     2.10 ms
質量評級: 🟢 優秀 (Excellent)
--------------------------------------------------------------------------------
🚀 iperf3 吞吐量分析
--------------------------------------------------------------------------------
平均吞吐量: 95.20 Mbps
重傳次數:   5
質量評級:   🟢 優秀 (Excellent)
```

### 方式 2: 手動分析

#### 快速查看關鍵指標

```bash
# 進入測試目錄
cd /root/NETWORK/output/2025-10-28_14-30-15_no_vpn

# 查看測試摘要
cat meta_*.json | jq '{target, vpn_mode, ping, iperf}'

# 計算平均 RTT
awk -F',' 'NR>1 {sum+=$2; n++} END {print "平均 RTT:", sum/n, "ms"}' ping_stats_*.csv

# 查看 iperf 吞吐量
cat iperf_tcp_*.json | jq '.end.sum_received.bits_per_second / 1000000'

# 查看圖表
ls image/*.png
```

---

## ⚡ 獨立 Python 腳本測試 (位於 `script/` 目錄)

如果你只需要單一工具，或是專注於排查特定 ROS Topic 效能，可以使用 `script/` 目錄中的強大獨立腳本：

### 1. 純 DDS/ROS 2 Topic 頻寬與 FPS 監控 (`dds_network_logger.py`)
這是一個不需要啟動完整測試框架就能使用的輕量級腳本，專門用來訂閱一個 Topic，並每秒統計一次資料接收量與 FPS。執行完畢**會自動繪出漂亮的折線圖並輸出 CSV**:

```bash
cd /root/NETWORK/script
python3 dds_network_logger.py \
  --topic /camera/camera/color/image_raw/compressed \
  --duration 30 \
  --expected-fps 15.0
```
- **適用場景**：專治掉幀與影像延遲，藉由 `expected-fps` 對比實際達到的 FPS 波動。

### 2. DDS/ROS 2 Latency 與 Jitter 分析 (`dds_sub_image.py`)
針對攜帶 header timestamp (如影像、光達) 的高頻寬 ROS 2 訊息，它能測量傳遞延遲與抖動 (Jitter) 並保存分析圖與日誌檔案 (`dds_latency_plot.png`)。

```bash
cd /root/NETWORK/script
python3 dds_sub_image.py
```

### 3. 無 ROS 依賴的純網路壓力測試 (`network_test_TCP.py` 等)
直接利用 Python 調用原生的 iperf3, ping, MTR 指令來生成圖表。這在你想要測試基礎網路拓撲狀況而不需要 ROS 2 環境參與時非常好用。

```bash
cd /root/NETWORK/script
sudo python3 network_test_TCP.py \
  --target 192.168.0.230 \
  --duration 15 \
  --load B
```

---

## 🎓 進階用法

### 自訂測試參數

```bash
ros2 launch network_tester network_test.launch.py \
  target:=192.168.0.230 \
  duration:=60 \              # 更長的測試時間
  load:=C \                   # 高負載
  bw_interval:=0.5 \          # 更頻繁的頻寬採樣
  skip_plots:=1 \             # 跳過圖表生成（加快速度）
  out:=/custom/output/path    # 自訂輸出目錄
```

### 多 Topic 並行監控

```bash
ros2 launch network_tester network_test.launch.py \
  target:=192.168.0.230 \
  duration:=30 \
  topics:="/cam1/image,/cam2/image,/lidar/points,/robot/joint_states" \
  types:="sensor_msgs/msg/Image,sensor_msgs/msg/Image,sensor_msgs/msg/PointCloud2,sensor_msgs/msg/JointState"
```

### 批次測試腳本

```bash
#!/bin/bash
# batch_test.sh - 批次測試不同負載

for load in A B C; do
    echo "Testing with load $load"
    ros2 launch network_tester network_test.launch.py \
      target:=192.168.0.230 \
      duration:=30 \
      load:=$load
    sleep 10
done

# 分析所有結果
python3 /root/NETWORK/ros2_ws/src/network_tester/scripts/analyze_results.py \
  /root/NETWORK/output --batch
```

---

## 🔧 故障排除

### 問題 1: `ros2 launch` 找不到套件

**原因**: 環境未載入

**解決方案**:
```bash
cd /root/NETWORK/ros2_ws
source install/setup.bash
ros2 pkg list | grep network_tester
```

---

### 問題 2: iperf3 連線失敗

**錯誤訊息**: `unable to connect to server: Connection refused`

**解決方案**:
```bash
# 在伺服器端確認 iperf3 是否運行
ps aux | grep iperf3

# 重新啟動 iperf3 伺服器
pkill iperf3
iperf3 -s

# 檢查防火牆
sudo ufw allow 5201/tcp
```

---

### 問題 3: ROS Topic 無法訂閱

**錯誤訊息**: `No messages received on topic`

**解決方案**:
```bash
# 確認 ROS_DOMAIN_ID 一致（僅適用 DDS）
echo $ROS_DOMAIN_ID  # 兩端應相同

# 檢查 topic 是否存在
ros2 topic list
ros2 topic echo /camera/image_raw --once

# 檢查網路連線
ping <target_ip>
```

---

### 問題 4: ROSbridge 顯示頻寬但伺服器端收不到數據

**問題描述**: 使用 `ros2 run network_tester network_rosbridge` 配合 ROSbridge 時，客戶端顯示有頻寬統計，但伺服器端 `10.0.0.88` 卻收不到 topic 數據。

**根本原因**:
- ⚠️ **ROSbridge 使用 WebSocket (TCP)，不是 DDS**
- `ROS_DOMAIN_ID` 對 ROSbridge 通訊**完全無效**
- 客戶端透過 WebSocket 訂閱伺服器的 rosbridge，頻寬監控測量的是 WebSocket 流量，而非實際 topic 重新發布

**關鍵差異**:

| 通訊方式 | 協定 | ROS_DOMAIN_ID 影響 | 網路邊界 |
|---------|------|-------------------|---------|
| **DDS 直連** | UDP 多播 | ✅ **必須匹配** | 建議同網段 |
| **ROSbridge** | WebSocket (TCP) | ❌ **無影響** | 可跨網段 |

**解決方案**:

1. **使用 DDS**（同網段推薦）:
```bash
# 伺服器端 - 確保設定 ROS_DOMAIN_ID
export ROS_DOMAIN_ID=0

# 客戶端 - 必須與伺服器相同
export ROS_DOMAIN_ID=0

# 使用 DDS 連線（不用 rosbridge）
ros2 launch network_tester network_test.launch.py \
  target:=192.168.0.230 \
  duration:=30 \
  topics:="/camera/camera/color/image_raw/compressed" \
  types:="sensor_msgs/msg/CompressedImage"
```

2. **使用 ROSbridge**（跨網段或 NAT）:
```bash
# 伺服器端 (10.0.0.88) - 啟動 rosbridge
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# 驗證 rosbridge 可連線
nc -zv 10.0.0.88 9090  # 應顯示 "succeeded"

# 客戶端 - ROS_DOMAIN_ID 無關
ros2 run network_tester network_rosbridge \
  --ros-args \
  -p target:=10.0.0.88 \
  -p use_rosbridge:=1 \
  -p ws_host:=10.0.0.88 \
  -p ws_port:=9090 \
  -p topics:="/camera/camera/color/image_raw/compressed" \
  -p types:="sensor_msgs/msg/CompressedImage"

# 在伺服器端驗證數據:
# 方法 A: 使用 rosbridge 客戶端訂閱
python3 -c "
import roslibpy
ros = roslibpy.Ros(host='127.0.0.1', port=9090)
ros.run()
topic = roslibpy.Topic(ros, '/camera/camera/color/image_raw/compressed', 'sensor_msgs/CompressedImage')
topic.subscribe(lambda m: print(f'收到: {len(m[\"data\"])} bytes'))
ros.run_forever()
"

# 方法 B: 使用 ros2 工具檢查（在伺服器上）
ros2 topic list
ros2 topic hz /camera/camera/color/image_raw/compressed
```

3. **理解頻寬數據的含義**:
   - 使用 ROSbridge: 頻寬顯示 **WebSocket 流量**（客戶端 ↔ rosbridge 伺服器）
   - 使用 DDS: 頻寬顯示訂閱者**實際收到的 topic 數據**
   - ROSbridge 會增加 JSON 編碼開銷（約 20-40% 額外頻寬）

---

### 問題 5: matplotlib 圖表無法生成

**錯誤訊息**: `ModuleNotFoundError: No module named 'matplotlib'`

**解決方案**:
```bash
pip3 install matplotlib

# 或跳過圖表生成
ros2 launch network_tester network_test.launch.py \
  target:=192.168.0.230 \
  skip_plots:=1
```

---

### 問題 5: matplotlib 圖表無法生成

**錯誤訊息**: `ModuleNotFoundError: No module named 'matplotlib'`

**解決方案**:
```bash
pip3 install matplotlib

# 或跳過圖表生成
ros2 launch network_tester network_test.launch.py \
  target:=192.168.0.230 \
  skip_plots:=1
```

---

### 問題 6: VPN 偵測不準確

**解決方案**: 手動指定 VPN 模式
```bash
# 無 VPN
ros2 launch network_tester network_test.launch.py \
  target:=192.168.0.230 \
  vpn_mode:=no_vpn

# 有 VPN
ros2 launch network_tester network_test.launch.py \
  target:=10.8.0.2 \
  vpn_mode:=with_vpn
```

---

## 🏗️ 專案架構

### 目錄結構

```
NETWORK/
├── 📁 docker/                    # Docker 環境
│   ├── Dockerfile
│   ├── build.sh
│   └── run.sh
├── 📁 ros2_ws/                   # ROS 2 工作空間
│   └── src/
│       └── network_tester/       # 主要套件
│           ├── 📄 node_network_tester.py        # 主 Node
│           ├── 📁 tools/                        # 工具模組
│           │   ├── vpn_detect.py
│           │   ├── ping_tool.py
│           │   ├── iperf_tool.py
│           │   ├── mtr_tool.py
│           │   ├── flent_tool.py
│           │   ├── ros_bw_monitor.py
│           │   ├── plot_utils.py
│           │   └── io_utils.py
│           ├── 📁 launch/                       # Launch 檔案
│           │   └── network_test.launch.py
│           ├── 📁 scripts/                      # 工具腳本
│           │   ├── run_network_test.py
│           │   └── analyze_results.py
│           └── 📄 setup.py
├── 📁 output/                    # 測試結果輸出
├── 📁 entrypoint/                # 進入點腳本
│   └── toturial/
│       ├── QUICK_START.md
│       └── VPN_DETECTION_GUIDE.md
└── 📄 README.md                  # 本檔案
```

### 模組說明

詳細架構請參考：[ARCHITECTURE.md](ros2_ws/src/network_tester/ARCHITECTURE.md)

---

## 📚 參考資料

### 相關文件
- [ROS 2 Humble 官方文件](https://docs.ros.org/en/humble/)
- [iperf3 使用指南](https://iperf.fr/iperf-doc.php)
- [MTR 網路診斷工具](https://github.com/traviscross/mtr)
- [Flent 網路測試](https://flent.org/)

### 教學文件
- [快速入門指南](entrypoint/toturial/QUICK_START.md)
- [VPN 偵測指南](entrypoint/toturial/VPN_DETECTION_GUIDE.md)
- [模組化架構說明](ros2_ws/src/network_tester/ARCHITECTURE.md)
- [結果分析指南](ros2_ws/src/network_tester/README_MODULAR.md)

### 效能基準

| 指標 | 良好 | 一般 | 需改善 |
|------|------|------|--------|
| RTT 平均 | < 30ms | 30-100ms | > 100ms |
| RTT 抖動 | < 5ms | 5-20ms | > 20ms |
| 丟包率 | < 0.1% | 0.1-1% | > 1% |
| 吞吐量 | > 80% 理論值 | 50-80% | < 50% |
| 重傳次數 | < 10 | 10-50 | > 50 |

---

## 📝 更新日誌

### v2.0.0 (2025-10-28)
- ✨ 模組化重構：拆分為 8 個獨立工具模組
- ✨ 新增自動化分析腳本
- ✨ 圖表統一放到 `image/` 目錄
- ✨ metadata.json 不再包含冗餘的 intervals 數據
- 🐛 修復 iperf3 timeout 問題
- 📝 完全重寫 README

### v1.0.0 (2024-XX-XX)
- 🎉 初始版本
- ✨ 支援 Ping、iperf3、MTR、Flent 測試
- ✨ ROS 2 Topic 頻寬監控

---

## 📄 授權

MIT License

---

## 👨‍💻 作者

**Lester Liou**

- GitHub: [@LesterLiou](https://github.com/LesterLiou)

---

## 🙏 致謝

感謝以下開源專案：
- [ROS 2](https://ros.org/)
- [iperf3](https://iperf.fr/)
- [MTR](https://github.com/traviscross/mtr)
- [Flent](https://flent.org/)
- [matplotlib](https://matplotlib.org/)

---

**🎯 快速連結**

- [📖 快速開始](#-快速開始)
- [📊 輸出檔案說明](#-輸出檔案說明)
- [🔍 結果分析](#-結果分析)
- [🔧 故障排除](#-故障排除)
- [📚 進階教學](ros2_ws/src/network_tester/README_MODULAR.md)

---

Made with ❤️ by Lester Liou
