# Network Tester - 模組化版本

## 📁 專案結構

```
network_tester/
├── launch/
│   └── network_test.launch.py           # ROS 2 啟動檔
├── network_tester/
│   ├── __init__.py                      # Package 初始化
│   ├── node_network_tester.py           # 主 ROS Node
│   ├── tools/                           # 工具模組
│   │   ├── __init__.py
│   │   ├── vpn_detect.py                # VPN 偵測
│   │   ├── io_utils.py                  # 檔案 I/O 工具
│   │   ├── ping_tool.py                 # Ping 測試
│   │   ├── iperf_tool.py                # iperf3 測試
│   │   ├── mtr_tool.py                  # MTR 測試
│   │   ├── flent_tool.py                # Flent RRUL 測試
│   │   ├── ros_bw_monitor.py            # ROS topic 頻寬監控
│   │   └── plot_utils.py                # 圖表繪製
│   ├── network_tester_unified.py        # 舊版統一測試（保留）
│   └── ...                              # 其他舊版檔案
├── scripts/
│   ├── run_network_test.py              # 簡化啟動腳本
│   └── analyze_results.py               # 結果分析腳本
└── setup.py                             # 安裝設定
```

## 🚀 使用方式

### 方法 1: 使用 Launch 檔案（推薦）

```bash
# 基本測試（自動偵測 VPN）
ros2 launch network_tester network_test.launch.py target:=192.168.0.230

# 無 VPN 環境測試
ros2 launch network_tester network_test.launch.py \
  target:=192.168.0.245 \
  duration:=30 \
  load:=B \
  vpn_mode:=no_vpn \
  topics:="/camera/image_raw,/robot/joint_states" \
  types:="sensor_msgs/msg/Image,sensor_msgs/msg/JointState"

# 有 VPN 環境測試
ros2 launch network_tester network_test.launch.py \
  target:=192.168.0.245 \
  duration:=30 \
  load:=B \
  vpn_mode:=with_vpn \
  topics:="/camera/camera/color/image_raw/compressed" \
  types:="sensor_msgs/msg/CompressedImage"
```

### 方法 2: 直接執行 Node

```bash
ros2 run network_tester network_test_node \
  --ros-args \
  -p target:=192.168.0.230 \
  -p duration:=30 \
  -p load:=B \
  -p vpn_mode:=auto
```

### 方法 3: 使用簡化腳本

```bash
cd /root/NETWORK/ros2_ws/src/network_tester/scripts

# 先設定執行權限
chmod +x run_network_test.py analyze_results.py

# 執行測試
./run_network_test.py --target 192.168.0.230 --duration 30 --load B

# 分析歷史結果
./analyze_results.py /root/NETWORK/output
```

## 📊 參數說明

| 參數 | 預設值 | 說明 |
|------|--------|------|
| `target` | `--` | 測試目標 IP 位址 |
| `duration` | `30` | 測試持續時間（秒）|
| `load` | `B` | 負載等級（A/B/C）|
| `out` | `/root/NETWORK/output` | 輸出目錄 |
| `vpn_mode` | `auto` | VPN 模式（no_vpn/with_vpn/auto）|
| `topics` | `""` | ROS topics（逗號分隔）|
| `types` | `""` | 訊息類型（逗號分隔）|
| `bw_interval` | `1` | 頻寬統計間隔（秒）|
| `skip_plots` | `0` | 跳過圖表生成（0/1）|

### 負載等級說明

- **Load A**: 低負載（1 串流，無頻寬限制）
- **Load B**: 中負載（5 串流，2 Mbps）⭐ 預設
- **Load C**: 高負載（5 串流，24 Mbps）

## 📦 建置與安裝

```bash
cd /root/NETWORK/ros2_ws

# 建置套件
colcon build --packages-select network_tester

# 載入環境
source install/setup.bash

# 驗證安裝
ros2 pkg list | grep network_tester
ros2 run network_tester network_test_node --help
```

## 🔍 輸出檔案說明

每次測試會產生一個時間戳記目錄，例如 `2025-10-28_14-30-15_no_vpn/`，包含：

### 原始數據檔案
- `ping_*.txt` - Ping 原始輸出
- `iperf_tcp_*.json` - iperf3 JSON 結果
- `mtr_*.json` - MTR JSON 結果
- `ros2_bw.csv` - ROS topic 頻寬即時數據

### 處理後數據
- `ping_stats_*.csv` - Ping RTT 統計
- `iperf_intervals_*.csv` - iperf3 間隔統計
- `mtr_hops_*.csv` - MTR 逐跳統計
- `ros2_bw_summary.json` - ROS 頻寬摘要

### 元數據
- `meta_*.json` - 完整測試元數據

### 圖表（如果啟用）
- `chart_rtt_*.png` - RTT 時序圖
- `chart_throughput_*.png` - 吞吐量圖
- `chart_mtr_*.png` - MTR 路由圖
- `chart_ros2_bw_*_*.png` - ROS topic 頻寬圖
- `rrul_*.png` - Flent RRUL 綜合圖

## 🛠️ 模組說明

### 工具模組 (`tools/`)

每個模組都是獨立且可重用的：

1. **VPNDetector** (`vpn_detect.py`)
   - 自動偵測 VPN 連線狀態
   - 檢查 tun/tap 介面
   - 檢測 VPN IP 範圍

2. **IOUtils** (`io_utils.py`)
   - CSV/JSON 讀寫
   - 系統網路 I/O 統計
   - 安全型別轉換

3. **PingTool** (`ping_tool.py`)
   - 啟動 ping 測試
   - 解析 RTT 數據
   - 計算 jitter 和 bufferbloat

4. **IperfTool** (`iperf_tool.py`)
   - 執行 iperf3 測試
   - 解析 JSON 結果
   - 提取間隔統計

5. **MTRTool** (`mtr_tool.py`)
   - 執行 MTR 路由追蹤
   - 解析逐跳延遲
   - 計算路由統計

6. **FlentTool** (`flent_tool.py`)
   - 執行 Flent RRUL 測試
   - 生成綜合性能圖表

7. **ROSBandwidthMonitor** (`ros_bw_monitor.py`)
   - 訂閱 ROS topics
   - 即時計算頻寬和頻率
   - 自動儲存統計

8. **PlotUtils** (`plot_utils.py`)
   - RTT 時序圖
   - 吞吐量圖
   - MTR 路由圖
   - ROS 頻寬圖

## 🔄 相容性

### 舊版指令仍然可用

```bash
# 舊版統一測試仍可使用
ros2 run network_tester network_unified --ros-args -p target:=192.168.0.230
```

### 遷移建議

建議使用新版模組化 Node：
- ✅ 更好的錯誤處理
- ✅ 更清晰的程式碼結構
- ✅ 更容易擴充和維護
- ✅ 更詳細的日誌輸出

## 📝 開發建議

### 新增測試工具

1. 在 `tools/` 建立新模組，例如 `traceroute_tool.py`
2. 實作測試邏輯（啟動、解析、儲存）
3. 在 `tools/__init__.py` 中匯出
4. 在 `node_network_tester.py` 中整合
5. 更新 `setup.py` 和文檔

### 程式碼風格

- 使用 Type Hints
- 加入 docstring
- 錯誤處理要完整
- Logger 輸出要清晰
- 優先保存原始數據

## 🐛 故障排除

### 問題：找不到 network_test_node

```bash
# 重新建置並載入環境
cd /root/NETWORK/ros2_ws
colcon build --packages-select network_tester
source install/setup.bash
```

### 問題：matplotlib 找不到

```bash
pip3 install matplotlib
```

### 問題：權限不足

```bash
# 設定腳本執行權限
chmod +x scripts/*.py
```

### 問題：iperf3 或 mtr 找不到

```bash
sudo apt-get install iperf3 mtr
```

## 📞 支援

如有問題或建議，請查看：
- 主要文檔：`/root/NETWORK/README.md`
- 快速入門：`/root/NETWORK/entrypoint/toturial/QUICK_START.md`
- VPN 偵測指南：`/root/NETWORK/entrypoint/toturial/VPN_DETECTION_GUIDE.md`
