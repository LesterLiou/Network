# Network Tester 模組化重構 - 架構說明

## 🎯 重構目標

將原本 600+ 行的單一檔案 `network_tester_unified.py` 重構成：
- ✅ **易維護**：每個功能獨立模組
- ✅ **可測試**：模組間低耦合
- ✅ **可擴展**：新增功能只需加新模組
- ✅ **清晰**：職責分明，程式碼可讀性高

## 📦 模組架構

```
network_tester/
├── 📄 node_network_tester.py (250 行)         主控 Node
│   └── 負責：參數解析、流程控制、結果整合
│
├── 🔧 tools/                                  工具模組目錄
│   ├── vpn_detect.py (60 行)                  VPN 偵測
│   │   └── VPNDetector.detect()
│   │
│   ├── io_utils.py (120 行)                   I/O 工具
│   │   ├── get_io_stats()
│   │   ├── safe_float() / safe_int()
│   │   └── IOUtils 類別
│   │
│   ├── ping_tool.py (100 行)                  Ping 測試
│   │   └── PingTool 類別
│   │       ├── start()
│   │       ├── parse_results()
│   │       └── save_csv()
│   │
│   ├── iperf_tool.py (110 行)                 iperf3 測試
│   │   └── IperfTool 類別
│   │       ├── start()
│   │       ├── parse_results()
│   │       └── save_intervals_csv()
│   │
│   ├── mtr_tool.py (120 行)                   MTR 測試
│   │   └── MTRTool 類別
│   │       ├── start()
│   │       ├── parse_results()
│   │       └── save_hops_csv()
│   │
│   ├── flent_tool.py (50 行)                  Flent 測試
│   │   └── FlentTool 類別
│   │       └── run()
│   │
│   ├── ros_bw_monitor.py (160 行)             ROS 頻寬監控
│   │   └── ROSBandwidthMonitor 類別
│   │       ├── setup()
│   │       ├── _record_sample()
│   │       ├── get_data()
│   │       ├── save_summary()
│   │       └── teardown()
│   │
│   └── plot_utils.py (180 行)                 圖表繪製
│       └── PlotUtils 類別
│           ├── plot_ping_rtt()
│           ├── plot_throughput()
│           ├── plot_mtr_hops()
│           └── plot_ros_bandwidth()
│
├── 🚀 launch/
│   └── network_test.launch.py                 ROS 2 Launch 檔
│
└── 📜 scripts/
    ├── run_network_test.py                    簡化啟動腳本
    └── analyze_results.py                     結果分析工具
```

## 🔄 資料流程

```
1. 使用者啟動測試
   ↓
2. node_network_tester.py (主控 Node)
   ├─ 讀取參數
   ├─ VPNDetector: 偵測 VPN 模式
   └─ 建立輸出目錄
   ↓
3. 平行啟動網路測試工具
   ├─ PingTool: 背景執行 ping
   ├─ IperfTool: 背景執行 iperf3
   └─ MTRTool: 背景執行 mtr
   ↓
4. 同時啟動 ROS 監控
   └─ ROSBandwidthMonitor: 訂閱 topics，即時統計
   ↓
5. ROS Spin (duration 秒)
   └─ 持續收集 ROS 頻寬數據
   ↓
6. 等待測試完成
   └─ 所有子程序結束
   ↓
7. 解析結果（優先保存原始數據）
   ├─ PingTool.parse_results() → ping_stats.csv
   ├─ IperfTool.parse_results() → iperf_intervals.csv
   ├─ MTRTool.parse_results() → mtr_hops.csv
   └─ ROSBandwidthMonitor.save_summary() → ros2_bw_summary.json
   ↓
8. 儲存元數據
   └─ IOUtils.save_json() → meta_*.json
   ↓
9. 執行 Flent RRUL 測試
   └─ FlentTool.run()
   ↓
10. 生成圖表（可選）
    └─ PlotUtils: 各種圖表
    ↓
11. 完成！
```

## 🎨 設計原則

### 1. 單一職責原則 (SRP)
每個模組只負責一件事：
- `PingTool` 只處理 ping
- `IperfTool` 只處理 iperf3
- `PlotUtils` 只處理圖表

### 2. 開放封閉原則 (OCP)
對擴展開放，對修改封閉：
- 新增測試工具？只需加新模組
- 不需修改現有程式碼

### 3. 依賴反轉原則 (DIP)
高層模組不依賴低層實作：
- `NetworkTestNode` 透過介面使用工具
- 工具可獨立測試和替換

### 4. 介面隔離原則 (ISP)
每個工具提供簡潔介面：
```python
# Ping 工具範例
ping_tool = PingTool(target, duration, logger)
proc = ping_tool.start(output_file)      # 啟動
rtts, stats = ping_tool.parse_results()  # 解析
ping_tool.save_csv(output_file)          # 儲存
```

## 🔌 擴展範例

### 新增 Traceroute 工具

1. **建立模組** `tools/traceroute_tool.py`:
```python
class TracerouteTool:
    def __init__(self, target, logger=None):
        self.target = target
        self.logger = logger
    
    def start(self, output_file):
        # 實作 traceroute 啟動邏輯
        pass
    
    def parse_results(self, output_file):
        # 實作結果解析
        pass
```

2. **匯出模組** `tools/__init__.py`:
```python
from .traceroute_tool import TracerouteTool
__all__ = [..., 'TracerouteTool']
```

3. **整合到主 Node** `node_network_tester.py`:
```python
from .tools import TracerouteTool

# 在 run() 方法中
traceroute_tool = TracerouteTool(self.target, self.get_logger())
traceroute_proc = traceroute_tool.start(output_file)
procs.append(('traceroute', traceroute_proc))
```

4. **完成！** 不需修改其他程式碼

## 📊 程式碼統計

| 檔案 | 行數 | 職責 |
|------|------|------|
| `node_network_tester.py` | 250 | 主控流程 |
| `vpn_detect.py` | 60 | VPN 偵測 |
| `io_utils.py` | 120 | I/O 工具 |
| `ping_tool.py` | 100 | Ping 測試 |
| `iperf_tool.py` | 110 | iperf3 測試 |
| `mtr_tool.py` | 120 | MTR 測試 |
| `flent_tool.py` | 50 | Flent 測試 |
| `ros_bw_monitor.py` | 160 | ROS 監控 |
| `plot_utils.py` | 180 | 圖表繪製 |
| **總計** | **1,150** | **模組化版本** |
| `network_tester_unified.py` | 600+ | 舊版單一檔案 |

**優勢**：
- ✅ 雖然總行數增加，但每個檔案都很小且專注
- ✅ 可讀性和維護性大幅提升
- ✅ 測試覆蓋率更容易達成
- ✅ 團隊協作更方便（不會互相衝突）

## 🧪 測試建議

### 單元測試

每個工具模組都可獨立測試：

```python
# test_ping_tool.py
import pytest
from network_tester.tools import PingTool

def test_ping_parse():
    tool = PingTool("8.8.8.8", 10)
    rtts, loss, jitter, stats = tool.parse_results("test_ping.txt")
    assert len(rtts) > 0
    assert loss >= 0
```

### 整合測試

測試主 Node 與工具的協作：

```python
# test_integration.py
def test_full_workflow():
    node = NetworkTestNode()
    node.target = "192.168.0.1"
    node.duration = 5
    node.run()
    # 驗證輸出檔案存在
```

## 🎓 學習價值

這個重構示範了：
1. **SOLID 原則**在實際專案中的應用
2. **模組化設計**如何提升程式碼品質
3. **ROS 2 架構**與 Python 最佳實踐的結合
4. **工程思維**：不僅是寫程式，更是設計系統

## 📚 延伸閱讀

- [Clean Code by Robert C. Martin](https://www.amazon.com/Clean-Code-Handbook-Software-Craftsmanship/dp/0132350882)
- [Design Patterns: Elements of Reusable Object-Oriented Software](https://en.wikipedia.org/wiki/Design_Patterns)
- [ROS 2 Best Practices](https://docs.ros.org/en/rolling/The-ROS2-Project/Contributing/Developer-Guide.html)

---

**總結**：這次重構將一個難以維護的大型檔案，轉變成了一個結構清晰、易於擴展的專業級套件。每個模組都像是一個小型的"微服務"，專注於自己的職責，但又能完美協作。🚀
