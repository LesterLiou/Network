"""
Network Tester Package
=======================
ROS 2 網路測試工具套件（模組化版本）

主要功能：
- Ping RTT 測試
- iperf3 TCP throughput 測試
- MTR 路由追蹤
- Flent RRUL 綜合測試
- ROS 2 topic 頻寬監控
- 系統網路 I/O 統計
- 自動圖表生成

使用範例：
    # 使用 launch 檔案
    ros2 launch network_tester network_test.launch.py target:=192.168.0.230
    
    # 直接執行 node
    ros2 run network_tester network_test_node --ros-args -p target:=192.168.0.230

版本：1.0.0（模組化重構版）
"""

__version__ = '1.0.0'
__author__ = 'Network Tester Team'

from .node_network_tester import NetworkTestNode

__all__ = ['NetworkTestNode']
