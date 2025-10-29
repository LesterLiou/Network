"""
ROS Bandwidth Monitor Module
=============================
封裝 ROS 2 topic 頻寬監控功能
"""

import time
from pathlib import Path
from typing import Dict, List
from rclpy.node import Node
from rclpy.serialization import serialize_message
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rosidl_runtime_py.utilities import get_message
from .io_utils import IOUtils


class ROSBandwidthMonitor:
    """ROS 2 Topic 頻寬監控器"""
    
    def __init__(self, node: Node, topic_names: List[str], type_names: List[str], 
                 interval: int = 1, logger=None):
        """
        初始化監控器
        
        Args:
            node: ROS 2 Node 實例
            topic_names: 要監控的 topic 列表
            type_names: 對應的訊息類型列表
            interval: 統計間隔（秒）
            logger: 可選的 logger
        """
        self.node = node
        self.topic_names = topic_names
        self.type_names = type_names
        self.interval = interval
        self.logger = logger
        
        # QoS 設定
        self.sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 計數器
        self._topic_bytes: Dict[str, int] = {}
        self._topic_msgs: Dict[str, int] = {}
        
        # 訂閱者和定時器
        self._subs = []
        self._timer = None
        
        # 數據儲存
        self._bw_data: List[Dict] = []
        self._csv_path: Path = None
    
    def setup(self, output_dir: Path) -> bool:
        """
        設定監控器（建立訂閱者和定時器）
        
        Args:
            output_dir: 輸出目錄
            
        Returns:
            是否成功
        """
        if not self.topic_names:
            if self.logger:
                self.logger.info("📡 No topics to monitor")
            return False
        
        self._csv_path = output_dir / 'ros2_bw.csv'
        
        # 初始化計數器
        for name in self.topic_names:
            self._topic_bytes[name] = 0
            self._topic_msgs[name] = 0
        
        # 建立訂閱者
        for topic_name, type_name in zip(self.topic_names, self.type_names):
            try:
                msg_class = get_message(type_name)
                
                def make_callback(name):
                    def cb(msg):
                        try:
                            payload = serialize_message(msg)
                            self._topic_bytes[name] += len(payload)
                            self._topic_msgs[name] += 1
                        except Exception as e:
                            if self.logger:
                                self.logger.error(f"Serialize error on {name}: {e}")
                    return cb
                
                sub = self.node.create_subscription(
                    msg_class,
                    topic_name,
                    make_callback(topic_name),
                    self.sensor_qos
                )
                self._subs.append(sub)
                
                if self.logger:
                    self.logger.info(f"✅ Subscribed to {topic_name} ({type_name})")
            
            except Exception as e:
                if self.logger:
                    self.logger.error(f"❌ Failed to subscribe {topic_name}: {e}")
        
        # 建立定時器
        self._timer = self.node.create_timer(float(self.interval), self._record_sample)
        
        if self.logger:
            self.logger.info(f"📊 Bandwidth monitor started (interval={self.interval}s)")
        
        return True
    
    def _record_sample(self):
        """記錄一次頻寬採樣"""
        row = {'timestamp': time.time()}
        
        for name in self.topic_names:
            bytes_val = self._topic_bytes[name]
            msgs_val = self._topic_msgs[name]
            
            # 計算頻寬和頻率
            mbps = (bytes_val * 8 / 1e6) / self.interval
            hz = msgs_val / self.interval
            avg_bytes = bytes_val / msgs_val if msgs_val > 0 else 0
            
            row[f'{name}_Mbps'] = round(mbps, 3)
            row[f'{name}_Hz'] = round(hz, 2)
            row[f'{name}_AvgBytes'] = int(avg_bytes)
            
            # 重置計數器
            self._topic_bytes[name] = 0
            self._topic_msgs[name] = 0
        
        # 儲存到記憶體
        self._bw_data.append(row)
        
        # 即時寫入 CSV（保證數據不遺失）
        if self._csv_path:
            headers = list(row.keys())
            IOUtils.append_to_csv(self._csv_path, row, headers)
    
    def get_data(self) -> List[Dict]:
        """
        取得所有採樣數據
        
        Returns:
            採樣數據列表
        """
        return self._bw_data.copy()
    
    def save_summary(self, output_dir: Path) -> None:
        """
        儲存頻寬統計摘要
        
        Args:
            output_dir: 輸出目錄
        """
        if not self._bw_data:
            return
        
        summary = {}
        for name in self.topic_names:
            mbps_key = f'{name}_Mbps'
            hz_key = f'{name}_Hz'
            
            mbps_vals = [r[mbps_key] for r in self._bw_data if mbps_key in r]
            hz_vals = [r[hz_key] for r in self._bw_data if hz_key in r]
            
            if mbps_vals:
                summary[name] = {
                    'mbps_avg': round(sum(mbps_vals) / len(mbps_vals), 3),
                    'mbps_max': round(max(mbps_vals), 3),
                    'mbps_min': round(min(mbps_vals), 3),
                    'hz_avg': round(sum(hz_vals) / len(hz_vals), 2) if hz_vals else 0,
                    'hz_max': round(max(hz_vals), 2) if hz_vals else 0,
                    'samples': len(mbps_vals)
                }
        
        summary_file = output_dir / 'ros2_bw_summary.json'
        IOUtils.save_json(summary_file, summary)
        
        if self.logger:
            self.logger.info(f"💾 Bandwidth summary saved to {summary_file}")
    
    def teardown(self):
        """清理監控器（取消訂閱和定時器）"""
        if self._timer:
            self._timer.cancel()
            self._timer = None
        self._subs.clear()
