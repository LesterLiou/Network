#!/usr/bin/env python3
"""
ROSbridge Bandwidth Monitor
============================
通过 WebSocket 订阅 ROSbridge topics 并监控频宽

依赖:
    pip3 install roslibpy

使用範例:
    monitor = ROSbridgeMonitor(
        ws_host='10.0.0.88',
        ws_port=9090,
        topic_names=['/camera/image_raw'],
        type_names=['sensor_msgs/CompressedImage'],
        interval=1.0,
        logger=node.get_logger()
    )
    monitor.setup(output_dir)
    # ... wait ...
    monitor.save_summary(output_dir)
    monitor.teardown()
"""

import time
import csv
import base64
from pathlib import Path
from typing import List, Dict, Optional
import threading

try:
    import roslibpy
    ROSLIBPY_AVAILABLE = True
except ImportError:
    ROSLIBPY_AVAILABLE = False
    print("⚠️  Warning: roslibpy not available. Install with: pip3 install roslibpy")


class ROSbridgeMonitor:
    """ROSbridge 频宽监控器"""
    
    def __init__(
        self, 
        ws_host: str,
        ws_port: int,
        topic_names: List[str],
        type_names: List[str],
        interval: float = 1.0,
        logger=None
    ):
        """
        初始化 ROSbridge 监控器
        
        Args:
            ws_host: ROSbridge 服务器地址
            ws_port: ROSbridge 端口（通常是 9090）
            topic_names: 要监控的 topic 列表
            type_names: 对应的消息类型列表
            interval: 采样间隔（秒）
            logger: ROS logger（可选）
        """
        if not ROSLIBPY_AVAILABLE:
            raise ImportError("roslibpy is required for ROSbridge monitoring. Install with: pip3 install roslibpy")
        
        self.ws_host = ws_host
        self.ws_port = ws_port
        self.topic_names = topic_names
        self.type_names = type_names
        self.interval = interval
        self.logger = logger
        
        self.ros_client = None
        self.topics = []
        self.csv_file = None
        self.csv_writer = None
        
        # 统计数据
        self._topic_bytes = {name: 0 for name in topic_names}
        self._topic_msgs = {name: 0 for name in topic_names}
        self._data_rows = []
        
        # 线程控制
        self._running = False
        self._record_thread = None
        self._event_thread = None  # 新增：事件循环线程
        self._last_sample_time = None
    
    def _log(self, level: str, msg: str):
        """内部日志函数"""
        if self.logger:
            if level == 'info':
                self.logger.info(msg)
            elif level == 'warn':
                self.logger.warn(msg)
            elif level == 'error':
                self.logger.error(msg)
        else:
            print(f"[{level.upper()}] {msg}")
    
    def setup(self, output_dir: Path):
        """
        设置并启动监控
        
        Args:
            output_dir: 输出目录
        """
        self._log('info', f"🔌 Connecting to ROSbridge at ws://{self.ws_host}:{self.ws_port}...")
        
        try:
            # 连接到 ROSbridge
            self.ros_client = roslibpy.Ros(host=self.ws_host, port=self.ws_port)
            
            # 启动 roslibpy 事件循环（非阻塞）
            # 注意：run() 会启动后台的 Twisted reactor
            self.ros_client.run()
            
            # 等待连接建立
            timeout = 5.0
            start_time = time.time()
            while not self.ros_client.is_connected and (time.time() - start_time) < timeout:
                time.sleep(0.1)
            
            if not self.ros_client.is_connected:
                self._log('error', f"❌ Failed to connect to ROSbridge at {self.ws_host}:{self.ws_port}")
                self._log('error', "   Make sure rosbridge server is running:")
                self._log('error', "   ros2 launch rosbridge_server rosbridge_websocket_launch.xml")
                return False
            
            self._log('info', f"✅ Connected to ROSbridge successfully")
            
            # 订阅所有 topics
            for topic_name, type_name in zip(self.topic_names, self.type_names):
                # 转换消息类型格式 (sensor_msgs/msg/Image -> sensor_msgs/Image)
                rosbridge_type = type_name
                
                self._log('info', f"📡 Subscribing to {topic_name} ({rosbridge_type})...")
                
                topic = roslibpy.Topic(
                    self.ros_client,
                    topic_name,
                    rosbridge_type
                )
                
                # 创建回调函数（使用闭包捕获 topic_name）
                def make_callback(name):
                    def callback(message):
                        self._on_message(name, message)
                    return callback
                
                callback_fn = make_callback(topic_name)
                topic.subscribe(callback_fn)
                self.topics.append((topic, callback_fn))  # 保存 topic 和 callback
                
                self._log('info', f"✅ Subscribed to {topic_name}")
            
            # 打开 CSV 文件
            self.csv_file = open(output_dir / 'rosbridge_bw.csv', 'w', newline='')
            fieldnames = ['timestamp']
            for name in self.topic_names:
                fieldnames.extend([f'{name}_Mbps', f'{name}_Hz'])
            self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=fieldnames)
            self.csv_writer.writeheader()
            
            # 启动记录线程
            self._running = True
            self._last_sample_time = time.time()
            self._record_thread = threading.Thread(target=self._record_loop, daemon=True)
            self._record_thread.start()
            
            self._log('info', f"✅ ROSbridge monitoring started (interval: {self.interval}s)")
            return True
        
        except Exception as e:
            self._log('error', f"❌ Failed to setup ROSbridge monitor: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def _on_message(self, topic_name: str, message: dict):
        """处理接收到的消息"""
        try:
            # 估算消息大小
            # 对于 CompressedImage，主要数据在 'data' 字段
            if 'data' in message:
                data_size = len(base64.b64decode(message['data']))
            else:
                # 其他消息类型，估算 JSON 大小
                import json
                data_size = len(json.dumps(message).encode('utf-8'))
            
            self._topic_bytes[topic_name] += data_size
            self._topic_msgs[topic_name] += 1
            
            # Debug: 打印第一条消息
            if self._topic_msgs[topic_name] == 1:
                self._log('info', f"✅ First message received on {topic_name}, size: {data_size} bytes")
        except Exception as e:
            self._log('error', f"❌ Error in _on_message for {topic_name}: {e}")
    
    def _record_loop(self):
        """后台记录循环"""
        while self._running:
            current_time = time.time()
            elapsed = current_time - self._last_sample_time
            
            if elapsed >= self.interval:
                self._record_sample()
                self._last_sample_time = current_time
            
            time.sleep(0.1)
    
    def _record_sample(self):
        """记录一次采样"""
        row = {'timestamp': time.time()}
        
        for name in self.topic_names:
            # 计算 Mbps 和 Hz
            mbps = (self._topic_bytes[name] * 8 / 1e6) / self.interval
            hz = self._topic_msgs[name] / self.interval
            
            row[f'{name}_Mbps'] = round(mbps, 3)
            row[f'{name}_Hz'] = round(hz, 2)
            
            # 重置计数器
            self._topic_bytes[name] = 0
            self._topic_msgs[name] = 0
        
        # 保存到内存和文件
        self._data_rows.append(row)
        if self.csv_writer:
            self.csv_writer.writerow(row)
            self.csv_file.flush()
    
    def get_data(self) -> List[Dict]:
        """获取所有记录的数据"""
        return self._data_rows
    
    def save_summary(self, output_dir: Path):
        """保存统计摘要"""
        if not self._data_rows:
            self._log('warn', "⚠️  No ROSbridge data to summarize")
            return
        
        summary = {
            'ws_host': self.ws_host,
            'ws_port': self.ws_port,
            'topics': {},
            'connection_method': 'rosbridge'
        }
        
        for topic_name in self.topic_names:
            mbps_values = [row[f'{topic_name}_Mbps'] for row in self._data_rows]
            hz_values = [row[f'{topic_name}_Hz'] for row in self._data_rows]
            
            if mbps_values:
                summary['topics'][topic_name] = {
                    'avg_Mbps': round(sum(mbps_values) / len(mbps_values), 3),
                    'max_Mbps': round(max(mbps_values), 3),
                    'min_Mbps': round(min(mbps_values), 3),
                    'avg_Hz': round(sum(hz_values) / len(hz_values), 2),
                    'max_Hz': round(max(hz_values), 2),
                    'total_samples': len(mbps_values)
                }
        
        # 保存为 JSON
        from .io_utils import IOUtils
        summary_file = output_dir / 'rosbridge_bw_summary.json'
        IOUtils.save_json(summary_file, summary)
        self._log('info', f"💾 ROSbridge summary saved -> {summary_file}")
    
    def teardown(self):
        """清理资源"""
        self._log('info', "🛑 Stopping ROSbridge monitor...")
        
        # 停止记录线程
        self._running = False
        if self._record_thread:
            self._record_thread.join(timeout=2.0)
        
        # 取消订阅
        for topic, callback_fn in self.topics:
            try:
                topic.unsubscribe(callback_fn)
            except:
                pass
        
        # 关闭连接
        if self.ros_client and self.ros_client.is_connected:
            try:
                self.ros_client.terminate()
            except:
                pass
        
        # 关闭文件
        if self.csv_file:
            self.csv_file.close()
        
        self._log('info', "✅ ROSbridge monitor stopped")
    
    @staticmethod
    def is_available() -> bool:
        """检查 roslibpy 是否可用"""
        return ROSLIBPY_AVAILABLE
