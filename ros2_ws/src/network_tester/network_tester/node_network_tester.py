#!/usr/bin/env python3
"""
Network Tester Node (Modularized)
==================================
主要 ROS 2 Node，協調所有網路測試工具

使用範例:
ros2 run network_tester network_test_node \
  --ros-args \
  -p target:=192.168.0.230 \
  -p duration:=30 \
  -p load:=B \
  -p out:="/root/NETWORK/output" \
  -p vpn_mode:=auto \
  -p topics:="/camera/image_raw,/robot/joint_states" \
  -p types:="sensor_msgs/msg/Image,sensor_msgs/msg/JointState"
"""

import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node

from .tools import (
    VPNDetector,
    IOUtils,
    get_io_stats,
    PingTool,
    IperfTool,
    MTRTool,
    FlentTool,
    ROSBandwidthMonitor,
    ROSbridgeMonitor,
    PlotUtils
)


class NetworkTestNode(Node):
    """網路測試主 Node"""
    
    # 負載設定
    LOAD_CONFIG = {
        'A': {'streams': 1, 'bandwidth': '0'},
        'B': {'streams': 5, 'bandwidth': '2M'},
        'C': {'streams': 5, 'bandwidth': '24M'},
    }
    
    def __init__(self):
        super().__init__('network_test_node')
        
        # ---- 宣告參數 ----
        self.declare_parameter('target', '--')
        self.declare_parameter('duration', 30)
        self.declare_parameter('load', 'B')
        self.declare_parameter('out', '/root/NETWORK/output')
        self.declare_parameter('vpn_mode', 'auto')
        self.declare_parameter('topics', '')
        self.declare_parameter('types', '')
        self.declare_parameter('bw_interval', 1)
        self.declare_parameter('skip_plots', 0)
        
        # ROSbridge 参数
        self.declare_parameter('use_rosbridge', 0)
        self.declare_parameter('ws_host', '')
        self.declare_parameter('ws_port', 9090)
        
        # ---- 讀取參數 ----
        self.target = self.get_parameter('target').get_parameter_value().string_value
        self.duration = int(self.get_parameter('duration').get_parameter_value().integer_value)
        self.load = self.get_parameter('load').get_parameter_value().string_value
        self.out_dir = self.get_parameter('out').get_parameter_value().string_value
        self.vpn_mode = self.get_parameter('vpn_mode').get_parameter_value().string_value
        self.bw_interval = int(self.get_parameter('bw_interval').get_parameter_value().integer_value)
        self.skip_plots = bool(int(self.get_parameter('skip_plots').get_parameter_value().integer_value))
        
        # ROSbridge 参数
        self.use_rosbridge = bool(int(self.get_parameter('use_rosbridge').get_parameter_value().integer_value))
        self.ws_host = self.get_parameter('ws_host').get_parameter_value().string_value
        self.ws_port = int(self.get_parameter('ws_port').get_parameter_value().integer_value)
        
        # 如果使用 ROSbridge 但未指定 host，则使用 target
        if self.use_rosbridge and not self.ws_host:
            self.ws_host = self.target
        
        # ROS 2 Topics
        topics_str = self.get_parameter('topics').get_parameter_value().string_value
        types_str = self.get_parameter('types').get_parameter_value().string_value
        self.topic_names = [t.strip() for t in topics_str.split(',') if t.strip()]
        self.type_names = [t.strip() for t in types_str.split(',') if t.strip()]
        
        if self.topic_names and len(self.topic_names) != len(self.type_names):
            self.get_logger().error(f"Topics count ({len(self.topic_names)}) != Types count ({len(self.type_names)})")
            raise ValueError("Topics and types mismatch")
        
        # ---- VPN 模式自動偵測 ----
        if self.vpn_mode == 'auto':
            self.vpn_mode = VPNDetector.detect(self.target, self.get_logger())
        
        # ---- 顯示設定 ----
        self.get_logger().info("=" * 70)
        self.get_logger().info("🚀 Network Test Configuration:")
        self.get_logger().info(f"  Target: {self.target}")
        self.get_logger().info(f"  Duration: {self.duration}s")
        self.get_logger().info(f"  Load: {self.load}")
        self.get_logger().info(f"  VPN Mode: {self.vpn_mode}")
        self.get_logger().info(f"  Topics: {self.topic_names}")
        self.get_logger().info(f"  Use ROSbridge: {self.use_rosbridge}")
        if self.use_rosbridge:
            self.get_logger().info(f"  ROSbridge: ws://{self.ws_host}:{self.ws_port}")
        self.get_logger().info(f"  Skip Plots: {self.skip_plots}")
        self.get_logger().info("=" * 70)
        
        # ---- ROS 頻寬監控器 ----
        self.bw_monitor = None
        self.rosbridge_monitor = None
        
        if self.topic_names:
            if self.use_rosbridge:
                # 使用 ROSbridge 监控
                if not ROSbridgeMonitor.is_available():
                    self.get_logger().error("❌ roslibpy not available. Install with: pip3 install roslibpy")
                    raise ImportError("roslibpy is required for ROSbridge monitoring")
                
                self.rosbridge_monitor = ROSbridgeMonitor(
                    self.ws_host,
                    self.ws_port,
                    self.topic_names,
                    self.type_names,
                    self.bw_interval,
                    self.get_logger()
                )
            else:
                # 使用 DDS 监控
                self.bw_monitor = ROSBandwidthMonitor(
                    self, 
                    self.topic_names, 
                    self.type_names,
                    self.bw_interval,
                    self.get_logger()
                )
    
    def run(self):
        """執行完整測試流程"""
        
        # 取得負載設定
        cfg = self.LOAD_CONFIG.get(self.load, self.LOAD_CONFIG['B'])
        
        # 建立輸出目錄
        ts = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        base = Path(self.out_dir) / f"{ts}_{self.vpn_mode}"
        base.mkdir(parents=True, exist_ok=True)
        
        self.get_logger().info(f"📁 Output directory: {base}")
        
        # 初始化 metadata
        meta = {
            'target': self.target,
            'duration': self.duration,
            'load': self.load,
            'vpn_mode': self.vpn_mode,
            'timestamp': ts,
            'topics': self.topic_names,
            'test_start_time': datetime.now().isoformat()
        }
        
        start_time = time.time()
        meta['net_before'] = get_io_stats()
        
        # ========================================================================
        # 步驟 1: 啟動 ROS 頻寬監控
        # ========================================================================
        if self.bw_monitor:
            self.bw_monitor.setup(base)
        elif self.rosbridge_monitor:
            self.rosbridge_monitor.setup(base)
        
        # ========================================================================
        # 步驟 2: 啟動網路測試工具（背景執行）
        # ========================================================================
        self.get_logger().info("Starting network tests...")
        procs = []
        
        # 2.1 Ping 測試
        ping_tool = PingTool(self.target, self.duration, self.get_logger())
        ping_file = base / f"ping_{ts}.txt"
        ping_proc = ping_tool.start(ping_file)
        if ping_proc:
            procs.append(('ping', ping_proc))
        
        # 2.2 iperf3 測試
        iperf_tool = IperfTool(
            self.target, 
            self.duration, 
            cfg['streams'], 
            cfg['bandwidth'],
            self.get_logger()
        )
        iperf_file = base / f"iperf_tcp_{ts}.json"
        iperf_proc = iperf_tool.start(iperf_file)
        if iperf_proc:
            procs.append(('iperf3', iperf_proc))
        
        # 2.3 MTR 測試
        mtr_tool = MTRTool(self.target, self.get_logger())
        mtr_file = base / f"mtr_{ts}.json"
        mtr_proc = mtr_tool.start(mtr_file)
        if mtr_proc:
            procs.append(('mtr', mtr_proc))
        
        # ========================================================================
        # 步驟 3: ROS Spin 期間持續監控
        # ========================================================================
        self.get_logger().info(f"⏳ Running for {self.duration}s...")
        deadline = time.time() + self.duration
        
        try:
            # 如果使用 ROSbridge，不需要 spin（ROSbridge 有自己的事件循环）
            if self.rosbridge_monitor:
                while time.time() < deadline and rclpy.ok():
                    time.sleep(0.1)
            else:
                # 使用 DDS，需要 spin
                while time.time() < deadline and rclpy.ok():
                    rclpy.spin_once(self, timeout_sec=0.1)
        except KeyboardInterrupt:
            self.get_logger().warn("⚠️  Test interrupted by user")
        
        # ========================================================================
        # 步驟 4: 等待所有子程序完成
        # ========================================================================
        self.get_logger().info("⏸️  Waiting for background processes...")
        
        # 設定合理的 timeout：duration + 額外緩衝時間
        process_timeout = self.duration + 30
        
        for name, proc in procs:
            try:
                proc.wait(timeout=process_timeout)
                self.get_logger().info(f"✅ {name} completed")
            except Exception as e:
                self.get_logger().warn(f"⚠️  {name} timeout after {process_timeout}s: {e}")
                proc.kill()
        
        # 確保文件句柄關閉
        if hasattr(iperf_tool, 'close'):
            iperf_tool.close()
        
        end_time = time.time()
        meta['test_end_time'] = datetime.now().isoformat()
        meta['completion_sec'] = round(end_time - start_time, 2)
        
        # ========================================================================
        # 步驟 5: 解析和保存原始數據
        # ========================================================================
        self.get_logger().info("💾 Parsing and saving raw data...")
        
        # 5.1 系統網路統計
        net_after = get_io_stats()
        meta['net_after'] = net_after
        meta['rx_MB'] = round((net_after['bytes_recv'] - meta['net_before']['bytes_recv']) / 1e6, 2)
        meta['tx_MB'] = round((net_after['bytes_sent'] - meta['net_before']['bytes_sent']) / 1e6, 2)
        meta['packet_drop_rx'] = net_after['dropin'] - meta['net_before']['dropin']
        meta['packet_drop_tx'] = net_after['dropout'] - meta['net_before']['dropout']
        
        # 5.2 解析 Ping
        rtts, loss_pct, jitter_ms, ping_stats = ping_tool.parse_results(ping_file)
        meta['ping'] = ping_stats
        ping_tool.save_csv(rtts, base / f"ping_stats_{ts}.csv")
        
        # 5.3 解析 iperf3
        # 注意：不在 metadata 中包含 intervals 以節省空間
        # 詳細的間隔數據會單獨保存到 CSV 檔案
        iperf_stats = iperf_tool.parse_results(iperf_file, include_intervals=False)
        meta['iperf'] = iperf_stats
        
        # 但仍然需要解析一次 intervals 來保存 CSV（用於圖表生成）
        iperf_stats_full = iperf_tool.parse_results(iperf_file, include_intervals=True)
        iperf_tool.save_intervals_csv(iperf_stats_full, base / f"iperf_intervals_{ts}.csv")
        
        # 5.4 解析 MTR
        mtr_stats = mtr_tool.parse_results(mtr_file)
        meta['mtr'] = mtr_stats
        mtr_tool.save_hops_csv(mtr_stats, base / f"mtr_hops_{ts}.csv")
        
        # 5.5 儲存 ROS 頻寬統計
        if self.bw_monitor:
            self.bw_monitor.save_summary(base)
            self.bw_monitor.teardown()
        elif self.rosbridge_monitor:
            self.rosbridge_monitor.save_summary(base)
            self.rosbridge_monitor.teardown()
        
        # 5.6 儲存 metadata
        meta_file = base / f"meta_{ts}.json"
        IOUtils.save_json(meta_file, meta)
        self.get_logger().info(f"💾 Metadata saved -> {meta_file}")
        
        # 5.7 更新 summary.csv
        self._update_summary(meta, ts, base)
        
        # ========================================================================
        # 步驟 6: Flent RRUL 測試
        # ========================================================================
        self.get_logger().info("🌐 Running Flent RRUL test...")
        
        # 創建 image 目錄（如果還沒創建）
        image_dir = base / 'image'
        image_dir.mkdir(exist_ok=True)
        
        flent_tool = FlentTool(self.target, self.duration, self.get_logger())
        flent_base = image_dir / f"rrul_{ts}"
        if flent_tool.run(flent_base):
            meta['flent_plot'] = str(flent_base) + '.png'
        
        # ========================================================================
        # 步驟 7: 生成圖表
        # ========================================================================
        if not self.skip_plots and PlotUtils.is_available():
            self.get_logger().info("📊 Generating charts...")
            # 使用包含 intervals 的完整數據來生成圖表
            self._generate_charts(base, ts, rtts, iperf_stats_full, mtr_stats)
        elif not PlotUtils.is_available():
            self.get_logger().warn("⚠️  matplotlib not available, skipping charts")
        else:
            self.get_logger().info("⏭️  Skipping chart generation (skip_plots=True)")
        
        # ========================================================================
        # 完成
        # ========================================================================
        self.get_logger().info("=" * 70)
        self.get_logger().info("✅ Test completed successfully!")
        self.get_logger().info(f"📁 Results saved to: {base}")
        self.get_logger().info(f"⏱️  Total time: {meta['completion_sec']}s")
        self.get_logger().info("=" * 70)
    
    def _update_summary(self, meta: dict, ts: str, base: Path):
        """更新 summary.csv"""
        summary_csv = Path(self.out_dir) / 'summary.csv'
        summary_row = {
            'timestamp': ts,
            'vpn_mode': self.vpn_mode,
            'target': self.target,
            'load': self.load,
            'duration_s': self.duration,
            'rtt_avg_ms': meta['ping'].get('rtt_avg_ms'),
            'rtt_max_ms': meta['ping'].get('rtt_max_ms'),
            'loss_pct': meta['ping'].get('loss_percent'),
            'jitter_ms': meta['ping'].get('jitter_ms'),
            'bufferbloat_ms': meta['ping'].get('bufferbloat_ms'),
            'bw_mbps': meta['iperf'].get('bandwidth_mbps'),
            'retrans': meta['iperf'].get('total_retransmits'),
            'mtr_hops': len(meta['mtr'].get('hops', [])),
            'rx_MB': meta['rx_MB'],
            'tx_MB': meta['tx_MB'],
            'drop_rx': meta['packet_drop_rx'],
            'drop_tx': meta['packet_drop_tx'],
            'completion_s': meta['completion_sec']
        }
        IOUtils.append_to_csv(summary_csv, summary_row, list(summary_row.keys()))
        self.get_logger().info(f"💾 Summary updated -> {summary_csv}")
    
    def _generate_charts(self, base: Path, ts: str, rtts: list, iperf_stats: dict, mtr_stats: dict):
        """生成所有圖表"""
        try:
            # 創建 image 子目錄
            image_dir = base / 'image'
            image_dir.mkdir(exist_ok=True)
            self.get_logger().info(f"📁 Charts will be saved to: {image_dir}")
            
            # RTT 圖
            PlotUtils.plot_ping_rtt(rtts, self.vpn_mode, image_dir / f'chart_rtt_{ts}.png')
            
            # Throughput 圖
            PlotUtils.plot_throughput(
                iperf_stats.get('intervals', []), 
                self.vpn_mode, 
                image_dir / f'chart_throughput_{ts}.png'
            )
            
            # MTR 圖
            PlotUtils.plot_mtr_hops(
                mtr_stats.get('hops', []), 
                self.vpn_mode, 
                image_dir / f'chart_mtr_{ts}.png'
            )
            
            # ROS 頻寬圖
            if self.bw_monitor:
                bw_data = self.bw_monitor.get_data()
                for topic_name in self.topic_names:
                    safe_name = topic_name.replace('/', '_')
                    PlotUtils.plot_ros_bandwidth(
                        topic_name,
                        bw_data,
                        self.vpn_mode,
                        image_dir / f'chart_ros2_bw_{safe_name}_{ts}.png'
                    )
            elif self.rosbridge_monitor:
                bw_data = self.rosbridge_monitor.get_data()
                for topic_name in self.topic_names:
                    safe_name = topic_name.replace('/', '_')
                    PlotUtils.plot_ros_bandwidth(
                        topic_name,
                        bw_data,
                        self.vpn_mode,
                        image_dir / f'chart_rosbridge_bw_{safe_name}_{ts}.png'
                    )
            
            self.get_logger().info(f"✅ Charts generated successfully in {image_dir}")
        
        except Exception as e:
            self.get_logger().error(f"⚠️  Chart generation failed: {e}")


def main(args=None):
    """主入口"""
    rclpy.init(args=args)
    node = NetworkTestNode()
    
    try:
        node.run()
    except Exception as e:
        node.get_logger().error(f"Test failed: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if node.bw_monitor:
            node.bw_monitor.teardown()
        if node.rosbridge_monitor:
            node.rosbridge_monitor.teardown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
