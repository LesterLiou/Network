"""
Plot Utilities Module
======================
封裝所有圖表繪製功能
"""

from pathlib import Path
from typing import List, Dict

try:
    import matplotlib
    matplotlib.use('Agg')  # Non-interactive backend
    import matplotlib.pyplot as plt
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False


class PlotUtils:
    """圖表繪製工具"""
    
    @staticmethod
    def is_available() -> bool:
        """檢查 matplotlib 是否可用"""
        return HAS_MATPLOTLIB
    
    @staticmethod
    def plot_ping_rtt(rtts: List[float], vpn_mode: str, output_file: Path) -> bool:
        """
        繪製 Ping RTT 時序圖
        
        Args:
            rtts: RTT 列表
            vpn_mode: VPN 模式標記
            output_file: 輸出檔案路徑
            
        Returns:
            是否成功
        """
        if not HAS_MATPLOTLIB or not rtts:
            return False
        
        try:
            plt.figure(figsize=(12, 5))
            plt.plot(rtts, linewidth=0.8, alpha=0.7)
            plt.xlabel('Sample Number')
            plt.ylabel('RTT (ms)')
            plt.title(f'Ping RTT Over Time ({vpn_mode})')
            plt.grid(alpha=0.3)
            
            # 統計標註
            avg = sum(rtts) / len(rtts)
            plt.axhline(avg, color='r', linestyle='--', linewidth=1, label=f'Avg: {avg:.2f}ms')
            plt.axhline(max(rtts), color='orange', linestyle=':', linewidth=1, label=f'Max: {max(rtts):.2f}ms')
            plt.axhline(min(rtts), color='green', linestyle=':', linewidth=1, label=f'Min: {min(rtts):.2f}ms')
            plt.legend()
            
            plt.tight_layout()
            plt.savefig(output_file, dpi=150)
            plt.close()
            return True
        
        except Exception as e:
            print(f"[ERROR] Failed to plot RTT: {e}")
            return False
    
    @staticmethod
    def plot_throughput(intervals: List[Dict], vpn_mode: str, output_file: Path) -> bool:
        """
        繪製 iperf3 Throughput 時序圖
        
        Args:
            intervals: iperf3 間隔數據列表
            vpn_mode: VPN 模式標記
            output_file: 輸出檔案路徑
            
        Returns:
            是否成功
        """
        if not HAS_MATPLOTLIB or not intervals:
            return False
        
        try:
            plt.figure(figsize=(12, 5))
            x = list(range(len(intervals)))
            y = [iv['mbps'] for iv in intervals]
            
            plt.plot(x, y, marker='o', linewidth=1.5)
            plt.xlabel('Interval')
            plt.ylabel('Throughput (Mbps)')
            plt.title(f'iperf3 Throughput Over Time ({vpn_mode})')
            plt.grid(alpha=0.3)
            
            if y:
                avg = sum(y) / len(y)
                plt.axhline(avg, color='r', linestyle='--', linewidth=1, label=f'Avg: {avg:.2f} Mbps')
                plt.legend()
            
            plt.tight_layout()
            plt.savefig(output_file, dpi=150)
            plt.close()
            return True
        
        except Exception as e:
            print(f"[ERROR] Failed to plot throughput: {e}")
            return False
    
    @staticmethod
    def plot_mtr_hops(hops: List[Dict], vpn_mode: str, output_file: Path) -> bool:
        """
        繪製 MTR 逐跳延遲圖
        
        Args:
            hops: MTR 路由資料列表
            vpn_mode: VPN 模式標記
            output_file: 輸出檔案路徑
            
        Returns:
            是否成功
        """
        if not HAS_MATPLOTLIB or not hops:
            return False
        
        try:
            plt.figure(figsize=(14, 6))
            
            hop_nums = [h['hop'] for h in hops]
            avg_lats = [h['avg_ms'] for h in hops]
            worst_lats = [h['worst_ms'] for h in hops]
            best_lats = [h['best_ms'] for h in hops]
            
            plt.plot(hop_nums, avg_lats, marker='o', label='Avg', linewidth=2)
            plt.plot(hop_nums, worst_lats, marker='v', label='Worst', linewidth=1, linestyle='--', alpha=0.6)
            plt.plot(hop_nums, best_lats, marker='^', label='Best', linewidth=1, linestyle='--', alpha=0.6)
            
            plt.xlabel('Hop Number')
            plt.ylabel('Latency (ms)')
            plt.title(f'MTR Hop-by-Hop Latency ({vpn_mode})')
            plt.grid(alpha=0.3)
            plt.legend()
            
            # 標註主機名稱
            for i, hop in enumerate(hops):
                if i % 2 == 0:  # 避免標籤過密
                    plt.text(hop['hop'], hop['avg_ms'], hop['host'], 
                            fontsize=7, rotation=45, verticalalignment='bottom')
            
            plt.tight_layout()
            plt.savefig(output_file, dpi=150)
            plt.close()
            return True
        
        except Exception as e:
            print(f"[ERROR] Failed to plot MTR: {e}")
            return False
    
    @staticmethod
    def plot_ros_bandwidth(topic_name: str, bw_data: List[Dict], vpn_mode: str, 
                          output_file: Path) -> bool:
        """
        繪製 ROS 2 Topic 頻寬圖
        
        Args:
            topic_name: Topic 名稱
            bw_data: 頻寬數據列表
            vpn_mode: VPN 模式標記
            output_file: 輸出檔案路徑
            
        Returns:
            是否成功
        """
        if not HAS_MATPLOTLIB or not bw_data:
            return False
        
        try:
            mbps_key = f'{topic_name}_Mbps'
            hz_key = f'{topic_name}_Hz'
            
            mbps_vals = [r[mbps_key] for r in bw_data if mbps_key in r]
            hz_vals = [r[hz_key] for r in bw_data if hz_key in r]
            
            if not mbps_vals:
                return False
            
            fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
            
            # 頻寬圖
            ax1.plot(mbps_vals, linewidth=1.5)
            ax1.set_xlabel('Sample')
            ax1.set_ylabel('Bandwidth (Mbps)')
            ax1.set_title(f'ROS 2 Topic Bandwidth: {topic_name} ({vpn_mode})')
            ax1.grid(alpha=0.3)
            
            avg_mbps = sum(mbps_vals) / len(mbps_vals)
            ax1.axhline(avg_mbps, color='r', linestyle='--', linewidth=1, label=f'Avg: {avg_mbps:.2f} Mbps')
            ax1.legend()
            
            # 頻率圖
            if hz_vals:
                ax2.plot(hz_vals, linewidth=1.5, color='orange')
                ax2.set_xlabel('Sample')
                ax2.set_ylabel('Frequency (Hz)')
                ax2.set_title(f'Message Rate: {topic_name}')
                ax2.grid(alpha=0.3)
                
                avg_hz = sum(hz_vals) / len(hz_vals)
                ax2.axhline(avg_hz, color='r', linestyle='--', linewidth=1, label=f'Avg: {avg_hz:.2f} Hz')
                ax2.legend()
            
            plt.tight_layout()
            plt.savefig(output_file, dpi=150)
            plt.close()
            return True
        
        except Exception as e:
            print(f"[ERROR] Failed to plot ROS bandwidth: {e}")
            return False
