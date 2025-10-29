"""
iperf3 Test Tool Module
========================
封裝 iperf3 測試和結果解析功能
"""

import json
import csv
from pathlib import Path
from subprocess import Popen, STDOUT
from typing import Dict, Optional
from .io_utils import IOUtils


class IperfTool:
    """iperf3 測試工具"""
    
    def __init__(self, target: str, duration: int, streams: int, bandwidth: str, logger=None):
        """
        初始化 iperf3 工具
        
        Args:
            target: 目標 IP 位址
            duration: 測試持續時間（秒）
            streams: 並行串流數
            bandwidth: 頻寬限制（如 '2M', '0' 表示無限制）
            logger: 可選的 logger
        """
        self.target = target
        self.duration = duration
        self.streams = streams
        self.bandwidth = bandwidth
        self.logger = logger
        self._output_file = None
    
    def start(self, output_file: Path) -> Optional[Popen]:
        """
        啟動 iperf3 測試（背景執行）
        
        Args:
            output_file: 輸出檔案路徑（JSON 格式）
            
        Returns:
            Popen 程序物件
        """
        try:
            cmd = [
                'iperf3', '-c', self.target,
                '-t', str(self.duration),
                '-P', str(self.streams),
                '-b', self.bandwidth,
                '-J'
            ]
            # 使用 unbuffered 模式確保輸出立即寫入
            self._output_file = open(output_file, 'w', buffering=1)
            proc = Popen(cmd, stdout=self._output_file, stderr=STDOUT)
            if self.logger:
                self.logger.info(f"✅ iperf3 started -> {output_file}")
            return proc
        except Exception as e:
            if self.logger:
                self.logger.error(f"❌ iperf3 failed: {e}")
            return None
    
    def parse_results(self, iperf_file: Path, include_intervals: bool = False) -> Dict:
        """
        解析 iperf3 JSON 結果
        
        Args:
            iperf_file: iperf3 輸出檔案路徑
            include_intervals: 是否包含詳細間隔數據（預設 False，不包含以節省空間）
            
        Returns:
            統計字典
        """
        stats = {
            'bandwidth_mbps': None,
            'total_retransmits': None,
            'lost_packets': None,
            'lost_percent': None,
        }
        
        # 只有明確要求時才包含 intervals
        if include_intervals:
            stats['intervals'] = []
        
        try:
            content = IOUtils.read_text_file(iperf_file).strip()
            if not content:
                if self.logger:
                    self.logger.warning(f"iperf3 file is empty: {iperf_file}")
                return stats
            
            data = json.loads(content)
            
            # 總體統計
            if 'end' in data and 'sum_received' in data['end']:
                sum_recv = data['end']['sum_received']
                bps = sum_recv.get('bits_per_second', 0)
                stats['bandwidth_mbps'] = round(bps / 1e6, 2)
                stats['lost_packets'] = sum_recv.get('lost_packets', 0)
                stats['lost_percent'] = round(sum_recv.get('lost_percent', 0), 2)
            
            if 'end' in data and 'sum_sent' in data['end']:
                stats['total_retransmits'] = data['end']['sum_sent'].get('retransmits', 0)
            
            # 間隔統計（僅在需要時解析）
            if include_intervals and 'intervals' in data:
                for iv in data['intervals']:
                    if 'sum' in iv:
                        s = iv['sum']
                        stats['intervals'].append({
                            'interval': f"{s.get('start', 0):.1f}-{s.get('end', 0):.1f}",
                            'mbps': round(s.get('bits_per_second', 0) / 1e6, 2),
                            'retransmits': s.get('retransmits', 0)
                        })
        
        except Exception as e:
            if self.logger:
                self.logger.error(f"Failed to parse iperf3: {e}")
        
        return stats
    
    def save_intervals_csv(self, stats: Dict, output_file: Path) -> None:
        """
        儲存 iperf3 間隔數據到 CSV
        
        Args:
            stats: iperf3 統計字典
            output_file: 輸出 CSV 檔案路徑
        """
        intervals = stats.get('intervals', [])
        if not intervals:
            return
        
        try:
            with open(output_file, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['interval', 'mbps', 'retransmits'])
                for iv in intervals:
                    writer.writerow([iv['interval'], iv['mbps'], iv['retransmits']])
            if self.logger:
                self.logger.info(f"💾 iperf intervals CSV saved -> {output_file}")
        except Exception as e:
            if self.logger:
                self.logger.error(f"Failed to save iperf CSV: {e}")
    
    def close(self):
        """關閉輸出檔案句柄"""
        if self._output_file:
            try:
                self._output_file.close()
            except:
                pass
            self._output_file = None
