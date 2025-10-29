"""
Ping Test Tool Module
======================
封裝 ping 測試和結果解析功能
"""

import re
import csv
from pathlib import Path
from subprocess import Popen, STDOUT
from typing import Tuple, List, Optional


class PingTool:
    """Ping 測試工具"""
    
    def __init__(self, target: str, duration: int, logger=None):
        """
        初始化 Ping 工具
        
        Args:
            target: 目標 IP 位址
            duration: 測試持續時間（秒）
            logger: 可選的 logger
        """
        self.target = target
        self.duration = duration
        self.logger = logger
        self.ping_count = duration * 5  # 0.2s 間隔
    
    def start(self, output_file: Path) -> Optional[Popen]:
        """
        啟動 Ping 測試（背景執行）
        
        Args:
            output_file: 輸出檔案路徑
            
        Returns:
            Popen 程序物件
        """
        try:
            proc = Popen(
                ['ping', '-i', '0.2', '-c', str(self.ping_count), self.target],
                stdout=open(output_file, 'w'),
                stderr=STDOUT
            )
            if self.logger:
                self.logger.info(f"✅ Ping started -> {output_file}")
            return proc
        except Exception as e:
            if self.logger:
                self.logger.error(f"❌ Ping failed: {e}")
            return None
    
    def parse_results(self, ping_file: Path) -> Tuple[List[float], float, float, dict]:
        """
        解析 Ping 結果
        
        Args:
            ping_file: Ping 輸出檔案路徑
            
        Returns:
            (rtts, loss_pct, jitter_ms, stats_dict)
            - rtts: RTT 列表
            - loss_pct: 丟包率
            - jitter_ms: 抖動（ms）
            - stats_dict: 統計字典
        """
        rtts = []
        try:
            with open(ping_file, 'r') as f:
                for line in f:
                    match = re.search(r'time[=<]([0-9.]+)\s*ms', line)
                    if match:
                        rtts.append(float(match.group(1)))
        except Exception as e:
            if self.logger:
                self.logger.error(f"Failed to parse ping: {e}")
        
        received = len(rtts)
        loss_pct = round((self.ping_count - received) / self.ping_count * 100, 2) if self.ping_count > 0 else 100.0
        
        # 計算 jitter（相鄰 RTT 差值的平均）
        jitter = 0.0
        if len(rtts) > 1:
            diffs = [abs(rtts[i] - rtts[i-1]) for i in range(1, len(rtts))]
            jitter = round(sum(diffs) / len(diffs), 2)
        
        # 組裝統計
        stats = {
            'count': self.ping_count,
            'received': received,
            'loss_percent': loss_pct,
            'rtt_avg_ms': round(sum(rtts) / len(rtts), 2) if rtts else None,
            'rtt_max_ms': round(max(rtts), 2) if rtts else None,
            'rtt_min_ms': round(min(rtts), 2) if rtts else None,
            'jitter_ms': jitter,
            'bufferbloat_ms': round(max(rtts) - min(rtts), 2) if rtts else None
        }
        
        return rtts, loss_pct, jitter, stats
    
    def save_csv(self, rtts: List[float], output_file: Path) -> None:
        """
        儲存 RTT 數據到 CSV
        
        Args:
            rtts: RTT 列表
            output_file: 輸出 CSV 檔案路徑
        """
        try:
            with open(output_file, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['sample', 'rtt_ms'])
                for i, rtt in enumerate(rtts):
                    writer.writerow([i+1, rtt])
            if self.logger:
                self.logger.info(f"💾 Ping CSV saved -> {output_file}")
        except Exception as e:
            if self.logger:
                self.logger.error(f"Failed to save ping CSV: {e}")
