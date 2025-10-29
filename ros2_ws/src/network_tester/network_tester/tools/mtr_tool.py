"""
MTR Test Tool Module
====================
封裝 MTR 測試和結果解析功能
"""

import json
import csv
from pathlib import Path
from subprocess import Popen, STDOUT
from typing import Dict, Optional
from .io_utils import safe_float, safe_int


class MTRTool:
    """MTR 測試工具"""
    
    def __init__(self, target: str, logger=None):
        """
        初始化 MTR 工具
        
        Args:
            target: 目標 IP 位址
            logger: 可選的 logger
        """
        self.target = target
        self.logger = logger
    
    def start(self, output_file: Path, count: int = 10) -> Optional[Popen]:
        """
        啟動 MTR 測試（背景執行）
        
        Args:
            output_file: 輸出檔案路徑（JSON 格式）
            count: 測試次數
            
        Returns:
            Popen 程序物件
        """
        try:
            proc = Popen(
                ['mtr', '-rwzc', str(count), '-j', self.target],
                stdout=open(output_file, 'w'),
                stderr=STDOUT
            )
            if self.logger:
                self.logger.info(f"✅ MTR started -> {output_file}")
            return proc
        except Exception as e:
            if self.logger:
                self.logger.error(f"❌ MTR failed: {e}")
            return None
    
    def parse_results(self, mtr_file: Path) -> Dict:
        """
        解析 MTR JSON 結果
        
        Args:
            mtr_file: MTR 輸出檔案路徑
            
        Returns:
            統計字典
        """
        stats = {
            'hops': [],
            'total_hops': 0,
            'avg_latency_ms': None,
            'total_loss_pct': 0.0
        }
        
        try:
            with open(mtr_file, 'r') as f:
                data = json.load(f)
            
            if 'report' in data and 'hubs' in data['report']:
                hubs = data['report']['hubs']
                stats['total_hops'] = len(hubs)
                
                for hub in hubs:
                    hop_info = {
                        'hop': hub.get('count', 0),
                        'host': hub.get('host', 'N/A'),
                        'loss_pct': safe_float(hub.get('Loss%', 0)),
                        'snt': safe_int(hub.get('Snt', 0)),
                        'last_ms': safe_float(hub.get('Last', 0)),
                        'avg_ms': safe_float(hub.get('Avg', 0)),
                        'best_ms': safe_float(hub.get('Best', 0)),
                        'worst_ms': safe_float(hub.get('Wrst', 0)),
                        'stdev_ms': safe_float(hub.get('StDev', 0))
                    }
                    stats['hops'].append(hop_info)
                
                # 計算平均延遲（最後一跳）
                if hubs:
                    stats['avg_latency_ms'] = safe_float(hubs[-1].get('Avg', 0))
                    stats['total_loss_pct'] = safe_float(hubs[-1].get('Loss%', 0))
        
        except Exception as e:
            if self.logger:
                self.logger.error(f"Failed to parse MTR: {e}")
        
        return stats
    
    def save_hops_csv(self, stats: Dict, output_file: Path) -> None:
        """
        儲存 MTR 路由數據到 CSV
        
        Args:
            stats: MTR 統計字典
            output_file: 輸出 CSV 檔案路徑
        """
        hops = stats.get('hops', [])
        if not hops:
            return
        
        try:
            with open(output_file, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    'hop', 'host', 'loss_pct', 'snt', 
                    'last_ms', 'avg_ms', 'best_ms', 'worst_ms', 'stdev_ms'
                ])
                for hop in hops:
                    writer.writerow([
                        hop['hop'], hop['host'], hop['loss_pct'], hop['snt'],
                        hop['last_ms'], hop['avg_ms'], hop['best_ms'], 
                        hop['worst_ms'], hop['stdev_ms']
                    ])
            if self.logger:
                self.logger.info(f"💾 MTR hops CSV saved -> {output_file}")
        except Exception as e:
            if self.logger:
                self.logger.error(f"Failed to save MTR CSV: {e}")
