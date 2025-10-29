"""
Flent Test Tool Module
=======================
封裝 Flent RRUL 測試功能
"""

from pathlib import Path
from subprocess import Popen
from typing import Optional


class FlentTool:
    """Flent RRUL 測試工具"""
    
    def __init__(self, target: str, duration: int, logger=None):
        """
        初始化 Flent 工具
        
        Args:
            target: 目標 IP 位址
            duration: 測試持續時間（秒）
            logger: 可選的 logger
        """
        self.target = target
        self.duration = duration
        self.logger = logger
    
    def run(self, output_base: Path, timeout: int = None) -> bool:
        """
        執行 Flent RRUL 測試（同步執行）
        
        Args:
            output_base: 輸出檔案基礎路徑（不含副檔名）
            timeout: 超時時間（秒），預設為 duration + 30
            
        Returns:
            是否成功
        """
        if timeout is None:
            timeout = self.duration + 30
        
        try:
            proc = Popen([
                'flent', 'rrul',
                '-p', 'all_scaled',
                '-l', str(self.duration),
                '-s', '0.2',
                '--socket-stats',
                '-H', self.target,
                '-o', str(output_base) + '.png'
            ])
            proc.wait(timeout=timeout)
            
            if self.logger:
                self.logger.info(f"✅ Flent completed -> {output_base}.png")
            return True
            
        except Exception as e:
            if self.logger:
                self.logger.warn(f"⚠️  Flent failed: {e}")
            return False
