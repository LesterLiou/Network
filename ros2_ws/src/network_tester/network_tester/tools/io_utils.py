"""
I/O Utilities Module
====================
處理檔案 I/O、數據轉換和系統網路統計
"""

import csv
import json
from pathlib import Path
from typing import Dict, List, Any

try:
    import psutil
except ImportError:
    psutil = None


def get_io_stats() -> Dict[str, int]:
    """取得系統網路 I/O 統計"""
    if psutil:
        return psutil.net_io_counters()._asdict()
    
    # Fallback: 從 /proc/net/dev 讀取
    stats = {
        'bytes_recv': 0, 'packets_recv': 0, 'dropin': 0,
        'bytes_sent': 0, 'packets_sent': 0, 'dropout': 0
    }
    try:
        with open("/proc/net/dev") as f:
            for line in f.readlines()[2:]:
                parts = line.split()
                if len(parts) >= 17:
                    stats['bytes_recv'] += int(parts[1])
                    stats['packets_recv'] += int(parts[2])
                    stats['dropin'] += int(parts[4])
                    stats['bytes_sent'] += int(parts[9])
                    stats['packets_sent'] += int(parts[10])
                    stats['dropout'] += int(parts[12])
    except Exception as e:
        print(f"[WARN] Failed to read /proc/net/dev: {e}")
    return stats


def safe_float(value: Any, default: float = 0.0) -> float:
    """安全轉換為浮點數"""
    try:
        return float(value) if value is not None else default
    except (ValueError, TypeError):
        return default


def safe_int(value: Any, default: int = 0) -> int:
    """安全轉換為整數"""
    try:
        return int(value) if value is not None else default
    except (ValueError, TypeError):
        return default


class IOUtils:
    """檔案 I/O 工具類"""
    
    @staticmethod
    def append_to_csv(csv_path: Path, row: Dict, headers: List[str]) -> None:
        """
        追加資料到 CSV 檔案
        
        Args:
            csv_path: CSV 檔案路徑
            row: 資料行（字典）
            headers: 欄位標題
        """
        write_header = not csv_path.exists()
        try:
            with open(csv_path, "a", newline='') as f:
                writer = csv.DictWriter(f, fieldnames=headers)
                if write_header:
                    writer.writeheader()
                writer.writerow(row)
        except Exception as e:
            print(f"[ERROR] Failed to write CSV {csv_path}: {e}")
    
    @staticmethod
    def save_json(file_path: Path, data: Dict, indent: int = 2) -> None:
        """
        儲存 JSON 檔案
        
        Args:
            file_path: 檔案路徑
            data: 要儲存的資料
            indent: 縮排空格數
        """
        try:
            with open(file_path, 'w') as f:
                json.dump(data, f, indent=indent)
        except Exception as e:
            print(f"[ERROR] Failed to save JSON {file_path}: {e}")
    
    @staticmethod
    def load_json(file_path: Path) -> Dict:
        """
        載入 JSON 檔案
        
        Args:
            file_path: 檔案路徑
            
        Returns:
            JSON 資料（字典）
        """
        try:
            with open(file_path, 'r') as f:
                return json.load(f)
        except Exception as e:
            print(f"[ERROR] Failed to load JSON {file_path}: {e}")
            return {}
    
    @staticmethod
    def read_text_file(file_path: Path) -> str:
        """
        讀取文字檔案
        
        Args:
            file_path: 檔案路徑
            
        Returns:
            檔案內容
        """
        try:
            with open(file_path, 'r') as f:
                return f.read()
        except Exception as e:
            print(f"[ERROR] Failed to read file {file_path}: {e}")
            return ""
    
    @staticmethod
    def write_csv(csv_path: Path, data: List[Dict], headers: List[str]) -> None:
        """
        寫入 CSV 檔案（覆蓋）
        
        Args:
            csv_path: CSV 檔案路徑
            data: 資料列表
            headers: 欄位標題
        """
        try:
            with open(csv_path, "w", newline='') as f:
                writer = csv.DictWriter(f, fieldnames=headers)
                writer.writeheader()
                writer.writerows(data)
        except Exception as e:
            print(f"[ERROR] Failed to write CSV {csv_path}: {e}")
