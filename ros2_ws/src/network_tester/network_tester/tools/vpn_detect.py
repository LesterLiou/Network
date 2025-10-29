"""
VPN Detection Module
====================
自動偵測 VPN 連線狀態
"""

from subprocess import Popen, PIPE
from typing import Optional


class VPNDetector:
    """VPN 模式偵測器"""
    
    @staticmethod
    def detect(target_ip: str, logger=None) -> str:
        """
        自動偵測 VPN 模式
        
        Args:
            target_ip: 目標 IP 位址
            logger: 可選的 logger（ROS Node logger）
            
        Returns:
            'with_vpn' 或 'no_vpn'
        """
        try:
            # 方法 1: 檢查 tun/tap 介面（傳統 VPN）
            result = Popen(['ip', 'route'], stdout=PIPE, stderr=PIPE, text=True)
            stdout, _ = result.communicate(timeout=2)
            if 'tun' in stdout.lower() or 'tap' in stdout.lower():
                if logger:
                    logger.info("偵測到 VPN（tun/tap 介面）")
                return 'with_vpn'
            
            # 方法 2: 檢查目標 IP 是否在特定 VPN 網段
            # 只檢查 10.0.0.x 和 10.8.0.x，避免誤判其他 10.x 私有網段
            target_parts = target_ip.split('.')
            if len(target_parts) == 4:
                if (target_parts[0] == '10' and target_parts[1] == '0' and target_parts[2] == '0') or \
                   (target_parts[0] == '10' and target_parts[1] == '8' and target_parts[2] == '0'):
                    if logger:
                        logger.info(f"偵測到 VPN IP 範圍: {target_ip}")
                    return 'with_vpn'
            
            # 方法 3: 檢查本機 IP 是否在特定 VPN 網段
            result = Popen(['hostname', '-I'], stdout=PIPE, stderr=PIPE, text=True)
            stdout, _ = result.communicate(timeout=2)
            ips = stdout.strip().split()
            for ip in ips:
                ip_parts = ip.split('.')
                if len(ip_parts) == 4:
                    # 只檢查 10.0.0.x 和 10.8.0.x
                    if (ip_parts[0] == '10' and ip_parts[1] == '0' and ip_parts[2] == '0') or \
                       (ip_parts[0] == '10' and ip_parts[1] == '8' and ip_parts[2] == '0'):
                        if logger:
                            logger.info(f"偵測到本機使用 VPN IP: {ip}")
                        return 'with_vpn'
                
        except Exception as e:
            if logger:
                logger.warn(f"VPN 偵測失敗: {e}")
        
        return 'no_vpn'
