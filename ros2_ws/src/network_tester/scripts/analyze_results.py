#!/usr/bin/env python3
"""
批次分析歷史測試結果
======================
掃描 output 目錄中的所有測試結果，生成統計報告

使用範例:
./analyze_results.py /root/NETWORK/output
"""

import argparse
import json
from pathlib import Path
from datetime import datetime


def analyze_directory(output_dir: Path):
    """分析輸出目錄中的所有測試結果"""
    
    results = []
    
    # 掃描所有測試目錄
    for test_dir in sorted(output_dir.iterdir()):
        if not test_dir.is_dir():
            continue
        
        # 尋找 meta JSON 檔案
        meta_files = list(test_dir.glob('meta_*.json'))
        if not meta_files:
            continue
        
        meta_file = meta_files[0]
        
        try:
            with open(meta_file, 'r') as f:
                meta = json.load(f)
            
            result = {
                'test_dir': test_dir.name,
                'timestamp': meta.get('timestamp'),
                'target': meta.get('target'),
                'vpn_mode': meta.get('vpn_mode'),
                'load': meta.get('load'),
                'duration': meta.get('duration'),
                'rtt_avg': meta.get('ping', {}).get('rtt_avg_ms'),
                'rtt_max': meta.get('ping', {}).get('rtt_max_ms'),
                'loss_pct': meta.get('ping', {}).get('loss_percent'),
                'jitter': meta.get('ping', {}).get('jitter_ms'),
                'bandwidth': meta.get('iperf', {}).get('bandwidth_mbps'),
                'retransmits': meta.get('iperf', {}).get('total_retransmits'),
                'mtr_hops': meta.get('mtr', {}).get('total_hops'),
                'rx_MB': meta.get('rx_MB'),
                'tx_MB': meta.get('tx_MB'),
            }
            
            results.append(result)
        
        except Exception as e:
            print(f"[WARN] Failed to parse {meta_file}: {e}")
    
    return results


def print_summary(results: list):
    """印出統計摘要"""
    
    if not results:
        print("沒有找到測試結果")
        return
    
    print(f"\n找到 {len(results)} 個測試結果\n")
    print("=" * 120)
    print(f"{'測試時間':<20} {'VPN模式':<12} {'目標':<18} {'負載':<6} "
          f"{'RTT平均':<10} {'丟包率':<8} {'頻寬':<12} {'重傳':<8}")
    print("=" * 120)
    
    for r in results:
        print(f"{r['timestamp']:<20} {r['vpn_mode']:<12} {r['target']:<18} {r['load']:<6} "
              f"{r['rtt_avg']:<10} {r['loss_pct']:<8} {r['bandwidth']:<12} {r['retransmits']:<8}")
    
    print("=" * 120)
    
    # 統計分析
    vpn_groups = {}
    for r in results:
        mode = r['vpn_mode']
        if mode not in vpn_groups:
            vpn_groups[mode] = []
        vpn_groups[mode].append(r)
    
    print("\n📊 按 VPN 模式分組統計:\n")
    
    for mode, group in vpn_groups.items():
        rtts = [r['rtt_avg'] for r in group if r['rtt_avg'] is not None]
        bws = [r['bandwidth'] for r in group if r['bandwidth'] is not None]
        
        if rtts and bws:
            print(f"  {mode}:")
            print(f"    測試次數: {len(group)}")
            print(f"    平均 RTT: {sum(rtts)/len(rtts):.2f} ms")
            print(f"    平均頻寬: {sum(bws)/len(bws):.2f} Mbps")
            print()


def main():
    parser = argparse.ArgumentParser(description='批次分析歷史測試結果')
    parser.add_argument('output_dir', type=Path, help='輸出目錄路徑')
    parser.add_argument('--export', type=Path, help='匯出統計到 JSON 檔案')
    
    args = parser.parse_args()
    
    if not args.output_dir.exists():
        print(f"錯誤: 目錄不存在 {args.output_dir}")
        return
    
    print(f"分析目錄: {args.output_dir}")
    
    results = analyze_directory(args.output_dir)
    print_summary(results)
    
    if args.export:
        with open(args.export, 'w') as f:
            json.dump(results, f, indent=2)
        print(f"\n統計已匯出到: {args.export}")


if __name__ == '__main__':
    main()
