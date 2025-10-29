#!/usr/bin/env python3
"""
簡化的網路測試執行腳本
=======================
提供更簡潔的命令列介面

使用範例:
# 基本測試
./run_network_test.py --target 192.168.0.230

# 完整測試
./run_network_test.py \
  --target 192.168.0.230 \
  --duration 30 \
  --load B \
  --vpn-mode no_vpn \
  --topics "/camera/image_raw,/robot/joint_states" \
  --types "sensor_msgs/msg/Image,sensor_msgs/msg/JointState"
"""

import argparse
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(description='網路測試簡化執行腳本')
    
    parser.add_argument('--target', required=True, help='測試目標 IP 位址')
    parser.add_argument('--duration', type=int, default=30, help='測試持續時間（秒）')
    parser.add_argument('--load', default='B', choices=['A', 'B', 'C'], help='負載等級')
    parser.add_argument('--out', default='/root/NETWORK/output', help='輸出目錄')
    parser.add_argument('--vpn-mode', default='auto', choices=['no_vpn', 'with_vpn', 'auto'], 
                       help='VPN 模式')
    parser.add_argument('--topics', default='', help='ROS topics（逗號分隔）')
    parser.add_argument('--types', default='', help='訊息類型（逗號分隔）')
    parser.add_argument('--bw-interval', type=int, default=1, help='頻寬統計間隔（秒）')
    parser.add_argument('--skip-plots', action='store_true', help='跳過圖表生成')
    
    args = parser.parse_args()
    
    # 建構 ROS 2 指令
    cmd = [
        'ros2', 'run', 'network_tester', 'network_test_node',
        '--ros-args',
        '-p', f'target:={args.target}',
        '-p', f'duration:={args.duration}',
        '-p', f'load:={args.load}',
        '-p', f'out:={args.out}',
        '-p', f'vpn_mode:={args.vpn_mode}',
        '-p', f'topics:={args.topics}',
        '-p', f'types:={args.types}',
        '-p', f'bw_interval:={args.bw_interval}',
        '-p', f'skip_plots:={1 if args.skip_plots else 0}',
    ]
    
    print("執行指令:", ' '.join(cmd))
    print()
    
    try:
        subprocess.run(cmd, check=True)
    except subprocess.CalledProcessError as e:
        print(f"測試失敗: {e}", file=sys.stderr)
        sys.exit(1)
    except KeyboardInterrupt:
        print("\n測試被使用者中斷")
        sys.exit(0)


if __name__ == '__main__':
    main()
