"""
Network Test Launch File
=========================
簡化啟動網路測試的 ROS 2 launch 檔案

使用範例:
# 基本測試（自動偵測 VPN）
ros2 launch network_tester network_test.launch.py target:=192.168.0.230

# 完整參數測試
ros2 launch network_tester network_test.launch.py \
  target:=192.168.0.230 \
  duration:=30 \
  load:=B \
  vpn_mode:=no_vpn \
  topics:="/camera/image_raw,/robot/joint_states" \
  types:="sensor_msgs/msg/Image,sensor_msgs/msg/JointState"
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # ---- 宣告 Launch 參數 ----
        DeclareLaunchArgument(
            'target',
            default_value='--',
            description='測試目標 IP 位址'
        ),
        DeclareLaunchArgument(
            'duration',
            default_value='30',
            description='測試持續時間（秒）'
        ),
        DeclareLaunchArgument(
            'load',
            default_value='B',
            description='負載等級 (A|B|C)'
        ),
        DeclareLaunchArgument(
            'out',
            default_value='/root/NETWORK/output',
            description='輸出目錄路徑'
        ),
        DeclareLaunchArgument(
            'vpn_mode',
            default_value='auto',
            description='VPN 模式 (no_vpn|with_vpn|auto)'
        ),
        DeclareLaunchArgument(
            'topics',
            default_value='',
            description='要監控的 ROS topics（逗號分隔）'
        ),
        DeclareLaunchArgument(
            'types',
            default_value='',
            description='對應的訊息類型（逗號分隔）'
        ),
        DeclareLaunchArgument(
            'bw_interval',
            default_value='1',
            description='頻寬統計間隔（秒）'
        ),
        DeclareLaunchArgument(
            'skip_plots',
            default_value='0',
            description='跳過圖表生成 (0|1)'
        ),
        DeclareLaunchArgument(
            'use_rosbridge',
            default_value='0',
            description='使用 ROSbridge 而非 DDS (0|1)'
        ),
        DeclareLaunchArgument(
            'ws_host',
            default_value='',
            description='ROSbridge 服務器地址（默認使用 target）'
        ),
        DeclareLaunchArgument(
            'ws_port',
            default_value='9090',
            description='ROSbridge 端口（默認 9090）'
        ),
        
        # ---- 啟動 Network Test Node ----
        Node(
            package='network_tester',
            executable='network_test_node',
            name='network_test_node',
            output='screen',
            parameters=[{
                'target': LaunchConfiguration('target'),
                'duration': LaunchConfiguration('duration'),
                'load': LaunchConfiguration('load'),
                'out': LaunchConfiguration('out'),
                'vpn_mode': LaunchConfiguration('vpn_mode'),
                'topics': LaunchConfiguration('topics'),
                'types': LaunchConfiguration('types'),
                'bw_interval': LaunchConfiguration('bw_interval'),
                'skip_plots': LaunchConfiguration('skip_plots'),
                'use_rosbridge': LaunchConfiguration('use_rosbridge'),
                'ws_host': LaunchConfiguration('ws_host'),
                'ws_port': LaunchConfiguration('ws_port'),
            }]
        ),
    ])
