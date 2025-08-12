#!/usr/bin/env bash
set -euo pipefail

# ---- 暫時關掉 nounset，避免 ROS setup.bash 讀未定義變數時爆掉 ----
set +u
source /opt/ros/${ROS_DISTRO:-humble}/setup.bash
if [ -f /root/NETWORK/ros2_ws/install/setup.bash ]; then
  source /root/NETWORK/ros2_ws/install/setup.bash
fi
set -u
# ---------------------------------------------------------------------

# 只開內網 rosbridge
exec ros2 launch rosbridge_server rosbridge_websocket_launch.xml \
  address:=192.168.0.5 port:=9090

