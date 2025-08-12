#!/usr/bin/env bash
set -euo pipefail

# ====== 基本參數 ======
IMAGE_NAME=${IMAGE_NAME:-flent_network}
CONTAINER_NAME=${CONTAINER_NAME:-flent_network_container}

# 用法：./run.sh [bash|rosbridge]
MODE=${1:-bash}

# ROS 網域（同網內想互通就設一樣）
ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-7}

# ====== X11 顯示授權（要跑 image_view 時用得到）======
xhost +local:root 1>/dev/null || true
cleanup() { xhost -local:root 1>/dev/null || true; }
trap cleanup EXIT

# ====== 共用參數（host 網路最省事，內網 rosbridge 推薦）======
COMMON_ARGS=(
  --rm
  --name "${CONTAINER_NAME}"
  --net=host                   
  -e DISPLAY="$DISPLAY"
  -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID"
  -e ROS_LOCALHOST_ONLY=0
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  -v /tmp/.X11-unix:/tmp/.X11-unix
  -v "$(pwd)/..":/root/NETWORK   
  --device /dev/bus/usb:/dev/bus/usb  # RealSense/USB
  --privileged                   
)

echo "[INFO] Image: ${IMAGE_NAME}"
echo "[INFO] Mode : ${MODE}"
echo "[INFO] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"

if [[ "${MODE}" == "bash" ]]; then
  echo "[INFO] Entering container shell..."
  exec docker run -it "${COMMON_ARGS[@]}" "${IMAGE_NAME}" bash

elif [[ "${MODE}" == "rosbridge" ]]; then
  echo "[INFO] Starting rosbridge (and RealSense if啟動腳本內有) ..."
  # 你若在 Dockerfile 有 COPY start_rosbridge.sh
  exec docker run -it "${COMMON_ARGS[@]}" "${IMAGE_NAME}" /usr/local/bin/start_rosbridge.sh

else
  echo "Usage: $0 [bash|rosbridge]"
  exit 1
fi
