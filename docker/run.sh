#!/usr/bin/env bash
set -euo pipefail

IMAGE_NAME=${IMAGE_NAME:-flent_network}
CONTAINER_NAME=${CONTAINER_NAME:-flent_network_container}
MODE=${1:-bash}
ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}

# ---- GUI 檢測（沒 DISPLAY 就略過）----
HAS_X=0
if [[ -n "${DISPLAY-}" ]] && [[ -S /tmp/.X11-unix/X0 ]] && command -v xhost >/dev/null 2>&1; then
  xhost +local:root 1>/dev/null || true
  trap 'xhost -local:root 1>/dev/null || true' EXIT
  HAS_X=1
fi

# ---- 共同參數（先不放 GUI 參數）----
COMMON_ARGS=(
  --rm
  --name "${CONTAINER_NAME}"
  --net=host
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID}"
  -e ROS_LOCALHOST_ONLY=0
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  -v "$(pwd)/..":/root/NETWORK
  --device /dev/bus/usb:/dev/bus/usb
  --privileged
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix

)

# 只有在有 X Server 時才加 GUI 參數
if [[ $HAS_X -eq 1 ]]; then
  COMMON_ARGS+=( -e DISPLAY="$DISPLAY" -v /tmp/.X11-unix:/tmp/.X11-unix )
fi

echo "[INFO] Image: ${IMAGE_NAME}"
echo "[INFO] Mode : ${MODE}"
echo "[INFO] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"

if [[ "${MODE}" == "bash" ]]; then
  exec docker run -it "${COMMON_ARGS[@]}" "${IMAGE_NAME}" bash
elif [[ "${MODE}" == "rosbridge" ]]; then
  exec docker run -it "${COMMON_ARGS[@]}" "${IMAGE_NAME}" /usr/local/bin/start_rosbridge.sh
else
  echo "Usage: $0 [bash|rosbridge]"
  exit 1
fi
