#!/bin/bash

IMAGE_NAME=flent_network
CONTAINER_NAME=flent_network_container

# 允許 X11 顯示畫面
xhost +local:root

echo "[INFO] Running Docker container with GUI support (entering bash)..."
docker run -it --rm \
  --net=host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $(pwd)/..:/root/NETWORK \
  $IMAGE_NAME bash

# 移除本地 X11 權限（安全性）
xhost -local:root


