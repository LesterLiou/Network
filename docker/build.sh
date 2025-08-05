#!/bin/bash

set -e  # 出錯時中止腳本

IMAGE_NAME=flent_network
DOCKERFILE_PATH=.

GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${GREEN}[INFO] Building Docker image: $IMAGE_NAME...${NC}"
START_TIME=$(date +%s)

docker build -t $IMAGE_NAME $DOCKERFILE_PATH

END_TIME=$(date +%s)
BUILD_TIME=$((END_TIME - START_TIME))

echo -e "${GREEN}[SUCCESS] Build completed in ${BUILD_TIME}s.${NC}"