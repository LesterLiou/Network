#!/bin/bash

TARGET=${TARGET_HOST:-8.8.8.8}
DURATION=${DURATION:-60}
OUTPUT=/output/rrul_${TARGET}.png

echo "[INFO] Running RRUL test to $TARGET for $DURATION seconds..."
flent rrul -H $TARGET -l $DURATION -p all -o $OUTPUT
