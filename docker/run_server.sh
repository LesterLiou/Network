#!/bin/bash

PORT=12865

echo "[INFO] 啟動 netserver on port $PORT..."

# 檢查 netserver 是否已經啟動
if ss -tuln | grep ":$PORT" > /dev/null; then
  echo "[INFO] netserver 已經在執行中（port $PORT 已被佔用）"
else
  echo "[INFO] 尚未啟動，正在啟動 netserver..."
  nohup netserver -p $PORT > netserver.log 2>&1 &
  sleep 1
fi

# 顯示目前監聽的 port 狀態
echo "[INFO] 目前啟用中的服務："
ss -tuln | grep ":$PORT"

# 顯示 netserver 是否真的啟動
echo "[INFO] 確認程序："
ps aux | grep netserver | grep -v grep

echo "[✅] Server 準備完成，可供 Flent 測試用。"

echo "[INFO] 檢查並啟動 iperf3 server..."

# 預設 TCP/UDP 使用的 port（你可以根據 client 設定調整）
IPERF_PORT1=5201  # 用於主測試
IPERF_PORT2=5002  # 用於背景流量

# 啟動 iperf3 5201
if ! ss -tuln | grep ":$IPERF_PORT1" > /dev/null; then
  echo "[INFO] 啟動 iperf3 -s on port $IPERF_PORT1"
  nohup iperf3 -s -p $IPERF_PORT1 > iperf3_${IPERF_PORT1}.log 2>&1 &
else
  echo "[INFO] iperf3 服務已在 port $IPERF_PORT1 運作中"
fi

# 啟動 iperf3 5002（背景負載）
if ! ss -tuln | grep ":$IPERF_PORT2" > /dev/null; then
  echo "[INFO] 啟動 iperf3 -s on port $IPERF_PORT2"
  nohup iperf3 -s -p $IPERF_PORT2 > iperf3_${IPERF_PORT2}.log 2>&1 &
else
  echo "[INFO] iperf3 服務已在 port $IPERF_PORT2 運作中"
fi

