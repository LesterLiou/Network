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
