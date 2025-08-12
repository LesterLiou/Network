# What is ROSbridge & How to use

## 1. Between DDS & ROSbridge
| 項目       | 原生 DDS（同 ROS\_DOMAIN\_ID）  | rosbridge（WebSocket/JSON）     || 
| -------- | -------------------------- | ------------------------------ | ------------------------------ |
| 角色定位     | ROS 2 預設傳輸/發現              | 透過 WebSocket 橋接非 ROS 客戶端或跨網      
| 加密/驗證    | ✗ 無（明文）                    | ✗ 預設無（可用 **WSS/TLS** 另補）            
| 安全邊界     | **ROS\_DOMAIN\_ID 不是安全邊界** | 以 TCP 連線為邊界（可上 TLS 與帳密）        |               
| 穿越子網/NAT | 差（仰賴多播/UDP 7400+）          | 好（單一 TCP 埠，易過防火牆）              |      
| 延遲/吞吐    | ★★★（最佳）                    | ★（JSON/WS 轉碼，吞吐較低）             |            
| 工具/相容    | 直接用 `ros2 *`、QoS 完整        | 需配 rosbridge\_server；QoS 概念被簡化 |              |
| 維運複雜度    | 低                          | 低\~中（多一個橋接服務）                  |       
| 典型用途     | 同 LAN/同機房、純 ROS 2          | 跨網/瀏覽器/非 ROS 客戶端               |           


## 2.
