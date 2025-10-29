# ROSbridge 功能使用指南

## ✅ 功能已实现

ROSbridge 支持已完全集成到模块化版本中！

## 🚀 快速开始

### 方法 1：使用自动化测试脚本（最简单）

```bash
# 基本用法（使用默认参数）
/root/NETWORK/test_rosbridge.sh 10.0.0.88

# 指定 topic 和消息类型
/root/NETWORK/test_rosbridge.sh \
  10.0.0.88 \
  /camera/camera/color/image_raw/compressed \
  sensor_msgs/msg/CompressedImage
```

这个脚本会自动：
1. ✅ 检查环境和依赖
2. ✅ 验证 ROSbridge 连接
3. ✅ 快速数据验证（5秒）
4. ✅ 运行完整网络测试（30秒）
5. ✅ 显示结果摘要

### 方法 2：使用 Launch 文件（完全控制）

```bash
# 基本 ROSbridge 测试
ros2 launch network_tester network_test.launch.py \
  target:=10.0.0.88 \
  duration:=30 \
  load:=B \
  topics:="/camera/camera/color/image_raw/compressed" \
  types:="sensor_msgs/msg/CompressedImage" \
  use_rosbridge:=1 \
  ws_host:=10.0.0.88 \
  ws_port:=9090

# 多 topic 测试
ros2 launch network_tester network_test.launch.py \
  target:=10.0.0.88 \
  duration:=30 \
  load:=B \
  topics:="/camera/image,/robot/joint_states" \
  types:="sensor_msgs/msg/CompressedImage,sensor_msgs/msg/JointState" \
  use_rosbridge:=1
```

### 方法 3：仅验证 ROSbridge 数据

```bash
cd /root/NETWORK/ros2_ws/src/network_tester/scripts
python3 verify_rosbridge.py \
  --host 10.0.0.88 \
  --port 9090 \
  --topic /camera/camera/color/image_raw/compressed \
  --type sensor_msgs/CompressedImage \
  --duration 30
```

## 📋 参数说明

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `use_rosbridge` | `0` | 启用 ROSbridge（1=启用，0=使用DDS）|
| `ws_host` | 同 `target` | ROSbridge 服务器地址 |
| `ws_port` | `9090` | ROSbridge 端口 |
| `target` | 必需 | 测试目标 IP |
| `topics` | 必需 | Topic 名称（逗号分隔）|
| `types` | 必需 | 消息类型（逗号分隔）|

## 📊 输出文件

使用 ROSbridge 时，会生成以下额外文件：

```
output/2025-10-29_15-30-45_with_vpn/
├── rosbridge_bw.csv                  # ROSbridge 频宽时间序列
├── rosbridge_bw_summary.json         # ROSbridge 统计摘要
└── image/
    └── chart_rosbridge_bw_*.png      # ROSbridge 频宽图表
```

### rosbridge_bw_summary.json 示例

```json
{
  "ws_host": "10.0.0.88",
  "ws_port": 9090,
  "connection_method": "rosbridge",
  "topics": {
    "/camera/camera/color/image_raw/compressed": {
      "avg_Mbps": 2.34,
      "max_Mbps": 3.12,
      "min_Mbps": 1.98,
      "avg_Hz": 10.5,
      "max_Hz": 12.0,
      "total_samples": 30
    }
  }
}
```

## 🔍 验证数据是否真的传输

### 问题：为什么 `ros2 topic list` 看不到？

**这是正常的！** ROSbridge 使用 WebSocket，不是 DDS。

### 解决方案：使用验证脚本

```bash
# 在服务器端运行
python3 /root/NETWORK/ros2_ws/src/network_tester/scripts/verify_rosbridge.py \
  --host 127.0.0.1 \
  --port 9090 \
  --topic /camera/camera/color/image_raw/compressed
```

**成功输出**：
```
✅ Connected successfully!
📡 Subscribing to /camera/camera/color/image_raw/compressed...
✅ First message received at 14:30:25
   Data size: 45231 bytes
📊 Messages: 150 | Rate: 5.0 Hz | Bandwidth: 1.81 Mbps | Elapsed: 30.0s
```

**失败输出**：
```
❌ No messages received!

🔍 Troubleshooting:
   1. Check if topic exists on server: ros2 topic list
   2. Check if publisher is running
   3. Verify topic name and message type
```

## 🆚 DDS vs ROSbridge 对比

| 特性 | DDS 直连 | ROSbridge |
|------|---------|-----------|
| **协议** | UDP 多播 | WebSocket (TCP) |
| **ROS_DOMAIN_ID** | ✅ 必须匹配 | ❌ 无影响 |
| **ros2 topic list** | ✅ 可见 | ❌ 不可见 |
| **跨网段/NAT** | ❌ 困难 | ✅ 容易 |
| **性能** | ⚡ 高 | 🐌 中（有编码开销） |
| **配置复杂度** | 🟢 简单 | 🟡 需要服务器 |
| **适用场景** | 同网段 | 跨网段/VPN |

## 🛠️ 故障排除

### 1. 连接失败

```bash
# 检查 ROSbridge 服务器
nc -zv 10.0.0.88 9090

# 如果失败，在服务器端启动
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

### 2. roslibpy 未安装

```bash
pip3 install roslibpy
```

### 3. 收不到消息

```bash
# 在服务器端检查 topic
ros2 topic list | grep your_topic
ros2 topic echo /your/topic --once

# 检查消息类型
ros2 topic type /your/topic
```

### 4. 性能问题

ROSbridge 比 DDS 慢 20-40%，因为：
- JSON 编码/解码开销
- TCP 而非 UDP
- 单线程 WebSocket 连接

**解决方案**：
- 如果在同网段，使用 DDS（`use_rosbridge:=0`）
- 降低图像分辨率或帧率
- 使用压缩消息类型（CompressedImage）

## 📝 完整示例

### 场景：跨 VPN 测试相机数据

```bash
# Step 1: 服务器端 (10.0.0.88) - 启动 rosbridge
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# Step 2: 服务器端 - 启动相机
ros2 run usb_cam usb_cam_node_exe

# Step 3: 客户端 - 验证连接
nc -zv 10.0.0.88 9090

# Step 4: 客户端 - 快速验证数据（可选）
python3 /root/NETWORK/ros2_ws/src/network_tester/scripts/verify_rosbridge.py \
  --host 10.0.0.88 \
  --port 9090 \
  --topic /camera/camera/color/image_raw/compressed \
  --duration 5

# Step 5: 客户端 - 完整网络测试
ros2 launch network_tester network_test.launch.py \
  target:=10.0.0.88 \
  duration:=30 \
  load:=B \
  topics:="/camera/camera/color/image_raw/compressed" \
  types:="sensor_msgs/msg/CompressedImage" \
  use_rosbridge:=1 \
  ws_host:=10.0.0.88 \
  ws_port:=9090

# Step 6: 查看结果
cd /root/NETWORK/output
ls -lt | head -5
cd $(ls -td */ | head -1)
cat rosbridge_bw_summary.json | python3 -m json.tool
```

## 🎓 高级用法

### 同时测试 DDS 和 ROSbridge

```bash
# 测试 1: DDS
ros2 launch network_tester network_test.launch.py \
  target:=10.0.0.88 \
  topics:="/camera/image" \
  types:="sensor_msgs/msg/CompressedImage" \
  vpn_mode:=with_vpn

# 测试 2: ROSbridge
ros2 launch network_tester network_test.launch.py \
  target:=10.0.0.88 \
  topics:="/camera/image" \
  types:="sensor_msgs/msg/CompressedImage" \
  use_rosbridge:=1 \
  vpn_mode:=with_vpn

# 对比结果
python3 /root/NETWORK/ros2_ws/src/network_tester/scripts/analyze_results.py \
  /root/NETWORK/output/*_with_vpn --compare
```

### 批量测试不同负载

```bash
#!/bin/bash
for load in A B C; do
    echo "Testing load $load with ROSbridge"
    ros2 launch network_tester network_test.launch.py \
      target:=10.0.0.88 \
      duration:=30 \
      load:=$load \
      topics:="/camera/image" \
      types:="sensor_msgs/msg/CompressedImage" \
      use_rosbridge:=1
    sleep 10
done
```

## 📚 相关文档

- [ROSbridge 验证指南](../docs/ROSBRIDGE_VERIFICATION.md)
- [主 README](../README.md)
- [架构文档](ros2_ws/src/network_tester/ARCHITECTURE.md)

## ❓ FAQ

**Q: ROSbridge 比 DDS 慢多少？**
A: 通常慢 20-40%，因为 JSON 编码和 TCP 开销。

**Q: 可以同时使用 DDS 和 ROSbridge 吗？**
A: 不能在同一次测试中，但可以分别测试后对比。

**Q: ROS_DOMAIN_ID 对 ROSbridge 有影响吗？**
A: 完全没有影响。ROSbridge 使用 WebSocket，与 DDS 独立。

**Q: 如何知道数据真的传过去了？**
A: 使用 `verify_rosbridge.py` 脚本或查看 `rosbridge_bw_summary.json`。

**Q: 为什么服务器端 ros2 topic list 看不到客户端？**
A: 这是正常的！ROSbridge 客户端通过 WebSocket 订阅，不在 DDS 网络中。
