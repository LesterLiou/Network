# ROSbridge 数据验证指南

## 问题说明

当您使用 ROSbridge 传输数据时，会发现：
- ✅ 客户端显示有频宽统计
- ❌ 服务器端 `ros2 topic list` 看不到 topic
- ❓ 数据到底有没有传过去？

## 根本原因

**ROSbridge 使用 WebSocket (TCP)，不是 ROS 2 DDS**

```
客户端                          服务器端 (10.0.0.88)
━━━━━━                         ━━━━━━━━━━━━━━━━━━━

WebSocket 客户端               ROSbridge Server
     |                              | (port 9090)
     | 订阅通过 WebSocket            |
     └──────────────────────────────┤
                                    |
                             ROS 2 DDS 网络
                             (ros2 topic list 只能看到这里)
```

## 验证方法

### 方法 1：使用验证脚本（推荐）

```bash
# 在服务器端 (10.0.0.88) 执行

# 1. 确保 rosbridge 正在运行
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# 2. 安装 roslibpy
pip3 install roslibpy

# 3. 运行验证脚本
cd /root/NETWORK/ros2_ws/src/network_tester/scripts
python3 verify_rosbridge.py \
  --host 127.0.0.1 \
  --port 9090 \
  --topic /camera/camera/color/image_raw/compressed \
  --type sensor_msgs/CompressedImage \
  --duration 30
```

**成功输出示例**:
```
🔍 ROSbridge Data Verification
======================================================================
Host:     127.0.0.1:9090
Topic:    /camera/camera/color/image_raw/compressed
Type:     sensor_msgs/CompressedImage
Duration: 30s
======================================================================

🔌 Connecting to ws://127.0.0.1:9090...
✅ Connected successfully!

📡 Subscribing to /camera/camera/color/image_raw/compressed...
⏳ Waiting for messages (max 30s)...

✅ First message received at 14:30:25
   Data size: 45231 bytes

📊 Messages: 150 | Rate: 5.0 Hz | Bandwidth: 1.81 Mbps | Elapsed: 30.0s

======================================================================
📊 Final Statistics
======================================================================
✅ Total messages:  150
✅ Total bytes:     6,784,650 (6.78 MB)
✅ Average rate:    5.00 Hz
✅ Average bandwidth: 1.81 Mbps
✅ Duration:        30.00s
======================================================================
```

**失败输出示例**:
```
❌ No messages received!

🔍 Troubleshooting:
   1. Check if topic exists on server:
      ros2 topic list | grep /camera/camera/color/image_raw/compressed
   2. Check if publisher is running
   3. Verify topic name and message type
```

### 方法 2：使用 Python 脚本

在服务器端创建并运行：

```python
#!/usr/bin/env python3
import roslibpy
import time

# 连接到本地 rosbridge
ros = roslibpy.Ros(host='127.0.0.1', port=9090)
ros.run()

if ros.is_connected:
    print("✅ Connected to rosbridge")
    
    # 订阅 topic
    topic = roslibpy.Topic(
        ros, 
        '/camera/camera/color/image_raw/compressed', 
        'sensor_msgs/CompressedImage'
    )
    
    count = 0
    def callback(message):
        global count
        count += 1
        data_size = len(message.get('data', []))
        print(f"✅ Message #{count}: {data_size} bytes")
    
    topic.subscribe(callback)
    
    try:
        print("⏳ Waiting for messages (Ctrl+C to stop)...")
        ros.run_forever()
    except KeyboardInterrupt:
        print(f"\n📊 Total received: {count} messages")
        topic.unsubscribe()
        ros.terminate()
else:
    print("❌ Cannot connect to rosbridge")
```

### 方法 3：检查网络连接

```bash
# 在服务器端查看 WebSocket 连接
netstat -an | grep 9090

# 应该看到类似：
# tcp  0  0  10.0.0.88:9090  10.0.0.44:55432  ESTABLISHED
```

## 当前版本限制

⚠️ **重要**: 当前的模块化版本 (`ros2 launch network_tester network_test.launch.py`) **不支持 ROSbridge**

- ❌ `use_rosbridge` 参数无效
- ❌ `ws_host` 参数无效
- ❌ `ws_port` 参数无效

**解决方案**:

1. **使用 DDS 方式**（推荐）:
```bash
# 两端设置相同的 ROS_DOMAIN_ID
export ROS_DOMAIN_ID=0

# 客户端
ros2 launch network_tester network_test.launch.py \
  target:=10.0.0.88 \
  duration:=30 \
  topics:="/camera/camera/color/image_raw/compressed" \
  types:="sensor_msgs/msg/CompressedImage"
```

2. **等待 ROSbridge 功能实现**（未来版本）

3. **手动测试 ROSbridge**:
```bash
# 服务器端
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# 客户端（使用验证脚本）
python3 verify_rosbridge.py --host 10.0.0.88 --port 9090 --topic /your/topic
```

## 关键概念理解

### DDS vs ROSbridge

| 特性 | DDS 直连 | ROSbridge |
|-----|---------|-----------|
| **协议** | UDP 多播 | WebSocket (TCP) |
| **ROS_DOMAIN_ID** | ✅ 必须匹配 | ❌ 无影响 |
| **ros2 topic list** | ✅ 可见 | ❌ 不可见 |
| **ros2 topic echo** | ✅ 可用 | ❌ 不可用 |
| **跨网段/NAT** | ❌ 困难 | ✅ 容易 |
| **性能** | ⚡ 高 | 🐌 中（有编码开销） |
| **验证方法** | `ros2 topic hz` | `roslibpy` 客户端 |

### 为什么 ros2 topic list 看不到 ROSbridge 的 topic？

ROSbridge 是一个**桥接器**，它：
1. 在服务器端通过 DDS 订阅 ROS topic
2. 将数据转换为 JSON 格式
3. 通过 WebSocket 发送给客户端
4. 客户端接收 WebSocket 数据（**不是 DDS**）

所以：
- 服务器端的 `ros2 topic list` 显示的是 **DDS 网络的 topic**（发布者那边）
- 客户端通过 WebSocket 订阅，**不会出现在 DDS 网络中**
- 这是**设计如此**，不是 bug

## 故障排除

### 问题 1: 验证脚本连接失败

```bash
# 检查 rosbridge 是否运行
ps aux | grep rosbridge

# 检查端口是否开放
nc -zv 127.0.0.1 9090

# 重启 rosbridge
pkill rosbridge
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

### 问题 2: 收不到消息

```bash
# 在服务器端检查 topic 是否存在
ros2 topic list | grep your_topic

# 检查 topic 是否有发布者
ros2 topic info /your/topic

# 检查消息类型是否正确
ros2 topic type /your/topic
```

### 问题 3: roslibpy 安装失败

```bash
# 使用 pip3 安装
pip3 install roslibpy

# 如果有权限问题
pip3 install --user roslibpy

# 或使用 sudo
sudo pip3 install roslibpy
```

## 总结

✅ **ROSbridge 确实在传输数据，只是您看不到而已**

- 用 `verify_rosbridge.py` 脚本验证
- 不要期望在 `ros2 topic list` 中看到
- 如果需要在服务器端看到 topic，请使用 DDS 方式

🔧 **当前版本建议**:
- 优先使用 DDS 方式（method 2.1）
- 如果必须跨网段，手动使用 ROSbridge + 验证脚本
- 等待未来版本的完整 ROSbridge 支持
