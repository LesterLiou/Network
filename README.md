# Network Performance Testing Suite for ROS 2

**Complete ROS 2 Network Performance Testing Toolkit** - Supports TCP throughput, latency, jitter, MTR route analysis, Flent bufferbloat testing, and ROS 2 topic bandwidth monitoring.

[![ROS 2 Humble](https://img.shields.io/badge/ROS_2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Ubuntu 22.04](https://img.shields.io/badge/Ubuntu-22.04-orange)](https://ubuntu.com/)
[![Python 3.10](https://img.shields.io/badge/Python-3.10-green)](https://www.python.org/)

---

## Table of Contents

- [Features](#-features)
- [System Requirements](#-system-requirements)
- [Quick Start](#-quick-start)
- [Detailed Usage Guide](#-detailed-usage-guide)
- [Output Files Explanation](#-output-files-explanation)
- [Results Analysis](#-results-analysis)
- [Advanced Usage](#-advanced-usage)
- [Troubleshooting](#-troubleshooting)
- [Project Architecture](#-project-architecture)

---

## Features

### Network Testing Tools
- ✅ **Ping Test**: RTT, packet loss, jitter, bufferbloat
- ✅ **iperf3**: TCP/UDP throughput, retransmissions
- ✅ **MTR**: Hop-by-hop route latency analysis
- ✅ **Flent RRUL**: Comprehensive bufferbloat testing

### ROS 2 Integration
- ✅ **Topic Bandwidth Monitoring**: Real-time statistics for any ROS 2 topic
- ✅ **Multi-Topic Support**: Monitor multiple topics simultaneously
- ✅ **Automatic QoS**: Best Effort configuration for sensor data
- ✅ **DDS or ROSbridge**: Support for both communication methods

### Automation & Analysis
- ✅ **VPN Auto-Detection**: Intelligent VPN connection detection
- ✅ **Chart Generation**: Auto-generate RTT, throughput, route charts
- ✅ **Data Export**: Complete CSV and JSON format output
- ✅ **Batch Analysis**: Compare multiple test results

---

## System Requirements


### Software Requirements
| Software | Version | Required |
|----------|---------|----------|
| Ubuntu | 22.04 LTS | ✅ Required |
| ROS 2 | Humble | ✅ Required |
| Python | 3.10+ | ✅ Required |
| Docker | 20.10+ | 🔶 Recommended |
| iperf3 | 3.9+ | ✅ Required |
| mtr | 0.93+ | ✅ Required |
| flent | 2.0+ | 🔶 Optional |

---

## Quick Start

### Step 1: Clone the Repository

```bash
# Clone repository
git clone https://github.com/LesterLiou/Network.git
cd Network

# Check file structure
ls -la
```

### Step 2: Environment Setup 

#### Using Docker

```bash
# Build Docker image
cd docker
./build.sh

# Start container
./run.sh
```



### Step 3: Build ROS 2 Package

```bash
# Enter workspace
cd /root/NETWORK/ros2_ws

# Build package
r
# colcon build --packages-select network_tester

# Source environment
# source install/setup.bash

# Verify installation
ros2 pkg list | grep network_tester
```

**Expected Output**:
```
network_tester
```

### Step 4: Prepare Test Environment

#### 🖥️ Server Side Setup (Machine Under Test)

```bash
# Start server (without_rosbridge)
./run.sh

# Start server (with_rosbridge)
./run.sh rosbridge

cd ~/NETWORK        # run server
./run_server.sh
```

#### 📱 Client Side Setup (Testing Machine)

```bash
# Start client
./run.sh
```

### Step 5: Run Your First Test (In Client)

```bash
# Basic network test (no ROS Topics)
ros2 launch network_tester network_test.launch.py \
  target:=192.168.0.230 \
  duration:=30 \
  load:=B

# Wait for test completion (approx. 30-60 seconds)
```

**During Test Execution**:
```
🚀 Network Test Configuration:
  Target: 192.168.0.230
  Duration: 30s
  Load: B
  VPN Mode: no_vpn
...
✅ Test completed successfully!
📁 Results saved to: /root/NETWORK/output/2025-10-28_14-30-15_no_vpn
```

**You'll see**:
- `meta_*.json` - Test configuration and statistics summary
- `image/` directory - All charts
- `*.csv` - Raw data (can be opened with Excel)
- `ping_*.txt`, `iperf_tcp_*.json` - Detailed test logs


---

## 📘 Detailed Usage Guide

### Test Scenario 1: Basic Network Performance Test

**Purpose**: Test TCP throughput, latency, jitter

```bash
ros2 launch network_tester network_test.launch.py \
  target:=192.168.0.230 \
  duration:=30 \
  load:=B
```

**Parameter Explanation**:
- `target`: Test target IP
- `duration`: Test duration (seconds)
- `load`: Load level
  - `A`: Light load (1 stream, unlimited)
  - `B`: Medium load (5 streams, 2 Mbps)
  - `C`: Heavy load (5 streams, 24 Mbps)

---

### Test Scenario 2: ROS 2 Topic Bandwidth Test

**Purpose**: Test ROS 2 communication performance

#### 2.1 Using DDS Direct Connection 

```bash
# Ensure ROS_DOMAIN_ID is consistent
export ROS_DOMAIN_ID=0

# Execute test
ros2 launch network_tester network_test.launch.py \
  target:=192.168.0.230 \
  duration:=30 \
  load:=B \
  topics:="/camera/camera/color/image_raw/compressed/robot/joint_states" \
  types:="sensor_msgs/msg/CompressedImage,sensor_msgs/msg/JointState"
```

#### 2.2 Using ROSbridge (Cross-Domain)

**✅ ROSbridge support is now available!**

**Why ROSbridge topics don't appear in `ros2 topic list`**:
- ROSbridge uses **WebSocket (TCP)**, not DDS
- `ros2 topic list` only shows **DDS topics**
- ROSbridge subscriptions are **separate from ROS 2 DDS network**

**Server Side**:
```bash
# Start rosbridge server
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# Verify rosbridge is running
nc -zv <SERVER_IP> 9090
```

**Client Side**:
```bash
# Using launch file (Recommended)
ros2 launch network_tester network_test.launch.py \
  target:=10.0.0.88 \
  duration:=30 \
  load:=B \
  topics:="/camera/camera/color/image_raw/compressed" \
  types:="sensor_msgs/msg/CompressedImage" \
  use_rosbridge:=1 \
  ws_host:=10.0.0.88 \
  ws_port:=9090

# Or use the automated test script
/root/NETWORK/test_rosbridge.sh 10.0.0.88 /camera/camera/color/image_raw/compressed
```

**To verify ROSbridge data transmission** (on server side):

```bash
# Install roslibpy (if not already installed)
pip3 install roslibpy

# Run verification script
cd /root/NETWORK/ros2_ws/src/network_tester/scripts
python3 verify_rosbridge.py \
  --host 127.0.0.1 \
  --port 9090 \
  --topic /camera/camera/color/image_raw/compressed \
  --type sensor_msgs/CompressedImage \
  --duration 30
```

**Expected output if data is being transmitted**:
```
✅ Connected successfully!
📡 Subscribing to /camera/camera/color/image_raw/compressed...
✅ First message received at 14:30:25
📊 Messages: 150 | Rate: 5.0 Hz | Bandwidth: 2.34 Mbps | Elapsed: 30.0s
```

**Output Files**:
- `rosbridge_bw.csv` - ROSbridge bandwidth time series
- `rosbridge_bw_summary.json` - ROSbridge statistics summary
- `chart_rosbridge_bw_*.png` - ROSbridge bandwidth charts

---

### Test Scenario 3: VPN Performance Comparison

**Purpose**: Compare VPN impact on network performance

#### 3.1 No VPN Test
```bash
ros2 launch network_tester network_test.launch.py \
  target:=192.168.0.230 \
  duration:=30 \
  load:=B \
  vpn_mode:=no_vpn
```

#### 3.2 With VPN Test
```bash
# Start VPN connection (e.g., OpenVPN or WireGuard)
# Then run test

ros2 launch network_tester network_test.launch.py \
  target:=10.0.0.88 \
  duration:=30 \
  load:=B \
  vpn_mode:=with_vpn
```

---

## 📊 Output Files Explanation

Each test creates a timestamped directory with the following structure:

```
output/
└── 2025-10-28_14-30-15_no_vpn/
    ├── 📄 meta_2025-10-28_14-30-15.json      # Test summary (Most Important)
    ├── 📁 image/                              # All charts
    │   ├── chart_rtt_*.png                    # RTT latency chart
    │   ├── chart_throughput_*.png             # Throughput chart
    │   ├── chart_mtr_*.png                    # Route latency chart
    │   ├── chart_ros2_bw_*.png                # ROS Topic bandwidth chart
    │   └── rrul_*.png                         # Flent RRUL chart
    ├── 📊 ping_stats_*.csv                    # Ping RTT time series
    ├── 📊 iperf_intervals_*.csv               # iperf throughput time series
    ├── 📊 mtr_hops_*.csv                      # MTR hop-by-hop data
    ├── 📊 ros2_bw.csv                         # ROS Topic bandwidth time series
    ├── 📋 ros2_bw_summary.json                # ROS bandwidth statistics summary
    ├── 🔍 ping_*.txt                          # Ping raw output
    ├── 🔍 iperf_tcp_*.json                    # iperf3 raw JSON
    ├── 🔍 mtr_*.json                          # MTR raw JSON
    └── 🔍 rrul_*.flent.gz                     # Flent raw data
```

### Core Files Explanation

#### 📄 `meta_*.json` - Test Summary (★★★ Most Important)

Contains all key metrics:

```json
{
  "target": "192.168.0.230",
  "duration": 30,
  "vpn_mode": "no_vpn",
  "ping": {
    "rtt_avg_ms": 12.5,
    "rtt_max_ms": 25.3,
    "loss_percent": 0.0,
    "jitter_ms": 2.1,
    "bufferbloat_ms": 12.8
  },
  "iperf": {
    "bandwidth_mbps": 95.2,
    "total_retransmits": 5,
    "lost_percent": 0.01
  },
  "mtr": {
    "total_hops": 3,
    "avg_latency_ms": 15.2
  }
}
```

#### 📊 CSV Files - Detailed Time Series

Can be analyzed with Excel, Python pandas, or other tools:

```bash
# Analyze with pandas
python3 -c "
import pandas as pd
df = pd.read_csv('ping_stats_*.csv')
print(df.describe())
"
```

#### 📁 `image/` Directory - Visualization Charts

All charts are stored in one location for easy viewing and sharing.

---

## 🔍 Results Analysis

### Method 1: Automated Analysis Script (Recommended)

```bash
cd /root/NETWORK/ros2_ws/src/network_tester/scripts

# Analyze single test
python3 analyze_results.py /root/NETWORK/output/2025-10-28_14-30-15_no_vpn

# Batch analyze all tests
python3 analyze_results.py /root/NETWORK/output --batch

# Compare two tests
python3 analyze_results.py \
  /root/NETWORK/output/2025-10-28_14-30-15_no_vpn \
  /root/NETWORK/output/2025-10-28_15-00-20_with_vpn \
  --compare
```

**Example Output**:
```
================================================================================
📊 Network Test Analysis Report
================================================================================
Test Time: 2025-10-28T14:30:15
VPN Mode: no_vpn
--------------------------------------------------------------------------------
🏓 Ping RTT Analysis
--------------------------------------------------------------------------------
Average Latency: 12.50 ms
Maximum Latency: 25.30 ms
Jitter:          2.10 ms
Quality Rating:  🟢 Excellent
--------------------------------------------------------------------------------
🚀 iperf3 Throughput Analysis
--------------------------------------------------------------------------------
Average Throughput: 95.20 Mbps
Retransmissions:    5
Quality Rating:     🟢 Excellent
```

### Method 2: Manual Analysis

#### Quick View Key Metrics

```bash
# Enter test directory
cd /root/NETWORK/output/2025-10-28_14-30-15_no_vpn

# View test summary
cat meta_*.json | jq '{target, vpn_mode, ping, iperf}'

# Calculate average RTT
awk -F',' 'NR>1 {sum+=$2; n++} END {print "Average RTT:", sum/n, "ms"}' ping_stats_*.csv

# View iperf throughput
cat iperf_tcp_*.json | jq '.end.sum_received.bits_per_second / 1000000'

# View charts
ls image/*.png
```

---

## 🎓 Advanced Usage

### Custom Test Parameters

```bash
ros2 launch network_tester network_test.launch.py \
  target:=192.168.0.230 \
  duration:=60 \              # Longer test duration
  load:=C \                   # Heavy load
  bw_interval:=0.5 \          # More frequent bandwidth sampling
  skip_plots:=1 \             # Skip chart generation (faster)
  out:=/custom/output/path    # Custom output directory
```

### Multi-Topic Parallel Monitoring

```bash
ros2 launch network_tester network_test.launch.py \
  target:=192.168.0.230 \
  duration:=30 \
  topics:="/cam1/image,/cam2/image,/lidar/points,/robot/joint_states" \
  types:="sensor_msgs/msg/Image,sensor_msgs/msg/Image,sensor_msgs/msg/PointCloud2,sensor_msgs/msg/JointState"
```

### Batch Testing Script

```bash
#!/bin/bash
# batch_test.sh - Batch test different loads

for load in A B C; do
    echo "Testing with load $load"
    ros2 launch network_tester network_test.launch.py \
      target:=192.168.0.230 \
      duration:=30 \
      load:=$load
    sleep 10
done

# Analyze all results
python3 /root/NETWORK/ros2_ws/src/network_tester/scripts/analyze_results.py \
  /root/NETWORK/output --batch
```

---

## 🔧 Troubleshooting

### Issue 1: `ros2 launch` Cannot Find Package

**Cause**: Environment not sourced

**Solution**:
```bash
cd /root/NETWORK/ros2_ws
source install/setup.bash
ros2 pkg list | grep network_tester
```

---

### Issue 2: iperf3 Connection Failed

**Error Message**: `unable to connect to server: Connection refused`

**Solution**:
```bash
# Verify iperf3 is running on server
ps aux | grep iperf3

# Restart iperf3 server
pkill iperf3
iperf3 -s

# Check firewall
sudo ufw allow 5201/tcp
```

---

### Issue 3: Cannot Subscribe to ROS Topic

**Error Message**: `No messages received on topic`

**Solution**:
```bash
# Ensure ROS_DOMAIN_ID is consistent (DDS only)
echo $ROS_DOMAIN_ID  # Should be same on both sides

# Check if topic exists
ros2 topic list
ros2 topic echo /camera/image_raw --once

# Check network connectivity
ping <target_ip>
```

---

### Issue 4: ROSbridge Shows Bandwidth But No Data on Server

**Problem**: When using `ros2 run network_tester network_rosbridge` with ROSbridge, the client shows bandwidth statistics, but the server at `10.0.0.88` doesn't receive the topic data.

**Root Cause**: 
- ⚠️ **ROSbridge uses WebSocket (TCP), NOT DDS**
- `ROS_DOMAIN_ID` has **NO effect** on ROSbridge communication
- The client is subscribing via WebSocket to the server's rosbridge, but the bandwidth monitor is measuring the WebSocket traffic, not actual topic republishing

**Key Differences**:

| Communication Method | Protocol | ROS_DOMAIN_ID Effect | Network Boundary |
|---------------------|----------|---------------------|------------------|
| **DDS Direct** | UDP multicast | ✅ **Must match** | Same subnet recommended |
| **ROSbridge** | WebSocket (TCP) | ❌ **No effect** | Cross-network OK |

**Solution**:

1. **If using DDS** (Recommended for same network):
```bash
# Server side - ensure ROS_DOMAIN_ID is set
export ROS_DOMAIN_ID=0

# Client side - must match server
export ROS_DOMAIN_ID=0

# Use DDS connection (no rosbridge)
ros2 launch network_tester network_test.launch.py \
  target:=192.168.0.230 \
  duration:=30 \
  topics:="/camera/camera/color/image_raw/compressed" \
  types:="sensor_msgs/msg/CompressedImage"
```

2. **If using ROSbridge** (Cross-network or NAT):
```bash
# Server side (10.0.0.88) - start rosbridge
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# Verify rosbridge is accessible
nc -zv 10.0.0.88 9090  # Should show "succeeded"

# Client side - ROS_DOMAIN_ID doesn't matter
ros2 run network_tester network_rosbridge \
  --ros-args \
  -p target:=10.0.0.88 \
  -p use_rosbridge:=1 \
  -p ws_host:=10.0.0.88 \
  -p ws_port:=9090 \
  -p topics:="/camera/camera/color/image_raw/compressed" \
  -p types:="sensor_msgs/msg/CompressedImage"

# To verify data on server:
# Option A: Use rosbridge client to subscribe
python3 -c "
import roslibpy
ros = roslibpy.Ros(host='127.0.0.1', port=9090)
ros.run()
topic = roslibpy.Topic(ros, '/camera/camera/color/image_raw/compressed', 'sensor_msgs/CompressedImage')
topic.subscribe(lambda m: print(f'Received: {len(m[\"data\"])} bytes'))
ros.run_forever()
"

# Option B: Check if topic exists via ros2 tools (on server)
ros2 topic list
ros2 topic hz /camera/camera/color/image_raw/compressed
```

3. **Understanding the bandwidth data**:
   - With ROSbridge: Bandwidth shows **WebSocket traffic** (client ↔ rosbridge server)
   - With DDS: Bandwidth shows **actual topic data** received by subscriber
   - ROSbridge adds JSON encoding overhead (~20-40% more bandwidth)

---

### Issue 5: VPN Detection Inaccurate

**Solution**: Manually specify VPN mode
```bash
# No VPN
ros2 launch network_tester network_test.launch.py \
  target:=192.168.0.230 \
  vpn_mode:=no_vpn

# With VPN
ros2 launch network_tester network_test.launch.py \
  target:=10.0.0.88 \
  vpn_mode:=with_vpn
```

---

## Project Architecture

### Directory Structure

```
NETWORK/
├── 📁 docker/                    # Docker environment
│   ├── Dockerfile
│   ├── build.sh
│   └── run.sh
├── 📁 ros2_ws/                   # ROS 2 workspace
│   └── src/
│       └── network_tester/       # Main package
│           ├── node_network_tester.py        # Main Node
│           ├── 📁 tools/                        # Tool modules
│           │   ├── vpn_detect.py
│           │   ├── ping_tool.py
│           │   ├── iperf_tool.py
│           │   ├── mtr_tool.py
│           │   ├── flent_tool.py
│           │   ├── ros_bw_monitor.py
│           │   ├── plot_utils.py
│           │   └── io_utils.py
│           ├── 📁 launch/                       # Launch files
│           │   └── network_test.launch.py
│           ├── 📁 scripts/                      # Utility scripts
│           │   ├── run_network_test.py
│           │   └── analyze_results.py
│           └── setup.py
├── 📁 output/                    # Test results output
├── 📁 entrypoint/                # Entry point scripts
│   └── toturial/
│       ├── QUICK_START.md
│       └── VPN_DETECTION_GUIDE.md
└──  README.md                  # Main documentation (Chinese)
```

### Module Description

For detailed architecture, see: [ARCHITECTURE.md](ros2_ws/src/network_tester/ARCHITECTURE.md)

---

## 📚 References

### Related Documentation
- [ROS 2 Humble Official Documentation](https://docs.ros.org/en/humble/)
- [iperf3 User Guide](https://iperf.fr/iperf-doc.php)
- [MTR Network Diagnostic Tool](https://github.com/traviscross/mtr)
- [Flent Network Testing](https://flent.org/)

### Tutorial Documents
- [Quick Start Guide](entrypoint/toturial/QUICK_START.md)
- [VPN Detection Guide](entrypoint/toturial/VPN_DETECTION_GUIDE.md)
- [Modular Architecture Documentation](ros2_ws/src/network_tester/ARCHITECTURE.md)
- [Results Analysis Guide](ros2_ws/src/network_tester/README_MODULAR.md)

### Performance Benchmarks

| Metric | Good | Fair | Poor |
|--------|------|------|------|
| RTT Average | < 30ms | 30-100ms | > 100ms |
| RTT Jitter | < 5ms | 5-20ms | > 20ms |
| Packet Loss | < 0.1% | 0.1-1% | > 1% |
| Throughput | > 80% theoretical | 50-80% | < 50% |
| Retransmissions | < 10 | 10-50 | > 50 |


---

## 📄 License

MIT License

---

## 👨‍💻 Author

**Lester Liou**

- GitHub: [@LesterLiou](https://github.com/LesterLiou)

---

## 🙏 Acknowledgments

Thanks to the following open-source projects:
- [ROS 2](https://ros.org/)
- [iperf3](https://iperf.fr/)
- [MTR](https://github.com/traviscross/mtr)
- [Flent](https://flent.org/)
- [matplotlib](https://matplotlib.org/)

---

**🎯 Quick Links**

- [📖 Quick Start](#-quick-start)
- [📊 Output Files](#-output-files-explanation)
- [🔍 Results Analysis](#-results-analysis)
- [🔧 Troubleshooting](#-troubleshooting)
- [📚 Advanced Tutorial](ros2_ws/src/network_tester/README_MODULAR.md)

---

