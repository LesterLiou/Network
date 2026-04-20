# Network Tester Module Documentation

## 1. Project Structure

```text
network_tester/
├── launch/
│   └── network_test.launch.py           # ROS 2 Launch file
├── network_tester/
│   ├── node_network_tester.py           # Main Lifecycle Node
│   └── tools/                           # Pluggable testing modules
│       ├── vpn_detect.py                # VPN route analysis
│       ├── io_utils.py                  # File I/O & system metrics
│       ├── ping_tool.py                 # RTT & Jitter analysis
│       ├── iperf_tool.py                # Bandwidth & Retransmissions
│       ├── mtr_tool.py                  # Hop-by-hop latency routing
│       ├── flent_tool.py                # Bufferbloat (RRUL) testing
│       ├── ros_bw_monitor.py            # ROS topic message/rate monitor
│       └── plot_utils.py                # Graph generation (matplotlib)
└── scripts/
    ├── run_network_test.py              # CLI entry point
    └── analyze_results.py               # Post-processing script
```

## 2. Usage

### 2.1 ROS 2 Launch (Recommended)
```bash
ros2 launch network_tester network_test.launch.py \
  target:=192.168.0.230 \
  duration:=30 \
  load:=B \
  vpn_mode:=auto \
  topics:="/camera/image_raw" \
  types:="sensor_msgs/msg/Image"
```

### 2.2 Standalone CLI
```bash
cd /root/NETWORK/ros2_ws/src/network_tester/scripts
./run_network_test.py --target 192.168.0.230 --duration 30 --load B
```

## 3. Configuration Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `target` | `--` | Test target IP address |
| `duration` | `30` | Test duration in seconds |
| `load` | `B` | Target traffic load: A (unlimited), B (2 Mbps), C (24 Mbps) |
| `out` | `/root/NETWORK/output` | Data output directory |
| `vpn_mode` | `auto` | Routing mode (`no_vpn`, `with_vpn`, `auto`) |
| `topics` | `""` | Comma-separated ROS topic names |
| `types` | `""` | Comma-separated ROS message types |
| `skip_plots`| `0` | Disables rendering graphs if set to `1` |

## 4. Output Artifacts

Tests generate a timestamped output directory (e.g., `2025-10-28_14-30-15_no_vpn/`) containing:

- **Raw Data**: `ping_*.txt`, `iperf_tcp_*.json`, `mtr_*.json`
- **Processed Timeseries**: `ping_stats_*.csv`, `iperf_intervals_*.csv`, `mtr_hops_*.csv`, `ros2_bw.csv`
- **Metadata**: `meta_*.json` (Aggregated summary of all tests)
- **Plots**: `chart_rtt_*.png`, `chart_throughput_*.png`, etc.

## 5. Build Instructions

```bash
cd /root/NETWORK/ros2_ws
colcon build --packages-select network_tester
source install/setup.bash
```
