# Network Test 


## 📁  Project Structure

```bash
.
├── docker/                     # Dockerfile and run script
├── script/                      # Project script
├── README.md
└── ros2_ws/
    └── src/            
        └── network_tester/
            └── network_tester_node.py  
```

## ⚙️ Development Environment
- ROS 2 Humble
- Ubuntu 22.04
- Python 3.10

## Quick Start

### 1. (Both Server & Client) Clone the repository and build the Docker image 
```
git clone https://github.com/LesterLiou/Network.git
cd Network/docker
./build.sh 
```
### 2. (Server) Open server
```
cd Network/docker
./run.sh    #get into container
./run_server.sh    #turn on server and port
```
### 3.  (Client) Network Test
```
cd Network/docker
./run.sh    #get into container

### build ros2 ###
cd ~/Network/ros2_ws
r     #same with colcon build

### network test ###

ros2 run network_tester network_tester \
  --ros-args \
    -p target:= <ip> \
    -p duration:=15 \ 
    -p udp:=True \
    -p load:=B \
    -p out:="../output"
```
### 4. Output
```
output/time_stamp/
        ├── ping_*.txt            # Ping RTT log
        ├── iperf_tcp_*.txt       # TCP throughput log
        ├── iperf_udp_*.txt       # UDP throughput log (if --udp)
        ├── mtr_*.json            # Multi-hop route trace
        ├── summary_plot.png      # RTT over time
        ├── rrul_*.png            # Flent RRUL bufferbloat graph
        ├── meta_*.json           # Test config & stats
        └── summary.csv           # Cumulative summary of all runs
```


