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

## Development Environment
- ROS 2 Humble
- Ubuntu 22.04
- Python 3.10

## Quick Start

### 1. (Both Server & Client) Clone the repository and build the Docker image 
```bash
git clone https://github.com/LesterLiou/Network.git
cd ~/Network/docker
./build.sh 
```
### 2. (Server) Open server
```bash
cd ~/NETWORK/docker
./run.sh           #get into container
cd ~/NETWORK
./run_server.sh    #turn on server and port
```
### 3.  (Client) Network Test
```bash
cd ~/NETWORK/docker
./run.sh    #get into container


### build ros2 ###

cd ~/NETWORK/ros2_ws
r     #same with colcon build

### network test ###

ros2 run network_tester network_test \
  --ros-args \
  -p target:=<ip> \
  -p duration:=15 \
  -p load:=B \
  -p out:=\"../output\"
```
## Output
```bash
output/time_stamp/
        ├── chart_jitter*.png            
        ├── chart_mtr*.png            
        ├── chart_rtt*.png            
        ├── chart_throughput*.png                     
        ├── iperf_tcp_*.json       # TCP throughput log
        ├── ping_*.txt            # Ping RTT log      
        ├── mtr_*.json            # Multi-hop route trace
        ├── rrul_*.png            # Flent RRUL bufferbloat graph
        ├── rrul_*.flent.gz       # Flent RRUL data
        ├── meta_*.json           # Test config & stats
        └── summary.csv           # Cumulative summary of all runs
```
### Open flent GUI 
```bash
flent --gui rrul-<data_name>.flent.gz
```


## TEST
check the port
```bash
nc -zv -w 3 <SERVER_IP> 9090
```


