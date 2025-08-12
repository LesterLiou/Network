# Network Test 


## 📁  Project Structure

```bash
.
├── docker/                     # Dockerfile and run script
├── script/                      # Project script
├── output/ 
├── README.md
└── ros2_ws/
    └── src/            
        └── network_tester/
            ├── network_tester_image.py
            ├── network_tester_rosbridge.py
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
./run.sh           # get into container {bash|rosbridge}
# ./run.sh rosbridge      # Open port 9090 (rosbridge)
## Check the {--ROS_DOMAIN_ID 0}

cd ~/NETWORK
./run_server.sh    # turn on server and port
```
### 3.  (Client) Network Test
```bash
cd ~/NETWORK/docker
./run.sh    # get into container


### build ros2 ###

cd ~/NETWORK/ros2_ws
r     #same with colcon build


### network test (only Thoughput)###

ros2 run network_tester network_test \
  --ros-args \
  -p target:=<ip> \
  -p duration:=15 \
  -p load:=B \
  -p out:=\"../output\"


### network test (with ROSbridge JoinState)###

ros2 run network_tester network_rosbridge \
  --ros-args \
  -p target:=<ip> \
  -p duration:=15 \
  -p load:=B \
  -p out:="/root/NETWORK/output" \
  -p topics:="/left_follower/joint_states" \
  -p types:="sensor_msgs/msg/JointState" \
  -p bw_interval:=1 \
  -p use_rosbridge:=1 \
  -p ws_host:=<ip> \
  -p ws_port:=9090

```
## Output
```bash
output/time_stamp/
        ├── chart_jitter*.png            
        ├── chart_mtr*.png            
        ├── chart_rtt*.png            
        ├── chart_throughput*.png 
        ├── bw__*.png             # (Optional)   
        ├── iperf_tcp_*.json      # TCP throughput log
        ├── ping_*.txt            # Ping RTT log      
        ├── mtr_*.json            # Multi-hop route trace
        ├── rrul_*.png            # Flent RRUL bufferbloat graph
        ├── rrul_*.flent.gz       # Flent RRUL data
        ├── meta_*.json           # Test config & stats
        ├── ros2_bw.csv           # (Optional) ros2_bw (Mbps)  
        └── summary.csv           # Cumulative summary of all runs
```
### Open flent GUI 
```bash
flent --gui rrul-<data_name>.flent.gz
```


## TEST
### Client : check the port
```bash
nc -zv -w 3 <SERVER_IP> 9090
```
## With Joint Command (ROS_DOMAIN_ID / ROSbridge)

```bash
### DDS（ROS_DOMAIN_ID / DDS ） ###

ros2 run network_tester network_rosbridge \
  --ros-args \
  -p target:=<ip> \
  -p duration:=15 \
  -p load:=B \
  -p out:="/root/NETWORK/output" \
  -p topics:="/left_follower/joint_states" \
  -p types:="sensor_msgs/msg/JointState" \
  -p bw_interval:=1

### network test (with ROSbridge JoinState) ###

ros2 run network_tester network_rosbridge \
  --ros-args \
  -p target:=<ip> \
  -p duration:=15 \
  -p load:=B \
  -p out:="/root/NETWORK/output" \
  -p topics:="/left_follower/joint_states" \
  -p types:="sensor_msgs/msg/JointState" \
  -p bw_interval:=1 \
  -p use_rosbridge:=1 \
  -p ws_host:=<ip> \
  -p ws_port:=9090
```

