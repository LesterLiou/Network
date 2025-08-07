# HOW TO USE The Network test BASIC TOOL

This guide explains how to use two essential tools for basic network diagnostics: **PING** and **iperf3**.

## 0. Installation (Already included in Docker environment)
```bash
sudo apt update
sudo apt install iputils-ping iperf3 -y
```

## 1. PING
### 1)  Basic Usage
```bash
ping <target_ip_or_hostname>
```
### 2)  Parameters
```bash
-c <coumt>         # package_num
-i <seconds>       # Interval between each request
-s <size>          # package size (bytes)
```
### 3)      Example
```bash
ping 8.8.8.8         # ping Google DNS
ping google.com      # ping Google website
ping -c 10 -i 0.5 -s 64 8.8.8.8
# Send 10 packets, 0.5s apart, each with 64 bytes payload
```

## 2. iperf3
Measure Network Throughput (TCP/UDP)
### Step 1:  Run Sever
```bash
iperf3 -s
```
### Step 2:  Run Client
```bash
iperf3 -c <Server-IP>
```
###  # Parameters
```bash
-t <seconds>        # Duration of the test (default: 10)
-n, 	            # Send a fixed number of packets (overrides -t）。
-u                  # Use UDP instead of TCP
-p <port>           # Reverse mode: the server sends data, and the client receives
-b, --bandwidth
# For UDP: Set the constant bit rate to test a specific bandwidth
# For TCP: Limit the rate of each stream, which can be used to simulate a target throughput
```
### # Example 
####  1. TCP
```bash
iperf3 -c 192.168.0.230 -t 15
# Run a 15-second TCP throughput test to the target server
```
#### 2. UDP
```bash
iperf3 -c 192.168.0.230 -u -b 10M -t 15
# Run a 15-second UDP test at 10 Mbps to the target server
```

## 3. OUTPUT

### (1) PING
- Example
```lua
64 bytes from 192.168.0.230: icmp_seq=1 ttl=64 time=0.195 ms
...
--- 192.168.0.230 ping statistics ---
10 packets transmitted, 10 received, 0% packet loss, time 9210ms
rtt min/avg/max/mdev = 0.187/0.201/0.231/0.015 ms
```
- Explanation
```bash
-icmp_seq       # Packet sequence number
-ttl            # (Time To Live) limits how many hops a packet can take
-time           # Round Trip time (RTT)
-packet loss    # Percentage of lost packets during the test
-rtt min/avg/max/mdev	# Minimum, average, maximum latency and jitter (mdev = mean deviation)
```

### (2) iperf3 TCP
- Example

```css
[ ID] Interval           Transfer     Bitrate         Retr  Cwnd
[  5]   0.00-1.00   sec  17.4 MBytes   146 Mbits/sec    0   1.20 MBytes       
[  5]   1.00-2.00   sec  20.0 MBytes   168 Mbits/sec    0   2.03 MBytes       
[  5]   2.00-3.00   sec  21.2 MBytes   178 Mbits/sec   17   2.03 MBytes  
```
- Explanation
```bash
-Interval           # Time interval of measurement
-Transfer           # Amount of data transferred
-Bitrate            # Transfer speed (Mbits/sec)
-Retr               # Retransmissions (TCP only)
-Cwnd               # Congestion Window (TCP only)	shows how much data TCP is allowed to send, in Bytes/KBytes. Higher is better.
```

### (3) iperf3 UDP
- Example
```css
[ ID] Interval           Transfer     Bitrate         Total Datagrams
[  5]   0.00-1.00   sec  12.5 MBytes   105 Mbits/sec  9030  
[  5]   1.00-2.00   sec  7.97 MBytes  66.8 Mbits/sec  5770  
```
- Explanation
```bash
-Interval           # Time interval of measurement
-Transfer           # Amount of data transferred
-Bitrate            # Bandwidth per second (may vary due to UDP characteristics)
-Total Datagrams    # Number of packets sent in the interval
```
## 4. How to Choose
| Tool         | Focus                  | Key Interpretation                                        |
| ------------ | ---------------------- | --------------------------------------------------------- |
| `ping`       | Latency/Stability      | Look at RTT average and packet loss                       |
| `iperf3 TCP` | Throughput             | Stable and high bitrate is preferred                      |
| `iperf3 UDP` | Packet rate/volatility | Check if desired bitrate is met and consistency over time |



