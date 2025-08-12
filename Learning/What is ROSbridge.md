# What is ROSbridge & How to use
ROSbridge is a middleware that enables communication between the ROS 2 ecosystem and non-ROS clients (such as browsers, Unity, or Python scripts) using WebSocket or TCP connections.
It acts as a bridge between the ROS message-passing system and external applications, providing:

- JSON-based message exchange
- ROS topic publishing & subscribing
- ROS service calls
- ROS parameter access

## 1. Between DDS & ROSbridge
| Aspect                     | Native DDS (Same `ROS_DOMAIN_ID`)                             | ROSbridge (WebSocket/JSON)                                                      |
| -------------------------- | ------------------------------------------------------------- | ------------------------------------------------------------------------------- |
| **Role**                   | Default transport and discovery for ROS 2                     | Bridges non-ROS clients or cross-network connections via WebSocket              |
| **Security Boundary**      | **`ROS_DOMAIN_ID` is *not* a security boundary**              | Defined by TCP connection boundary (can add TLS and credentials)                |
| **Subnet / NAT Traversal** | Poor (relies on multicast/UDP ports 7400+)                    | Good (single TCP port, firewall-friendly)                                       |
| **Latency / Throughput**   | ★★★ (best performance)                                        | ★ (JSON/WS encoding adds overhead, lower throughput)                            |
| **Tools / Compatibility**  | Fully compatible with `ros2 *` tools and complete QoS control | Requires `rosbridge_server`; QoS concepts simplified                            |
| **Operational Complexity** | Low                                                           | Low–Medium (adds a bridging service to maintain)                                |
| **Typical Use Case**       | Same LAN / same datacenter, pure ROS 2 communications         | Cross-network, browser-based clients, or non-ROS integrations                   |


## SETTING

- Default port: 9090
- Protocol: WebSocket
- JSON message format
