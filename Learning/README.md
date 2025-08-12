# Difference between PING, TCP, and UDP

This document provides a basic overview of three essential network protocols/tools used for communication and diagnostics: **TCP**, **UDP**, and **PING**.

---
## 1. PING (Packet Internet Groper)

- **Used to test network connectivity and latency (RTT)**.
- **Based on ICMP (Internet Control Message Protocol)** to send echo requests.
- **Verifies host availability**: Confirms if a specific IP or domain is reachable.
- **Reports round-trip time (RTT)** and **packet loss percentage**.
- **Common use cases**: VPN testing, whitelist validation, internal/external network checks.

---

## 2. TCP (Transmission Control Protocol)

- **High reliability**: Detects lost or corrupted packets and automatically retransmits them.
- **Connection-oriented**: Requires a **three-way handshake** to establish a connection before data transfer.
- **Ordered delivery**: Ensures data is received in the exact order it was sent.
- **Slower speed**: Overhead from connection setup, acknowledgments, and retransmissions affects performance.

---

## 3. UDP (User Datagram Protocol)

- **Connectionless and unreliable**: Does not ensure the recipient receives the data.
- **No packet ordering**: Packets may arrive out of sequence.
- **Fast and low latency**: Lightweight due to lack of error checking and retransmission.

---

## 4. How to Choose
| Tool         | Focus                  | Key Interpretation                                        |
| ------------ | ---------------------- | --------------------------------------------------------- |
| `ping`       | Latency/Stability      | Look at RTT average and packet loss                       |
| `iperf3 TCP` | Throughput             | Stable and high bitrate is preferred                      |
| `iperf3 UDP` | Packet rate/volatility | Check if desired bitrate is met and consistency over time |

---

## Next Reading

### [How to Use the Basic Tool](./HOW%20TO%20USE%20BASIC%20TOOL.md)
### [What is ROSbridge](./What%20is%20ROSbridge.md)




