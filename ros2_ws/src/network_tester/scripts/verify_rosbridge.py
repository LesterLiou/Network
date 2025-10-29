#!/usr/bin/env python3
"""
ROSbridge Data Verification Script
===================================
用于验证 ROSbridge 是否真的在传输数据

使用方法（在服务器端运行）:
python3 verify_rosbridge.py --host 127.0.0.1 --port 9090 --topic /camera/camera/color/image_raw/compressed

要求:
pip3 install roslibpy
"""

import argparse
import time
import sys

try:
    import roslibpy
except ImportError:
    print("❌ Error: roslibpy not installed")
    print("   Install with: pip3 install roslibpy")
    sys.exit(1)


class ROSbridgeVerifier:
    def __init__(self, host, port, topic, msg_type='sensor_msgs/CompressedImage'):
        self.host = host
        self.port = port
        self.topic_name = topic
        self.msg_type = msg_type
        
        self.message_count = 0
        self.total_bytes = 0
        self.start_time = None
        self.last_print_time = None
        
    def callback(self, message):
        """处理接收到的消息"""
        self.message_count += 1
        
        # 计算数据大小
        if 'data' in message:
            data_size = len(message['data'])
            self.total_bytes += data_size
        else:
            data_size = 0
        
        current_time = time.time()
        
        # 第一次接收消息
        if self.start_time is None:
            self.start_time = current_time
            self.last_print_time = current_time
            print(f"\n✅ First message received at {time.strftime('%H:%M:%S')}")
            print(f"   Data size: {data_size} bytes")
        
        # 每秒打印一次统计
        if current_time - self.last_print_time >= 1.0:
            elapsed = current_time - self.start_time
            rate = self.message_count / elapsed if elapsed > 0 else 0
            bandwidth = (self.total_bytes * 8 / 1e6) / elapsed if elapsed > 0 else 0
            
            print(f"\r📊 Messages: {self.message_count} | "
                  f"Rate: {rate:.1f} Hz | "
                  f"Bandwidth: {bandwidth:.2f} Mbps | "
                  f"Elapsed: {elapsed:.1f}s", end='', flush=True)
            
            self.last_print_time = current_time
    
    def run(self, duration=30):
        """运行验证"""
        print("="*70)
        print("🔍 ROSbridge Data Verification")
        print("="*70)
        print(f"Host:     {self.host}:{self.port}")
        print(f"Topic:    {self.topic_name}")
        print(f"Type:     {self.msg_type}")
        print(f"Duration: {duration}s")
        print("="*70)
        
        # 连接 ROSbridge
        print(f"\n🔌 Connecting to ws://{self.host}:{self.port}...")
        ros = roslibpy.Ros(host=self.host, port=self.port)
        
        try:
            ros.run()
            time.sleep(0.5)  # 等待连接建立
            
            if not ros.is_connected:
                print("❌ Failed to connect to rosbridge")
                print("   Make sure rosbridge server is running:")
                print("   ros2 launch rosbridge_server rosbridge_websocket_launch.xml")
                return False
            
            print("✅ Connected successfully!")
            
            # 订阅 topic
            print(f"\n📡 Subscribing to {self.topic_name}...")
            topic = roslibpy.Topic(ros, self.topic_name, self.msg_type)
            topic.subscribe(self.callback)
            
            print(f"⏳ Waiting for messages (max {duration}s)...")
            print("   Press Ctrl+C to stop\n")
            
            # 等待指定时间
            deadline = time.time() + duration
            try:
                while time.time() < deadline and ros.is_connected:
                    time.sleep(0.1)
            except KeyboardInterrupt:
                print("\n\n⚠️  Interrupted by user")
            
            # 取消订阅
            topic.unsubscribe()
            
            # 打印最终统计
            print("\n\n" + "="*70)
            print("📊 Final Statistics")
            print("="*70)
            
            if self.message_count == 0:
                print("❌ No messages received!")
                print("\n🔍 Troubleshooting:")
                print("   1. Check if topic exists on server:")
                print(f"      ros2 topic list | grep {self.topic_name}")
                print("   2. Check if publisher is running")
                print("   3. Verify topic name and message type")
                return False
            else:
                elapsed = time.time() - self.start_time
                avg_rate = self.message_count / elapsed
                avg_bandwidth = (self.total_bytes * 8 / 1e6) / elapsed
                
                print(f"✅ Total messages:  {self.message_count}")
                print(f"✅ Total bytes:     {self.total_bytes:,} ({self.total_bytes/1e6:.2f} MB)")
                print(f"✅ Average rate:    {avg_rate:.2f} Hz")
                print(f"✅ Average bandwidth: {avg_bandwidth:.2f} Mbps")
                print(f"✅ Duration:        {elapsed:.2f}s")
                print("="*70)
                return True
        
        except Exception as e:
            print(f"\n❌ Error: {e}")
            import traceback
            traceback.print_exc()
            return False
        
        finally:
            if ros.is_connected:
                ros.terminate()


def main():
    parser = argparse.ArgumentParser(
        description='Verify ROSbridge data transmission',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Verify compressed image topic
  python3 verify_rosbridge.py \\
    --host 127.0.0.1 \\
    --port 9090 \\
    --topic /camera/camera/color/image_raw/compressed \\
    --type sensor_msgs/CompressedImage
  
  # Verify joint states
  python3 verify_rosbridge.py \\
    --host 127.0.0.1 \\
    --port 9090 \\
    --topic /robot/joint_states \\
    --type sensor_msgs/JointState
        """
    )
    
    parser.add_argument('--host', default='127.0.0.1',
                       help='ROSbridge host (default: 127.0.0.1)')
    parser.add_argument('--port', type=int, default=9090,
                       help='ROSbridge port (default: 9090)')
    parser.add_argument('--topic', required=True,
                       help='Topic name to subscribe')
    parser.add_argument('--type', dest='msg_type', 
                       default='sensor_msgs/CompressedImage',
                       help='Message type (default: sensor_msgs/CompressedImage)')
    parser.add_argument('--duration', type=int, default=30,
                       help='Test duration in seconds (default: 30)')
    
    args = parser.parse_args()
    
    verifier = ROSbridgeVerifier(
        args.host, 
        args.port, 
        args.topic, 
        args.msg_type
    )
    
    success = verifier.run(args.duration)
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
