#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy

class Talker(Node):
    def __init__(self):
        super().__init__('simple_talker')
        self.declare_parameter('topic_name', '/chatter')
        self.declare_parameter('freq', 2.0)  # Hz
        topic = self.get_parameter('topic_name').get_parameter_value().string_value
        freq  = float(self.get_parameter('freq').get_parameter_value().double_value or 2.0)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub = self.create_publisher(String, topic, qos)
        self.i = 0
        self.timer = self.create_timer(1.0 / freq, self.tick)
        self.get_logger().info(f'Publishing to {topic} @ {freq} Hz')

    def tick(self):
        msg = String()
        msg.data = f'hello #{self.i}'
        self.pub.publish(msg)
        self.i += 1

def main():
    rclpy.init()
    node = Talker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
