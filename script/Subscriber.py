#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy

class Listener(Node):
    def __init__(self):
        super().__init__('simple_listener')
        self.declare_parameter('topic_name', '/chatter')
        topic = self.get_parameter('topic_name').get_parameter_value().string_value
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(String, topic, self.cb, qos)
        self.get_logger().info(f'Subscribed to {topic}')

    def cb(self, msg: String):
        self.get_logger().info(f'recv: {msg.data}')

def main():
    rclpy.init()
    node = Listener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
