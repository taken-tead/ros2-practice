#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class HelloNode(Node):
    def __init__(self):
        super().__init__('hello_node')
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info("Hello World!")
        self.get_logger().info("num: %s" % 1)
        self.get_logger().warn("num: %s" % 2)
        self.get_logger().error("num: %s" % 3)
        self.get_logger().fatal("num: %s" % 4)

def main(args=None):
    rclpy.init(args=args)
    node = HelloNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

