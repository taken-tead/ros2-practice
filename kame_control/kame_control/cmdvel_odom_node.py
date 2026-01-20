#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
class CmdVelOdom(Node):
    def __init__(self):
        super().__init__('cmdvel_odom_node')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.last_odom = None
    def odom_cb(self, msg: Odometry):
        self.last_odom = msg
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.get_logger().info(f'odom: x={x:.2f}, y={y:.2f}')
    def set_cmd(self, linear_x: float = 0.0, angular_z: float = 0.0):
        cmd = Twist()
        cmd.linear.x = float(linear_x)
        cmd.angular.z = float(angular_z)
        self.pub.publish(cmd)
def spin_for(node: Node, seconds: float):
    start = node.get_clock().now()
    while rclpy.ok():
        elapsed = (node.get_clock().now() - start).nanoseconds * 1e-9
        if elapsed >= seconds:
            break
        rclpy.spin_once(node, timeout_sec=0.1)
def drive_straight(node: CmdVelOdom, speed_mps: float, seconds: float):
    node.get_logger().info(f'Drive straight: v={speed_mps} [m/s], t={seconds} [s]')
    node.set_cmd(linear_x=speed_mps, angular_z=0.0)
    spin_for(node, seconds)
    node.set_cmd(0.0, 0.0)
def rotate(node: CmdVelOdom, yaw_rate_rps: float, seconds: float):
    node.get_logger().info(f'Rotate: w={yaw_rate_rps} [rad/s], t={seconds} [s]')
    node.set_cmd(linear_x=0.0, angular_z=yaw_rate_rps)
    spin_for(node, seconds)
    node.set_cmd(0.0, 0.0)
def main(args=None):
    rclpy.init(args=args)
    node = CmdVelOdom()
    drive_straight(node, speed_mps=1.0, seconds=3.0)
    rotate(node, yaw_rate_rps=1.5, seconds=2.0)
    drive_straight(node, speed_mps=1.0, seconds=1.0)
    rotate(node, yaw_rate_rps=1.5, seconds=2.0)
    drive_straight(node, speed_mps=1.0, seconds=1.0)
    node.get_logger().info('Done. Stop and exit.')
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()

