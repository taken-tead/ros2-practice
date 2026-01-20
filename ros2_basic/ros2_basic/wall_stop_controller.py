#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class WallStopController(Node):
    def __init__(self):
        super().__init__('wall_stop_controller')
        # Publisher（速度指令）
        self.publisher = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            10
        )
        # Subscriber（現在位置）
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )
        # 制御周期（10Hz）
        self.timer = self.create_timer(0.1, self.control_loop)
        self.pose = Pose()
        self.now_rotating = False
    def pose_callback(self, msg):
        self.pose = msg
    def control_loop(self):
        cmd_vel = Twist()
        # 右の壁付近
        if self.pose.x > 9.8 and not self.now_rotating:
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 1.0
            if self.pose.theta < 0:
                self.now_rotating = True
        elif self.pose.x < -9.8 and not self.now_rotating:
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = -1.0
            if self.pose.theta < 0:
                self.now_rotating = True
        # 通常時（前進）
        else:
            cmd_vel.linear.x = 2.0
            cmd_vel.angular.z = 0.0
            self.now_rotating = False
        self.publisher.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    node = WallStopController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

