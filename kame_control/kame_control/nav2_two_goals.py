#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

def yaw_to_quat(yaw_rad: float):
    qx = 0.0
    qy = 0.0
    qz = math.sin(yaw_rad / 2.0)
    qw = math.cos(yaw_rad / 2.0)
    return qx, qy, qz, qw

class Nav2TwoGoals(Node):
    def __init__(self):
        super().__init__('nav2_two_goals')
        # --- Publisher: RVizの「2D Pose Estimate」と同じ /initialpose —
        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10
        )
        # --- Action Client: RVizの「Nav2 Goal」と同じ /navigate_to_pose —
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

    # ------------------------
    # ① 初期位置を与える
    # ------------------------
    def publish_initial_pose(self, x, y, yaw_rad, frame_id='map'):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)

        qx, qy, qz, qw = yaw_to_quat(yaw_rad)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        # 共分散：位置と向きの許容範囲
        # index: x=0, y=7, yaw=35 (6x6 row-major)
        msg.pose.covariance[0] = 0.25     # x: (0.5m)^2
        msg.pose.covariance[7] = 0.25     # y: (0.5m)^2
        msg.pose.covariance[35] = 0.0685  # yaw: (15deg)^2 ≈ (0.26rad)^2

        self.initialpose_pub.publish(msg)
        self.get_logger().info(f'Initial pose published: x={x}, y={y}, yaw={yaw_rad}')

    # ------------------------
    # ② ゴールを送って結果を待つ
    # ------------------------
    def build_goal(self, x, y, yaw_rad, frame_id='map'):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = frame_id
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)

        qx, qy, qz, qw = yaw_to_quat(yaw_rad)
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw
        return goal

    def send_goal_and_wait(self, x, y, yaw_rad, timeout_sec=180.0):
        # Action server待ち
        self.get_logger().info('Waiting for /navigate_to_pose action server...')
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available. Is Nav2 running?')
            return False

        goal = self.build_goal(x, y, yaw_rad)
        self.get_logger().info(f'Send goal: x={x}, y={y}, yaw={yaw_rad}')

        send_future = self.nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('Goal rejected.')
            return False

        self.get_logger().info('Goal accepted. Waiting for result...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout_sec)

        if not result_future.done():
            self.get_logger().error('Result timeout.')
            return False

        result = result_future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Result: SUCCEEDED')
            return True
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn('Result: CANCELED')
            return False
        else:
            self.get_logger().error(f'Result: FAILED (status={status})')
            return False

def main():
    rclpy.init()
    node = Nav2TwoGoals()

    # 初期位置（RVizの2D Pose Estimate相当）
    INITIAL_POSE = (0.0, 0.0, 0.0)  # (x, y, yaw[rad])

    # ゴール地点（2か所）
    GOALS = [
        (2.5, 0.0, 0.0),     # goal1
        (2.5, 1.0, 1.57),   # goal2
    ]

    # 初期位置は1回だと反映されにくいことがあるので、10回くらい投げる
    for _ in range(10):
        node.publish_initial_pose(*INITIAL_POSE)
        rclpy.spin_once(node, timeout_sec=1.0)

    # 2地点へ順番に移動
    for i, (x, y, yaw) in enumerate(GOALS, start=1):
        ok = node.send_goal_and_wait(x, y, yaw)
        if not ok:
            node.get_logger().error(f'Stop: failed at goal {i}')
            break

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

