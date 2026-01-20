from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim_node',
            output='screen',
        ),
        Node(
            package='ros2_basic',
            executable='cmd_vel_publisher',
            name='cmd_vel_publisher',
            output='screen',
        ),
        Node(
            package='ros2_basic',
            executable='pose_subscriber',
            name='pose_subscriber',
            output='screen',
        ),
    ])

