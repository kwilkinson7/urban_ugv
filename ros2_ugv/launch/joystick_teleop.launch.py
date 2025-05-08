from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('ros2_ugv'),
        'config',
        'ps4.config.yaml'
    )

    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{'dev': '/dev/input/js0'}]
        ),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy',
            parameters=[config],
            remappings=[('/cmd_vel', '/cmd_vel')]
        ),
        Node(
            package='ros2_ugv',
            executable='motion_node',
            name='motion_node'
        )
    ])
