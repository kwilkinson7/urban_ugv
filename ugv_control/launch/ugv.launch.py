from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ugv_control',
            executable='motion_node',
            name='motion_node',
            output='screen',
            remappings=[('/odom_raw', '/odom')]
        )
    ])
