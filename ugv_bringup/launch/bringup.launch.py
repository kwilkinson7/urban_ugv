from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Package paths
    ugv_description = get_package_share_directory('ugv_description')
    lidar = get_package_share_directory('rplidar_ros')
    ugv = get_package_share_directory('ugv_control')
    elrs = get_package_share_directory('elrs_teleop')
    slam_toolbox = get_package_share_directory('slam_toolbox')
    ugv_bringup = get_package_share_directory('ugv_bringup')

    # SLAM parameters
    slam_params = os.path.join(ugv_bringup, 'config', 'slam_toolbox_params.yaml')

    return LaunchDescription([
        # Robot description (URDF + RViz)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ugv_description, 'launch', 'ugv_description.launch.py')
            ),
            launch_arguments={
                'model': os.path.join(ugv_description, 'urdf', 'ugv.urdf'),
                'rvizconfig': os.path.join(ugv_description, 'rviz', 'ugv.rviz'),
                'rviz': 'true'
            }.items()
        ),

        # RPLIDAR A1 node with proper frame_id override
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(lidar, 'launch', 'rplidar_a1_launch.py')
            ),
            launch_arguments={
                'frame_id': 'laser_link'
            }.items()
        ),

        # SLAM Toolbox node with external params and no namespace
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_toolbox, 'launch', 'online_async_launch.py')
            ),
            launch_arguments={
                'use_namespace': 'false',
                'params_file': slam_params
            }.items()
        ),

        # Motion control and odometry node
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ugv, 'launch', 'ugv.launch.py')
            )
        ),

        # ELRS Teleoperation node
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(elrs, 'launch', 'elrs_teleop_launch.py')
            )
        )
    ])
