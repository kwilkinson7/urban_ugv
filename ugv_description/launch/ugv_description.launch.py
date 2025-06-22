from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    model_path = PathJoinSubstitution([FindPackageShare('ugv_description'), 'urdf', 'ugv.urdf.xacro'])
    rviz_config_path = PathJoinSubstitution([FindPackageShare('ugv_description'), 'rviz', 'ugv.rviz'])

    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=model_path,
        description='Absolute path to robot URDF or XACRO file'
    )

    rviz_config_arg = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=rviz_config_path,
        description='Absolute path to RViz config file'
    )

    rviz_arg = DeclareLaunchArgument(
        name='rviz',
        default_value='true',
        choices=['true', 'false'],
        description='Launch RViz?'
    )

    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str
    )

    # base_link_tf = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_basefootprint_to_baselink',
    #     arguments=['0', '0', '0.0815', '0', '0', '0', 'base_footprint', 'base_link']
    # )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description},
            {'publish_frequency': 50.0},
            {'use_sim_time': False}  # Make sure you're not waiting for /clock
        ],
        output='screen'
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        # base_link_tf,
        model_arg,
        rviz_config_arg,
        rviz_arg,
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node,
    ])
