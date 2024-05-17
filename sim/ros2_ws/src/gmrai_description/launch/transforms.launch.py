import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import os

def generate_launch_description():
    # ------- Variables -------
    pkg_share = FindPackageShare(package='gmrai_description').find('gmrai_description')
    default_model_path = os.path.join(pkg_share, 'urdf/GMR-AI.urdf')

    # ------- Pre-built packages -------
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': Command(['xacro ', LaunchConfiguration('model')])},
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[
            {'robot_description': Command(['xacro ', LaunchConfiguration('model')])},
            {'source_list': ['gmr/joint_state']},
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )
    map_to_odom_static_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--frame-id', 'map', '--child-frame-id', 'odom']
    )
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(pkg_share, 'config/ekf.yaml'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    return launch.LaunchDescription([
        DeclareLaunchArgument(name='model', default_value=default_model_path,
                              description='Absolute path to robot urdf file'),
        DeclareLaunchArgument(name='use_sim_time', default_value='True',
                              description='Flag to enable use_sim_time'),
        robot_state_publisher_node,
        joint_state_publisher_node,
        map_to_odom_static_publisher_node,
        robot_localization_node,
    ])