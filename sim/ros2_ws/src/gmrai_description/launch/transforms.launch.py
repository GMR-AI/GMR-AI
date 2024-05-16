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
            {'robot_description': Command(['xacro ', LaunchConfiguration('model')])}
        ]
    )
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[
            {'robot_description': Command(['xacro ', LaunchConfiguration('model')])}
        ]
    )
    map_to_odom_static_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--frame-id', 'world', '--child-frame-id', 'odom']
    )

    # ------- Custom packages -------
    odom_to_base_link_frame_publisher_node = Node(
        package='custom_transforms',
        executable='odom_to_base_link',
        name='odom_to_base_link'
    )

    return launch.LaunchDescription([
        DeclareLaunchArgument(name='model', default_value=default_model_path,
                              description='Absolute path to robot urdf file'),
        robot_state_publisher_node,
        joint_state_publisher_node,
        map_to_odom_static_publisher_node,
        odom_to_base_link_frame_publisher_node
    ])