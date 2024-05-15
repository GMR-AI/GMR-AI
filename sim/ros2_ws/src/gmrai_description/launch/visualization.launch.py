import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import os
def generate_launch_description():
    # ------- Variables -------
    pkg_share = FindPackageShare(package='gmrai_description').find('gmrai_description')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')

    # ------- Pre-built packages -------
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')]
    )

    return launch.LaunchDescription([
        DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                        description='Absolute path to rviz config file'),
        rviz_node
    ])