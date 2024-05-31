import launch
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # ------- Variables -------
    pkg_share = FindPackageShare(package='gmrai_description').find('gmrai_description')

    # ------- Launch Files -------
    simulation_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_share, 'launch'),
                                       '/coppelia.launch.py']),
        launch_arguments={}.items(),
    )
    robot_manager_node = Node(
        package='gmrai_description',
        executable='robot_manager',
        name='robot_manager',
    )
    vision_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_share, 'launch'),
                                       '/vision.launch.py']),
        launch_arguments={}.items(),
    )

    return launch.LaunchDescription([
        robot_manager_node,
        simulation_nodes,
        vision_nodes
    ])