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
    param_file = LaunchConfiguration('params_file', default=os.path.join(pkg_share, 'config', 'nav2_params.yaml'))

    # ------- Launch Files -------
    simulation_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_share, 'launch'),
                                       '/coppelia.launch.py']),
        launch_arguments={}.items(),
    )
    transform_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_share, 'launch'),
                                       '/transforms.launch.py']),
        launch_arguments={}.items(),
    )
    robot_manager_node = Node(
        package='gmrai_description',
        executable='robot_manager',
        name='robot_manager',
    )
    nav2_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_share, 'launch'),
                                       '/navigation_launch.py']),
        launch_arguments={'params_file': param_file,
                          'use_sim_time': 'True'}.items()
    )
    vision_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_share, 'launch'),
                                       '/vision.launch.py']),
        launch_arguments={}.items(),
    )
    visualization_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_share, 'launch'),
                                       '/visualization.launch.py']),
        launch_arguments={}.items(),
    )

    return launch.LaunchDescription([
        simulation_nodes,
        transform_nodes,
        robot_manager_node,
        nav2_nodes,
        vision_nodes,
        visualization_nodes,
    ])