import launch
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # ------- Variables -------
    pkg_share = FindPackageShare(package='gmrai_description').find('gmrai_description')

    # ------- Launch Files -------
    transform_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_share, 'launch'),
                                       '/transforms.launch.py']),
        launch_arguments={}.items(),
    )
    visualization_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_share, 'launch'),
                                       '/visualization.launch.py']),
        launch_arguments={}.items(),
    )
    simulation_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_share, 'launch'),
                                       '/coppelia.launch.py']),
        launch_arguments={}.items(),
    )
    
    return launch.LaunchDescription([
        simulation_nodes,
        transform_nodes,
        visualization_nodes
    ])