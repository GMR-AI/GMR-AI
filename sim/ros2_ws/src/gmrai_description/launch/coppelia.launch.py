import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os

def generate_launch_description():
    # ------- Variables -------
    default_coppelia_root_dir = os.path.join(os.path.expanduser('~'), 'CoppeliaSim_Edu_V4_6_0_rev18_Ubuntu22_04')
    default_coppelia_scene_path = os.path.join(os.path.expanduser('~'), 'GMR-AI', 'sim', 'coppeliasim', 'GMR-Scene', 'GMR.ttt')
    default_coppelia_headless = 'False'

    # ------- Custom packages -------
    coppelia_node = Node(
        package='coppelia_ros2',
        executable='coppelia_ros2',
        name='coppelia_ros2',
        parameters=[
            {'coppelia_root_dir': LaunchConfiguration('coppelia_root_dir')},
            {'coppelia_scene_path': LaunchConfiguration('coppelia_scene_path')},
            {'coppelia_headless': LaunchConfiguration('coppelia_headless')}
        ]
    )

    return launch.LaunchDescription([
        DeclareLaunchArgument(name='coppelia_root_dir', default_value=default_coppelia_root_dir,
                              description='Absolute path to the coppelia root dir.'),
        DeclareLaunchArgument(name='coppelia_scene_path', default_value=default_coppelia_scene_path,
                              description='Absolute path to the coppelia scene.'),
        DeclareLaunchArgument(name='coppelia_headless', default_value=default_coppelia_headless,
                              description='Run coppelia headless or not (dont know what it means yet)'),
        coppelia_node
    ])