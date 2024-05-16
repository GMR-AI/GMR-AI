import launch
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml
import os

def generate_launch_description():
    # ------- Variables -------

    # ------- Pre-built packages -------

    # ------- Custom packages -------

    return launch.LaunchDescription([
    ])