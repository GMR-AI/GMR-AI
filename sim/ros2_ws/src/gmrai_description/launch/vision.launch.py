import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
def generate_launch_description():
    # ------- Variables -------
    pkg_share = FindPackageShare(package='object_detection').find('object_detection')
    default_model_path = os.path.join(pkg_share, 'model/GMRModel.pt')

    # ------- Custom packages -------
    reconstruction_node = Node(
        package='reconstruction3d',
        executable='reconstruction3d',
        name='reconstruction3d',
    )
    detection_node = Node(
        package='object_detection',
        executable='object_detection',
        name='object_detection',
        parameters=[
            {'model_file': LaunchConfiguration('model_file')},
        ]
    )

    return launch.LaunchDescription([
        DeclareLaunchArgument(name='model_file', default_value=default_model_path,
                              description='Path to the YOLOv8 pretrained model file.'),
        reconstruction_node,
        detection_node,
    ])