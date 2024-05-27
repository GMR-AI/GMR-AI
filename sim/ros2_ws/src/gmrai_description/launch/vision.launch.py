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
    default_instantngp_path = os.path.join(os.path.expanduser('~'), 'instant-ngp/scripts/run.py')
    default_conda_environment = 'instantngp'
    default_images_path = 'data'
    default_ply_path = 'model/gmr'

    # ------- Custom packages -------
    reconstruction_node = Node(
        package='reconstruction3d',
        executable='reconstruction3d',
        name='reconstruction3d',
        parameters=[
            {'instantngp_path': LaunchConfiguration('instantngp_path')},
            {'conda_environment': LaunchConfiguration('conda_environment')},
            {'images_path': LaunchConfiguration('images_path')},
            {'ply_path': LaunchConfiguration('ply_path')},
        ]
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

        DeclareLaunchArgument(name='instantngp_path', default_value=default_instantngp_path,
                              description='Path to the instant-ngp run.py file.'),

        DeclareLaunchArgument(name='conda_environment', default_value=default_conda_environment,
                              description="Name of the conda environment for instant-ngp. Leave blank if there's none."),
        
        DeclareLaunchArgument(name='images_path', default_value=default_images_path,
                              description='Path to the images folder for the reconstruction.'),

        DeclareLaunchArgument(name='ply_path', default_value=default_ply_path,
                              description='Path to the output reconstruction model, without the file extension'),
        
        reconstruction_node,
        detection_node,
    ])