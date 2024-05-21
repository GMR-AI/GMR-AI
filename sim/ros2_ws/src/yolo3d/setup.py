from setuptools import find_packages, setup

import os
from glob import glob

package_name = 'yolo3d'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/'+package_name+'/utils', glob('yolo3d/utils/*.py')),
        ('lib/'+package_name+'/models', glob('yolo3d/models/*.[py][ya]*')),
        ('lib/'+package_name+'/script', glob('yolo3d/script/*.[pt][yx]*')),
        ('lib/'+package_name+'/library', glob('yolo3d/library/.py')),
        ('lib/'+package_name+'/data', glob('yolo3d/data/*.yaml')),
        ('lib/'+package_name+'/eval/camera_cal', ['yolo3d/eval/camera_cal/calib_cam_to_cam.txt']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='adriangt2001',
    maintainer_email='adriangt2001@hotmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'inference_ros = yolo3d.inference_ros:main',
        ],
    },
)
