from setuptools import find_packages, setup

import os
from glob import glob

package_name = 'custom_transforms'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='adriangt2001',
    maintainer_email='adriangt2001@hotmail.com',
    description='Custom transforms for the odom to base_link transform',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom2baselink = custom_transforms.odom_to_base_link:main',
            'map2odom = custom_transforms.map_to_odom:main', # Uncomment when map_to_odom transform is implemented
        ],
    },
)
