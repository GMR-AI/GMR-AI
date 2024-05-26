from setuptools import find_packages, setup

import os
from glob import glob

package_name = 'object_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'model'), glob('object_detection/model/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='adriangt2001',
    maintainer_email='adriangt2001@hotmail.com',
    description='Object detection package for GMR-AI',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_detection = object_detection.object_detection:main',
        ],
    },
)
