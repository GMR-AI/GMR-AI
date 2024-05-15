from setuptools import find_packages, setup

package_name = 'coppelia_ros2'

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
    description='Package to run Coppelia in a launch file from ROS2.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'coppelia_ros2 = coppelia_ros2.run_coppeliasim:main',
        ],
    },
)
