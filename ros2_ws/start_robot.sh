#!/bin/bash

source /opt/ros/humble/setup.bash
source install/setup.bash
$1 $2&
ros2 launch rviz_launcher display.launch.py&