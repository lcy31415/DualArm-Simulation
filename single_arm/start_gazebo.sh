#!/bin/bash
# 启动 Gazebo 仿真环境

cd /home/lcy/ur5_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo "启动 Gazebo 仿真环境..."
ros2 launch ur5_gazebo sim.launch.py
