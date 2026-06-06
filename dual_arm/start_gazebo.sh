#!/bin/bash
WS=/home/lcy/Simulation/dual_arm
cd "$WS"
source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo "启动双臂 Gazebo 仿真..."
ros2 launch ur5_gazebo sim_dual.launch.py
