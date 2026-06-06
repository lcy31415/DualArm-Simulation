#!/bin/bash
WS=/home/lcy/Simulation/dual_arm
cd "$WS"
source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo "启动 MoveIt 双臂规划..."
echo "请等待 Gazebo 完全启动后再运行 (约15秒)"
ros2 launch ur5_moveit_config move_group_dual.launch.py
