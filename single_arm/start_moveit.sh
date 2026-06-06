#!/bin/bash
# 启动 MoveIt

cd /home/lcy/ur5_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo "启动 MoveIt..."
echo "请等待 Gazebo 完全启动后再运行此脚本（约10秒）"
ros2 launch ur5_moveit_config move_group.launch.py
