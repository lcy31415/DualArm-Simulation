#!/bin/bash
# 启动运动控制节点

cd /home/lcy/ur5_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo "启动运动控制节点..."
echo "请等待 MoveIt 完全启动后再运行此脚本（约18秒）"
ros2 run ur5_motion_control motion_controller_action
