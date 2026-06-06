#!/bin/bash
WS=/home/lcy/Simulation/dual_arm
cd "$WS"
source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo "启动双臂运动控制器..."
echo "请等待 MoveIt 完全启动后再运行 (约20秒)"
ros2 run ur5_motion_control dual_arm_motion_controller
