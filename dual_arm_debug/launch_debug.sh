#!/bin/bash
# 双臂调试工作空间启动脚本
# 启动 Gazebo + 手动关节滑块 GUI (依赖 dual_arm 中的 ur5_gazebo)

WS=/home/lcy/Simulation/dual_arm_debug
DUAL_ARM_WS=/home/lcy/Simulation/dual_arm
cd "$WS"

source /opt/ros/jazzy/setup.bash
source "$DUAL_ARM_WS/install/setup.bash"
source install/setup.bash

echo "启动双臂调试环境 (手动关节滑块)..."
ros2 launch dual_arm_debug debug_dual.launch.py
