#!/bin/bash
# UR5 项目启动脚本
# 使用方法：在新的终端中运行此脚本

cd /home/lcy/ur5_ws

# Source 环境
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# 启动系统
ros2 launch ur5_bringup system.launch.py
