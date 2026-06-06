#!/bin/bash
# 启动颜色检测节点

cd /home/lcy/ur5_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo "启动颜色检测节点..."
echo "请等待运动控制节点启动后再运行此脚本（约21秒）"
ros2 run ur5_perception color_detector_handshake --ros-args -p use_sim_time:=True
