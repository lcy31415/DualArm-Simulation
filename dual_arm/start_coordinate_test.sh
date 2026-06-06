#!/bin/bash
WS=/home/lcy/Simulation/dual_arm
cd "$WS"
source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo "启动坐标测试 GUI..."
ros2 run coordinate_test coordinate_test_gui
