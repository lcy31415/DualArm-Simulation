#!/bin/bash
WS=/home/lcy/Simulation/dual_arm
cd "$WS"
source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo "启动双臂分时物块分拣 Demo..."
echo "请等待 Gazebo + MoveIt + 运动控制器 全部就绪后再运行"
ros2 run ur5_motion_control dual_arm_sorting_demo
