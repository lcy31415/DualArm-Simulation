#!/bin/bash
# 启动 RViz

cd /home/lcy/ur5_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo "启动 RViz..."
rviz2 -d src/ur5_project/ur5_gazebo/rviz/ur5_config.rviz
