#!/bin/bash
# 启动物块分类节点

cd /home/lcy/ur5_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo "启动物块分类节点..."
echo "请等待颜色检测节点启动后再运行此脚本（约24秒）"
ros2 run ur5_perception block_classifier_action
