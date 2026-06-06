#!/bin/bash
# ========================================
# UR5 仿真系统总启动脚本
# 自动启动: Gazebo + RViz + MoveIt + Motion Control
# ========================================

set -e  # 遇到错误立即退出

echo "========================================"
echo "  UR5 仿真系统启动中..."
echo "========================================"

# 进入工作空间
cd "$(dirname "$0")"

# Source 环境
echo "[1/6] 加载 ROS 2 环境..."
source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo "[2/6] 启动 Gazebo 仿真环境..."
gnome-terminal --tab --title="Gazebo Simulation" -- bash -c "
    source /opt/ros/jazzy/setup.bash;
    source install/setup.bash;
    ros2 launch ur5_gazebo sim.launch.py;
    exec bash"

# 等待 Gazebo 启动
echo "等待 Gazebo 启动 (10秒)..."
sleep 10

echo "[3/6] 启动 MoveIt 和 RViz..."
gnome-terminal --tab --title="MoveIt + RViz" -- bash -c "
    source /opt/ros/jazzy/setup.bash;
    source install/setup.bash;
    ros2 launch ur5_moveit_config move_group.launch.py &
    sleep 5;
    rviz2 -d src/ur5_project/ur5_gazebo/rviz/ur5_config.rviz;
    exec bash"

# 等待 MoveIt 启动
echo "等待 MoveIt 启动 (8秒)..."
sleep 8

echo "[4/6] 启动运动控制节点(Action Server)..."
gnome-terminal --tab --title="Motion Control" -- bash -c "
    source /opt/ros/jazzy/setup.bash;
    source install/setup.bash;
    ros2 run ur5_motion_control motion_controller_action;
    exec bash"

echo "[5/6] 启动颜色检测节点(握手通讯模式)..."
gnome-terminal --tab --title="Color Detector" -- bash -c "
    source /opt/ros/jazzy/setup.bash;
    source install/setup.bash;
    ros2 run ur5_perception color_detector_handshake --ros-args -p use_sim_time:=True;
    exec bash"

# 等待检测节点启动
echo "等待检测节点启动 (3秒)..."
sleep 3

echo "[6/6] 启动物块分类节点(Action Client)..."
gnome-terminal --tab --title="Block Classifier" -- bash -c "
    source /opt/ros/jazzy/setup.bash;
    source install/setup.bash;
    ros2 run ur5_perception block_classifier_action;
    exec bash"

echo ""
echo "========================================"
echo "  ✓ 所有组件启动完成！"
echo "========================================"
echo ""
echo "终端说明:"
echo "  - Tab 1: Gazebo 仿真环境"
echo "  - Tab 2: MoveIt + RViz 可视化"
echo "  - Tab 3: 运动控制节点(Action Server)"
echo "  - Tab 4: 颜色检测节点(握手通讯)"
echo "  - Tab 5: 物块分类节点(Action Client)"
echo ""
echo "Action 通讯系统功能:"
echo "  - 自动检测物块并等待稳定"
echo "  - 批量发送检测数据"
echo "  - 使用 Action 等待运动完成反馈"
echo "  - 无需 time.sleep，精确同步"
echo ""
echo "测试命令 (在新终端运行):"
echo "  ros2 topic echo /detection_batch  # 查看检测批量数据"
echo ""
echo "关闭所有节点: Ctrl+C (在各个终端)"
echo "========================================"
