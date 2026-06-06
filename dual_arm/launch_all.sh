#!/bin/bash
# ========================================
# 双臂系统总启动脚本
# 用法: ./launch_all.sh
# Gazebo -> MoveIt -> 运动控制器 -> YOLOv8 检测
# ========================================

set -e

WS=/home/lcy/Simulation/dual_arm

echo "========================================"
echo "  双臂仿真系统启动中..."
echo "========================================"

# 终端1: Gazebo
gnome-terminal --title="[1] Gazebo Dual-Arm" -- bash -c "
    source /opt/ros/jazzy/setup.bash
    source $WS/install/setup.bash
    echo '>>> 启动 Gazebo 双臂仿真...'
    ros2 launch ur5_gazebo sim_dual.launch.py
    exec bash"

echo "[1/3] Gazebo 已启动, 等待就绪 (15秒)..."
sleep 15

# 终端2: MoveIt
gnome-terminal --title="[2] MoveIt Dual-Arm" -- bash -c "
    source /opt/ros/jazzy/setup.bash
    source $WS/install/setup.bash
    echo '>>> 启动 MoveIt move_group...'
    ros2 launch ur5_moveit_config move_group_dual.launch.py
    exec bash"

echo "[2/3] MoveIt 已启动, 等待就绪 (5秒)..."
sleep 5

# 终端3: 双臂运动控制器
gnome-terminal --title="[3] Motion Controller" -- bash -c "
    source /opt/ros/jazzy/setup.bash
    source $WS/install/setup.bash
    echo '>>> 启动双臂运动控制器...'
    ros2 run ur5_motion_control dual_arm_motion_controller
    exec bash"

echo "[3/3] 运动控制器已启动, 等待就绪 (3秒)..."
sleep 3

# 终端4: YOLOv8 检测（等待相机就绪）
gnome-terminal --title="[4] YOLOv8 Detection" -- bash -c "
    source /opt/ros/jazzy/setup.bash
    source $WS/install/setup.bash
    echo '等待相机话题就绪...'
    until ros2 topic list 2>/dev/null | grep -qF '/overhead_camera/rgb/image_raw'; do
        sleep 2
    done
    echo '相机就绪，启动检测节点...'
    bash "$WS/run_detect.sh"
    exec bash"

echo ""
echo "========================================"
echo "  ✓ 所有组件启动完成！"
echo "========================================"
echo ""
echo "终端说明:"
echo "  [1] Gazebo 双臂仿真"
echo "  [2] MoveIt move_group"
echo "  [3] 双臂运动控制器"
echo "  [4] YOLOv8 实时检测"
echo ""
