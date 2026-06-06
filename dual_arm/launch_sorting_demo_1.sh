#!/bin/bash
# ========================================
# 分拣演示 Demo 1 启动脚本
# 用法: ./launch_sorting_demo_1.sh
#
# 启动顺序:
#   [1] Gazebo 双臂仿真
#   [2] MoveIt move_group
#   [3] 双臂运动控制器
#   [4] YOLOv8 检测节点（--auto-start false）
#   [5] sorting_demo_1（双臂归位 → 触发检测）
# ========================================

set -e

WS=/home/lcy/Simulation/dual_arm
PYTHON="$WS/.venv/bin/python3"
DETECT_SCRIPT="$WS/src/ur5_project/ur5_perception/ur5_perception/detect_node.py"
MODEL="$WS/models/best.pt"

echo "========================================"
echo "  双臂分拣 Demo 1 启动中..."
echo "========================================"

# ── 检查 venv ──
if [ ! -f "$PYTHON" ]; then
    echo "错误：未找到虚拟环境，请先运行："
    echo "  bash $WS/setup_perception_env.sh"
    exit 1
fi

# ── 检查检测模型 ──
if [ ! -f "$MODEL" ]; then
    echo "错误：未找到 YOLOv8 模型权重: $MODEL"
    exit 1
fi

# 终端1: Gazebo
gnome-terminal --title="[1] Gazebo Dual-Arm" -- bash -c "
    source /opt/ros/jazzy/setup.bash
    source $WS/install/setup.bash
    echo '>>> 启动 Gazebo 双臂仿真...'
    ros2 launch ur5_gazebo sim_dual.launch.py
    exec bash"

echo "[1/4] Gazebo 已启动, 等待就绪 (15秒)..."
sleep 15

# 终端2: MoveIt
gnome-terminal --title="[2] MoveIt Dual-Arm" -- bash -c "
    source /opt/ros/jazzy/setup.bash
    source $WS/install/setup.bash
    echo '>>> 启动 MoveIt move_group...'
    ros2 launch ur5_moveit_config move_group_dual.launch.py
    exec bash"

echo "[2/4] MoveIt 已启动, 等待就绪 (5秒)..."
sleep 5

# 终端3: 双臂运动控制器
gnome-terminal --title="[3] Motion Controller" -- bash -c "
    source /opt/ros/jazzy/setup.bash
    source $WS/install/setup.bash
    echo '>>> 启动双臂运动控制器...'
    ros2 run ur5_motion_control dual_arm_motion_controller
    exec bash"

echo "[3/4] 运动控制器已启动, 等待就绪 (3秒)..."
sleep 3

# 终端4: YOLOv8 检测节点（--auto-start false，等待 /detection_enable 信号）
gnome-terminal --title="[4] YOLOv8 Detection" -- bash -c "
    source /opt/ros/jazzy/setup.bash
    source $WS/install/setup.bash
    export QT_LOGGING_RULES='qt.qpa.fonts.warning=false'
    echo '等待相机话题就绪...'
    until ros2 topic list 2>/dev/null | grep -qF '/overhead_camera/rgb/image_raw'; do
        sleep 2
    done
    echo '相机就绪，启动检测节点（等待机械臂归位后开始检测）...'
    "$PYTHON" "$DETECT_SCRIPT" --model "$MODEL" --auto-start false
    exec bash"

echo "[4/4] 检测节点已启动, 等待就绪 (3秒)..."
sleep 3

# 终端5: sorting_demo_1（双臂归位 → 发布 /detection_enable）
gnome-terminal --title="[5] Sorting Demo 1" -- bash -c "
    source /opt/ros/jazzy/setup.bash
    source $WS/install/setup.bash
    echo '>>> 启动分拣 Demo 1：双臂归位序列...'
    ros2 run ur5_sorting sorting_demo_1
    exec bash"

echo ""
echo "========================================"
echo "  ✓ 分拣 Demo 1 所有组件启动完成！"
echo "========================================"
echo ""
echo "终端说明:"
echo "  [1] Gazebo 双臂仿真"
echo "  [2] MoveIt move_group"
echo "  [3] 双臂运动控制器"
echo "  [4] YOLOv8 实时检测（归位完成后自动开始）"
echo "  [5] Sorting Demo 1（双臂归位 → 触发检测）"
echo ""
