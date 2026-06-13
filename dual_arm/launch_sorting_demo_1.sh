#!/bin/bash
# ========================================
# 分拣演示 Demo 1 启动脚本（tmux 版）
# 用法:  ./launch_sorting_demo_1.sh
# 关闭:  tmux kill-session -t sorting_demo
#        或在 tmux 内按 Ctrl+b 再输入 :kill-session
#
# 启动顺序:
#   [0] Gazebo 双臂仿真
#   [1] MoveIt move_group
#   [2] 双臂运动控制器
#   [3] YOLOv8 检测节点（--auto-start false）
#   [4] sorting_demo_1（双臂归位 → 触发检测）
# ========================================

set -e

WS=/home/lcy/Simulation/dual_arm
PYTHON="$WS/.venv/bin/python3"
DETECT_SCRIPT="$WS/src/ur5_project/ur5_perception/ur5_perception/detect_node.py"
MODEL="$WS/models/best.pt"
SESSION="sorting_demo"

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

# ── 检查 tmux ──
if ! command -v tmux &>/dev/null; then
    echo "错误：未安装 tmux，请运行：sudo apt install tmux"
    exit 1
fi

# 若 session 已存在则先清理
tmux kill-session -t "$SESSION" 2>/dev/null || true

# ── 创建 session，主窗口分为 2×2 pane 布局 ──
#   pane 0 (左上): Gazebo
#   pane 1 (右上): MoveIt
#   pane 2 (左下): MotionCtrl
#   pane 3 (右下): YOLO
tmux new-session -d -s "$SESSION" -n "Main"
tmux split-window  -t "$SESSION:0.0" -h          # 左右分割 → pane 1
tmux split-window  -t "$SESSION:0.0" -v          # 左侧上下分割 → pane 2
tmux split-window  -t "$SESSION:0.1" -v          # 右侧上下分割 → pane 3

# 按 Esc 键关闭所有进程（含 Gazebo GUI），带确认提示
tmux bind-key -n Escape confirm-before -p "按 y 关闭所有进程（含 Gazebo），按 n 取消" \
    "run-shell '$WS/stop_sorting_demo_1.sh'"

# 显示各 pane 标题
tmux set-option -t "$SESSION" pane-border-status top
tmux select-pane -t "$SESSION:0.0" -T "Gazebo"
tmux select-pane -t "$SESSION:0.1" -T "MoveIt"
tmux select-pane -t "$SESSION:0.2" -T "MotionCtrl"
tmux select-pane -t "$SESSION:0.3" -T "YOLO"

# ── pane 0: Gazebo（立即启动）──
tmux send-keys -t "$SESSION:0.0" \
    "source /opt/ros/jazzy/setup.bash && source $WS/install/setup.bash && echo '>>> 启动 Gazebo 双臂仿真...' && ros2 launch ur5_gazebo sim_dual.launch.py" Enter

echo "[1/5] Gazebo 已启动，等待就绪 (15秒)..."
sleep 15

# ── pane 1: MoveIt ──
tmux send-keys -t "$SESSION:0.1" \
    "source /opt/ros/jazzy/setup.bash && source $WS/install/setup.bash && echo '>>> 启动 MoveIt move_group...' && ros2 launch ur5_moveit_config move_group_dual.launch.py" Enter

echo "[2/5] MoveIt 已启动，等待就绪 (5秒)..."
sleep 5

# ── pane 2: 双臂运动控制器 ──
tmux send-keys -t "$SESSION:0.2" \
    "source /opt/ros/jazzy/setup.bash && source $WS/install/setup.bash && echo '>>> 启动双臂运动控制器...' && ros2 run ur5_motion_control dual_arm_motion_controller" Enter

echo "[3/5] 运动控制器已启动，等待就绪 (3秒)..."
sleep 3

# ── pane 3: YOLOv8 检测节点 ──
YOLO_CMD="source /opt/ros/jazzy/setup.bash && source $WS/install/setup.bash && export QT_LOGGING_RULES='qt.qpa.fonts.warning=false' && echo '等待相机话题就绪...' && until ros2 topic list 2>/dev/null | grep -qF '/overhead_camera/rgb/image_raw'; do sleep 2; done && echo '相机就绪，启动检测节点（等待机械臂归位后开始检测）...' && $PYTHON $DETECT_SCRIPT --model $MODEL --auto-start false"
tmux send-keys -t "$SESSION:0.3" "$YOLO_CMD" Enter

echo "[4/5] 检测节点已启动，等待就绪 (3秒)..."
sleep 3

# ── 窗口 1: Sorting Demo 1（单独窗口，方便查看输出）──
tmux new-window -t "$SESSION:1" -n "SortDemo"
tmux send-keys -t "$SESSION:1" \
    "source /opt/ros/jazzy/setup.bash && source $WS/install/setup.bash && echo '>>> 启动分拣 Demo 1：双臂归位序列...' && ros2 run ur5_sorting sorting_demo_1" Enter

# 切回主分屏窗口，聚焦 Gazebo pane
tmux select-window -t "$SESSION:0"
tmux select-pane   -t "$SESSION:0.0"

echo ""
echo "========================================"
echo "  ✓ 分拣 Demo 1 所有组件启动完成！"
echo "========================================"
echo ""
echo "tmux 快捷键:"
echo "  切换到 SortDemo:  Ctrl+b 再按 1"
echo "  回到分屏主界面:   Ctrl+b 再按 0"
echo "  在 pane 间移动:   Ctrl+b 再按方向键"
echo "  关闭全部:         tmux kill-session -t $SESSION"
echo "  暂时离开:         Ctrl+b 再按 d"
echo "  重新连接:         tmux attach -t $SESSION"
echo ""

tmux attach -t "$SESSION"
