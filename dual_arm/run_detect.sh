#!/bin/bash
# run_detect.sh  (dual_arm)
# 1. 后台启动 ur5_sorting/sorting_demo_1（两臂归位 → 发布 /detection_enable）
# 2. 前台启动 detect_node（--auto-start false，等收到信号后才开始检测）
#
# 用法：
#   ./run_detect.sh
#   ./run_detect.sh --model /path/to/other/best.pt

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

PYTHON="$WS/.venv/bin/python3"
DETECT_SCRIPT="$WS/src/ur5_project/ur5_perception/ur5_perception/detect_node.py"
MODEL="$WS/models/best.pt"

# ── 检查 venv（首次使用需运行 setup_perception_env.sh） ──
if [ ! -f "$PYTHON" ]; then
    echo "错误：未找到虚拟环境，请先运行："
    echo "  bash $WS/setup_perception_env.sh"
    exit 1
fi

# ── 检查模型权重 ──
if [ ! -f "$MODEL" ]; then
    echo "错误：未找到模型权重: $MODEL"
    exit 1
fi

# ── 加载 ROS2 ──
if [ -z "$ROS_DISTRO" ]; then
    source /opt/ros/jazzy/setup.bash
fi
source "$WS/install/setup.bash"

export QT_LOGGING_RULES="qt.qpa.fonts.warning=false"

echo "============================================"
echo "  dual_arm 启动序列 + YOLOv8 OBB 实时检测"
echo "  Python  : $PYTHON"
echo "  模型    : $MODEL"
echo "  ROS     : $ROS_DISTRO"
echo "  按 Ctrl+C 停止"
echo "============================================"
echo ""

# ── 后台：两臂归位序列（归位完成后自动退出） ──
echo ">>> 启动两臂归位序列 (ur5_sorting/sorting_demo_1)..."
ros2 run ur5_sorting sorting_demo_1 &
STARTUP_PID=$!

# ── 前台：检测节点（--auto-start false，等待 /detection_enable） ──
echo ">>> 启动检测节点（等待机械臂就绪后开始检测）..."
"$PYTHON" "$DETECT_SCRIPT" --model "$MODEL" --auto-start false "$@"

# 检测节点退出后，清理归位序列（通常已自行退出）
kill "$STARTUP_PID" 2>/dev/null || true
