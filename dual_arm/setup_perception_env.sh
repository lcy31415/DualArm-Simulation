#!/bin/bash
# ========================================
# setup_perception_env.sh
# 为 ur5_perception 检测节点创建 Python 虚拟环境
# 安装 ultralytics（YOLOv8）及 OpenCV
#
# 用法：bash setup_perception_env.sh
# ========================================

set -e

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV="$WS/.venv"

echo "========================================"
echo "  创建感知虚拟环境: $VENV"
echo "========================================"

# ── 创建 venv（保留系统 site-packages 以访问 ROS2 Python 包） ──
python3 -m venv --system-site-packages "$VENV"

"$VENV/bin/pip" install --upgrade pip

echo ">>> 安装 ultralytics（YOLOv8）..."
"$VENV/bin/pip" install ultralytics

echo ">>> 安装 opencv-python..."
"$VENV/bin/pip" install "opencv-python>=4.8"

# 强制在 venv 内安装 matplotlib，覆盖系统版本（系统版本用 NumPy 1.x 编译，与 venv 的 NumPy 2.x 不兼容）
echo ">>> 安装 matplotlib（覆盖系统版本以修复 NumPy ABI 冲突）..."
"$VENV/bin/pip" install --ignore-installed matplotlib

echo ""
echo "========================================"
echo "  ✓ 感知环境创建完成"
echo "  Python: $VENV/bin/python3"
echo "========================================"
