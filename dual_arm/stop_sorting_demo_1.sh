#!/bin/bash
# 关闭 sorting_demo_1 的所有进程（包括 Gazebo GUI）
SESSION="sorting_demo"

# 关闭 Gazebo 相关进程（兼容新版 gz 和旧版 gazebo）
pkill -SIGTERM -f "gz sim"      2>/dev/null || true
pkill -SIGTERM -f "gz-sim"      2>/dev/null || true
pkill -SIGTERM -f "gzserver"    2>/dev/null || true
pkill -SIGTERM -f "gzclient"    2>/dev/null || true
pkill -SIGTERM -f "ign gazebo"  2>/dev/null || true

sleep 0.5

# 关闭 tmux session（同时结束其余所有节点）
tmux kill-session -t "$SESSION" 2>/dev/null || true
