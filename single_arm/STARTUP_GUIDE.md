# UR5 系统启动指南

## 方式一：分别启动各个组件（推荐）

在不同的终端中按顺序运行以下脚本：

### 1. 启动 Gazebo 仿真环境
```bash
cd /home/lcy/ur5_ws
./start_gazebo.sh
```
等待 Gazebo 完全启动（约10秒）

### 2. 启动 MoveIt
```bash
cd /home/lcy/ur5_ws
./start_moveit.sh
```
等待 MoveIt 完全启动（约8秒）

### 3. 启动 RViz（可选）
```bash
cd /home/lcy/ur5_ws
./start_rviz.sh
```

### 4. 启动运动控制节点
```bash
cd /home/lcy/ur5_ws
./start_motion_control.sh
```
等待节点启动（约3秒）

### 5. 启动颜色检测节点
```bash
cd /home/lcy/ur5_ws
./start_color_detector.sh
```
等待节点启动（约3秒）

### 6. 启动物块分类节点
```bash
cd /home/lcy/ur5_ws
./start_block_classifier.sh
```

## 方式二：使用统一启动脚本

```bash
cd /home/lcy/ur5_ws
./start_system.sh
```

注意：此方式会在同一个终端中启动所有组件，如果某个组件出错，整个系统会停止。

## 测试命令

在新的终端中运行以下命令测试系统：

```bash
cd /home/lcy/ur5_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# 查看检测批量数据
ros2 topic echo /detection_batch

# 查看关节状态
ros2 topic echo /joint_states

# 查看相机图像
ros2 topic echo /camera/rgb/image_raw
```

## 关闭系统

在每个终端中按 `Ctrl+C` 停止相应的节点。

## 故障排除

### 问题：找不到包
**解决方法**：确保在新的终端中运行，并正确 source 环境：
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

### 问题：Gazebo 启动失败
**解决方法**：检查是否有其他 Gazebo 实例在运行：
```bash
killall gz
```

### 问题：MoveIt 无法连接到机器人
**解决方法**：确保 Gazebo 已完全启动，并且机器人已生成。

## 项目结构

```
src/ur5_project/
├── ur5_bringup/          # 系统启动包
├── ur5_interfaces/       # 自定义接口
├── ur5_perception/       # 视觉感知系统
├── ur5_motion_control/   # 运动控制
├── ur5_sensors/          # 传感器驱动
├── ur5_moveit_config/    # MoveIt 配置
└── ur5_gazebo/           # Gazebo 仿真
```
