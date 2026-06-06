# UR5 机器人项目

## 项目结构

```
src/
└── ur5_project/                 # 项目顶层文件夹
    │
    ├── ur5_bringup/             # 系统总入口
    │   ├── launch/
    │   │   └── system.launch.py # 系统启动文件（替代 launch_all.sh）
    │   └── config/              # 全局参数
    │
    ├── ur5_interfaces/          # 自定义接口（原 detection_interfaces）
    │   ├── msg/                 # 消息定义
    │   ├── srv/                 # 服务定义
    │   └── action/              # 动作定义
    │
    ├── ur5_perception/          # 视觉感知系统（合并 camera_detect + block_classification）
    │   ├── ur5_perception/      # Python源码
    │   │   ├── color_detector.py
    │   │   ├── color_detector_handshake.py
    │   │   ├── block_classifier_action.py
    │   │   └── image_viewer.py
    │   ├── launch/
    │   └── config/              # 视觉参数（HSV阈值、模型路径等）
    │
    ├── ur5_motion_control/      # 运动控制（原 motion_control_pkg）
    │   ├── ur5_motion_control/  # MoveIt 控制接口逻辑
    │   └── launch/
    │
    ├── ur5_sensors/             # 传感器驱动（原 imu_ros2_device）
    │   └── ur5_sensors/
    │       └── ybimu_driver.py
    │
    ├── ur5_moveit_config/       # MoveIt 配置
    │   ├── config/
    │   └── launch/
    │
    └── ur5_gazebo/              # Gazebo 仿真（原 ur5_sim_pkg）
        ├── urdf/
        ├── worlds/
        ├── models/
        ├── rviz/
        └── launch/
```

## 编译项目

```bash
cd /home/lcy/ur5_ws
colcon build
source install/setup.bash
```

## 启动系统

### 方式一：使用新的 Launch 文件（推荐）

```bash
ros2 launch ur5_bringup system.launch.py
```

### 方式二：分别启动各个组件

```bash
# 1. 启动 Gazebo 仿���
ros2 launch ur5_gazebo sim.launch.py

# 2. 启动 MoveIt
ros2 launch ur5_moveit_config move_group.launch.py

# 3. 启动运动控制节点
ros2 run ur5_motion_control motion_controller_action

# 4. 启动颜色检测节点
ros2 run ur5_perception color_detector_handshake --ros-args -p use_sim_time:=True

# 5. 启动物块分类节点
ros2 run ur5_perception block_classifier_action
```

## 包说明

### ur5_bringup
系统启动包，提供统一的启动入口。

### ur5_interfaces
定义了项目中使用的自定义消息、服务和动作接口。

### ur5_perception
视觉感知系统，包含：
- 颜色检测节点
- 物块分类节点
- 图像查看器

### ur5_motion_control
基于 MoveIt 的运动控制系统，提供 Action Server 接口。

### ur5_sensors
传感器驱动，目前包含 IMU 驱动。

### ur5_moveit_config
MoveIt 配置文件，由 MoveIt Setup Assistant 生成。

### ur5_gazebo
Gazebo 仿真环境，包含机器人模型、世界文件和 RViz 配置。

## 重构说明

本次重构主要完成了以下工作：

1. **创建顶层目录**：所有包统一放在 `ur5_project` 文件夹下
2. **包重命名**：
   - `detection_interfaces` → `ur5_interfaces`
   - `motion_control_pkg` → `ur5_motion_control`
   - `imu_ros2_device` → `ur5_sensors`
   - `ur5_sim_pkg` → `ur5_gazebo`
3. **包合并**：
   - `camera_detect_pkg` + `block_classification_pkg` → `ur5_perception`
4. **新增包**：
   - `ur5_bringup`：提供统一的系统启动入口
5. **更新依赖**：所有包的依赖引用已更新为新的包名

## 注意事项

- 所有旧的编译文件已清理，需要重新编译
- 启动系统前请确保已 source 环境：`source install/setup.bash`
- 如果遇到包找不到的问题，请检查是否已正确编译所有包
