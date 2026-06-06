#!/usr/bin/env python3
"""
MoveIt launch file for UR5 with Gazebo simulation
启动 MoveIt 与 Gazebo 仿真集成
"""

import os
import xacro
from pathlib import Path
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # 加载 URDF
    ur5_gazebo = get_package_share_directory('ur5_gazebo')
    xacro_file = os.path.join(ur5_gazebo, 'urdf', 'ur5_official.urdf.xacro')
    doc = xacro.process_file(xacro_file)
    robot_description = {'robot_description': doc.toxml()}

    # 使用 MoveItConfigsBuilder 构建配置（跳过 robot_description）
    moveit_config_pkg = get_package_share_directory('ur5_moveit_config')
    moveit_config = (
        MoveItConfigsBuilder(robot_name="ur5_robot", package_name="ur5_moveit_config")
        .robot_description_semantic(file_path="config/ur5.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .joint_limits(file_path="config/joint_limits.yaml")
        .to_moveit_configs()
    )

    # Move Group 节点
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,  # 手动加载的 URDF
            moveit_config.to_dict(),
            {'use_sim_time': True},
            {'publish_robot_description_semantic': True},
        ]
    )

    # RViz 节点（可选）
    rviz_config = os.path.join(moveit_config_pkg, 'config', 'moveit.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        parameters=[
            robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': True},
        ]
    )

    return LaunchDescription([
        move_group_node,
        # rviz_node,  # 取消注释以启动 RViz
    ])
