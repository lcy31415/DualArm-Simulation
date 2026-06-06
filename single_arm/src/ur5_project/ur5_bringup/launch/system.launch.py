#!/usr/bin/env python3
"""
UR5 System Launch File
启动完整的 UR5 仿真系统，包括 Gazebo、MoveIt、运动控制和视觉检测
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # 获取包路径
    ur5_gazebo_share = FindPackageShare('ur5_gazebo')
    ur5_moveit_config_share = FindPackageShare('ur5_moveit_config')

    # 1. 启动 Gazebo 仿真
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([ur5_gazebo_share, 'launch', 'sim.launch.py'])
        ])
    )

    # 2. 启动 MoveIt (延迟10秒等待 Gazebo)
    moveit_launch = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([ur5_moveit_config_share, 'launch', 'move_group.launch.py'])
                ])
            )
        ]
    )

    # 3. 启动 RViz (延迟15秒)
    rviz_config_path = PathJoinSubstitution([ur5_gazebo_share, 'rviz', 'ur5_config.rviz'])
    rviz_node = TimerAction(
        period=15.0,
        actions=[
            ExecuteProcess(
                cmd=['rviz2', '-d', rviz_config_path],
                output='screen'
            )
        ]
    )

    # 4. 启动运动控制节点 (延迟18秒)
    motion_control_node = TimerAction(
        period=18.0,
        actions=[
            Node(
                package='ur5_motion_control',
                executable='motion_controller_action',
                name='motion_controller_action',
                output='screen'
            )
        ]
    )

    # 5. 启动颜色检测节点 (延迟21秒)
    color_detector_node = TimerAction(
        period=21.0,
        actions=[
            Node(
                package='ur5_perception',
                executable='color_detector_handshake',
                name='color_detector_handshake',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )

    # 6. 启动物块分类节点 (延迟24秒)
    block_classifier_node = TimerAction(
        period=24.0,
        actions=[
            Node(
                package='ur5_perception',
                executable='block_classifier_action',
                name='block_classifier_action',
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        gazebo_launch,
        moveit_launch,
        rviz_node,
        motion_control_node,
        color_detector_node,
        block_classifier_node,
    ])

