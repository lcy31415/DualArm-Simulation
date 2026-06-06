#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_detect_pkg',
            executable='color_detector',
            name='color_detector',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'min_contour_area': 300},
                {'enable_visualization': True}
            ]
        ),
        Node(
            package='camera_detect_pkg',
            executable='image_viewer',
            name='image_viewer',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
    ])
