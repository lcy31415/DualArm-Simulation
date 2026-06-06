"""
detect.launch.py — 启动 ur5_perception 检测节点

用法（需先 colcon build）：
  ros2 launch ur5_perception detect.launch.py
  ros2 launch ur5_perception detect.launch.py model:=/path/to/best.pt
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    model_arg = DeclareLaunchArgument(
        "model",
        default_value=os.path.join(
            os.path.dirname(__file__),
            "../../../../../models/best.pt"
        ),
        description="YOLOv8 OBB 权重文件路径",
    )

    detect_node = Node(
        package="ur5_perception",
        executable="detect_node",
        name="ur5_perception",
        output="screen",
        arguments=["--model", LaunchConfiguration("model")],
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription([model_arg, detect_node])
