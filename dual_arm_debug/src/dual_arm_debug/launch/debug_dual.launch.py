"""
Dual-arm debug launch — Gazebo simulation with manual joint slider GUI.

Same world, same robot, same initial pose as the sorting simulation.
No MoveIt, no motion controllers, no classifier — just the slider panel.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    debug_pkg = get_package_share_directory("dual_arm_debug")
    gazebo_pkg = get_package_share_directory("ur5_gazebo")
    ur_description_path = get_package_share_directory("ur_description")

    # ================= Gazebo 资源路径 =================
    models_path = os.path.join(gazebo_pkg, "models")
    resource_env = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=":".join([os.path.join(ur_description_path, ".."), models_path])
    )

    # ================= 处理双臂 URDF（debug 版本，含 ros2_control） =================
    xacro_file = os.path.join(debug_pkg, "urdf", "ur5_dual_debug.urdf.xacro")
    doc = xacro.process_file(xacro_file)
    robot_description = {"robot_description": doc.toxml()}

    # ================= 启动 Gazebo（使用双臂世界，与分拣仿真相同） =================
    world_file = os.path.join(gazebo_pkg, "worlds", "workspace_dual.sdf")
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": f"-r {world_file}",
            "on_exit_shutdown": "true"
        }.items(),
    )

    # ================= Gazebo-ROS 桥接（时钟 + 俯视相机） =================
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/overhead_camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
            "/overhead_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/overhead_camera/rgbd/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/overhead_camera/rgbd/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            "--ros-args",
            "-r", "/overhead_camera/image_raw:=/overhead_camera/rgb/image_raw",
            "-r", "/overhead_camera/camera_info:=/overhead_camera/rgb/camera_info",
            "-r", "/overhead_camera/rgbd/depth_image:=/overhead_camera/depth/image_raw",
            "-r", "/overhead_camera/rgbd/points:=/overhead_camera/depth/points",
        ],
        output="screen"
    )

    # ================= Spawn 机器人 =================
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "robot_description", "-name", "ur5_dual_robot",
                   "-x", "0", "-y", "0", "-z", "0"],
        output="screen"
    )

    # ================= Robot State Publisher =================
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": True}]
    )

    # ================= 控制器 Spawner（延迟 8 秒等 Gazebo 就绪） =================
    def make_spawner(controller_name):
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[controller_name, "--controller-manager-timeout", "60"],
            output="screen",
        )

    load_controllers = TimerAction(
        period=8.0,
        actions=[
            make_spawner("joint_state_broadcaster"),
            make_spawner("left_arm_controller"),
            make_spawner("right_arm_controller"),
            make_spawner("left_gripper_controller"),
            make_spawner("right_gripper_controller"),
        ]
    )

    # ================= 相机内参发布器 =================
    camera_info_pub = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="ur5_gazebo",
                executable="camera_info_publisher.py",
                name="overhead_camera_info_publisher",
                output="screen",
                parameters=[{"use_sim_time": True}]
            )
        ]
    )

    # ================= 手动调试 GUI（延迟 10 秒，等控制器完全就绪） =================
    debug_gui = TimerAction(
        period=10.0,
        actions=[
            Node(
                package="dual_arm_debug",
                executable="dual_arm_joint_publisher_gui.py",
                name="dual_arm_debug_gui",
                output="screen",
                parameters=[{"use_sim_time": True}]
            )
        ]
    )

    return LaunchDescription([
        resource_env,
        gazebo,
        bridge,
        robot_state_publisher,
        spawn_entity,
        load_controllers,
        camera_info_pub,
        debug_gui,
    ])
