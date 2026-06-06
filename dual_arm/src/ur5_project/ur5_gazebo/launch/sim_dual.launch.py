"""
Dual-arm Gazebo simulation launch
启动双 UR5 + 双夹爪 + 俯视相机 + workspace 地标 仿真环境
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg_name = 'ur5_gazebo'
    pkg_share = get_package_share_directory(pkg_name)
    ur_description_path = get_package_share_directory('ur_description')

    # ================= Gazebo 资源路径 =================
    models_path = os.path.join(pkg_share, 'models')
    resource_env = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=':'.join([os.path.join(ur_description_path, '..'), models_path])
    )
    # ring_torus 模型已复制到 models_path，无需额外路径

    # ================= 处理双臂 URDF =================
    xacro_file = os.path.join(pkg_share, 'urdf', 'ur5_dual.urdf.xacro')
    doc = xacro.process_file(xacro_file)
    robot_description = {'robot_description': doc.toxml()}

    # ================= 启动 Gazebo（使用双臂世界） =================
    world_file = os.path.join(pkg_share, 'worlds', 'workspace_dual.sdf')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r {world_file}',
            'on_exit_shutdown': 'true'
        }.items(),
    )

    # ================= Gazebo-ROS 桥接（时钟 + 俯视相机） =================
    # 俯视相机话题路径 (Gazebo 内部): overhead_camera/image_raw, overhead_camera/rgbd
    # ROS 标准话题路径 (经重映射): /overhead_camera/rgb/image_raw 等
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # 时钟
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # RGB 图像
            '/overhead_camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/overhead_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            # RGBD 深度
            '/overhead_camera/rgbd/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/overhead_camera/rgbd/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '--ros-args',
            '-r', '/overhead_camera/image_raw:=/overhead_camera/rgb/image_raw',
            '-r', '/overhead_camera/camera_info:=/overhead_camera/rgb/camera_info',
            '-r', '/overhead_camera/rgbd/depth_image:=/overhead_camera/depth/image_raw',
            '-r', '/overhead_camera/rgbd/points:=/overhead_camera/depth/points',
        ],
        output='screen'
    )

    # ================= Spawn 机器人（位置已在 URDF 内 origin 写死，所以 spawn 在 0,0,0） =================
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'ur5_dual_robot',
                   '-x', '0', '-y', '0', '-z', '0'],
        output='screen'
    )

    # ================= Robot State Publisher =================
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    # ================= 控制器 Spawner（5 个，延迟 8 秒等 Gazebo 完全就绪） =================
    # --controller-manager-timeout 60：最多等 60 秒让 controller_manager 出现，避免过早超时
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

    # ================= 相机内参发布器（Gazebo 不自动发布 camera_info，RViz Camera 需要） =================
    camera_info_pub = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='ur5_gazebo',
                executable='camera_info_publisher.py',
                name='overhead_camera_info_publisher',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )

    return LaunchDescription([
        resource_env,
        gazebo,
        bridge,
        node_robot_state_publisher,
        spawn_entity,
        load_controllers,
        camera_info_pub,
    ])
