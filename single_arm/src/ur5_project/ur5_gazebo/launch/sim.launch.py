import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_name = 'ur5_gazebo'
    pkg_share = get_package_share_directory(pkg_name)

    # 获取 ur_description 的路径，用于让 Gazebo 找到 Mesh 文件
    ur_description_path = get_package_share_directory('ur_description')

    # ================= 关键修改 1: 设置环境变量 =================
    # 告诉 Gazebo 去哪里找模型文件 (Mesh + 自定义模型)
    # 添加包的 models 目录到 Gazebo 资源路径
    models_path = os.path.join(pkg_share, 'models')
    resource_env = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=':'.join([os.path.join(ur_description_path, '..'), models_path])
    )

    # ================= 关键修改 2: 指向新的 Xacro 文件 =================
    xacro_file = os.path.join(pkg_share, 'urdf', 'ur5_official.urdf.xacro')
    doc = xacro.process_file(xacro_file)
    robot_description = {'robot_description': doc.toxml()}

    # 启动 Gazebo（启用时钟桥接）
    # 使用自定义的 workspace.sdf 世界文件
    world_file = os.path.join(pkg_share, 'worlds', 'workspace.sdf')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r {world_file}',
            'on_exit_shutdown': 'true'
        }.items(),
    )

    # ================= Gazebo-ROS 桥接（时钟 + 相机） =================
    # 统一的桥接节点处理所有话题映射
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # 时钟桥接
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # RGB 相机图像
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            # RGB 相机信息
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            # RGBD 深度图像
            '/camera/rgbd/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            # RGBD 点云
            '/camera/rgbd/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '--ros-args',
            '-r', '/camera/image_raw:=/camera/rgb/image_raw',
            '-r', '/camera/camera_info:=/camera/rgb/camera_info',
            '-r', '/camera/rgbd/depth_image:=/camera/depth/image_raw',
            '-r', '/camera/rgbd/points:=/camera/depth/points'
        ],
        output='screen'
    )
    # ============================================

    # 生成机器人
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'ur5_robot', '-x', '-0.4', '-y', '0', '-z', '0'],
        output='screen'
    )

    # Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    # 控制器
    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
        parameters=[{'use_sim_time': True}]
    )

    load_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
        output="screen",
        parameters=[{'use_sim_time': True}]
    )

    # ================= 新增: 加载夹爪控制器 =================
    load_gripper_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller"],
        output="screen",
        parameters=[{'use_sim_time': True}]
    )
    # =====================================================

    # ================= 新增: 启动 RViz =================
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'ur5_config.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    # ==================================================

    return LaunchDescription([
        resource_env,
        gazebo,
        bridge,  # 统一的 Gazebo-ROS 桥接（时钟 + 相机）
        node_robot_state_publisher,
        spawn_entity,
        load_joint_state_broadcaster,
        load_arm_controller,
        load_gripper_controller,
        # rviz_node,  # 暂时禁用 RViz，可手动启动: rviz2
    ])