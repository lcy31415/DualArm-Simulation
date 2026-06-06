import os
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # ---------------------------------------------------------
    # 这里非常关键：加载你的机器人 MoveIt 配置
    # 1. 首先手动加载 URDF
    # 2. 然后使用 MoveItConfigsBuilder 构建其他配置
    # ---------------------------------------------------------

    # 加载 URDF
    ur5_gazebo = get_package_share_directory('ur5_gazebo')
    xacro_file = os.path.join(ur5_gazebo, 'urdf', 'ur5_official.urdf.xacro')
    doc = xacro.process_file(xacro_file)
    robot_description = {'robot_description': doc.toxml()}

    # 使用 MoveItConfigsBuilder 构建配置（不包含 robot_description）
    moveit_config = (
        MoveItConfigsBuilder(robot_name="ur5_robot", package_name="ur5_moveit_config")
        .robot_description_semantic(file_path="config/ur5.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])  # 启用 OMPL 规划
        .joint_limits(file_path="config/joint_limits.yaml")
        .to_moveit_configs()
    )

    # 运行我们的 Python 节点，并把参数传进去
    demo_node = Node(
        package="ur5_motion_control",
        executable="motion_controller",  # 对应 setup.py 中的 entry_point
        output="screen",
        parameters=[
            robot_description,  # 手动加载的 URDF
            moveit_config.to_dict(),
            {'use_sim_time': True},
            # 添加 planning pipeline 配置
            moveit_config.planning_pipelines,
        ],
    )

    return LaunchDescription([demo_node])