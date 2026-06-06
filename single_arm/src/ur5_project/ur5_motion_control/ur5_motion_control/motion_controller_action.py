#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    BoundingVolume,
)
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from ur5_interfaces.action import MotionCommand
from rclpy.task import Future
import time
import math


class MotionControllerAction(Node):
    def __init__(self):
        super().__init__("motion_controller_action")

        # 1. 创建 MoveGroup Action 客户端
        self._moveit_client = ActionClient(
            self, MoveGroup, "/move_action"
        )

        # 2. 创建 IK 服务客户端
        self._ik_client = self.create_client(
            GetPositionIK,
            '/compute_ik'
        )

        # 3. 订阅关节状态
        self._current_joint_state = None
        self._joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            10
        )

        # 4. 创建夹爪控制 Publisher
        self._gripper_pub = self.create_publisher(
            Float64MultiArray,
            '/gripper_controller/commands',
            10
        )

        # 5. 创建 MotionCommand Action Server
        self._action_server = ActionServer(
            self,
            MotionCommand,
            '/motion_command',
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback
        )

        # 当前执行状态
        self._current_goal_handle = None

        self.get_logger().info("等待 MoveGroup Action 服务器...")
        self._moveit_client.wait_for_server()
        self.get_logger().info("等待 IK 服务...")
        self._ik_client.wait_for_service()
        self.get_logger().info("MotionCommand Action 服务器已启动！")

    def _joint_state_callback(self, msg: JointState):
        """接收并存储当前关节状态"""
        self._current_joint_state = msg

    async def _async_sleep(self, duration):
        """使用 ROS 2 timer 实现异步延迟"""
        future = Future()

        def timer_callback():
            if not future.done():
                future.set_result(None)

        timer = self.create_timer(duration, timer_callback)

        try:
            await future
        finally:
            timer.cancel()
            self.destroy_timer(timer)

    def _goal_callback(self, goal_request):
        """接受或拒绝新的 goal"""
        self.get_logger().info(f"收到新的 goal 请求: {goal_request.command_type}")
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        """处理取消请求"""
        self.get_logger().info("收到取消请求")
        return CancelResponse.ACCEPT

    async def _execute_callback(self, goal_handle):
        """
        执行 Action goal
        """
        self._current_goal_handle = goal_handle
        request = goal_handle.request

        self.get_logger().info(
            f"开始执行: {request.command_type}"
        )

        # 发送初始反馈
        feedback = MotionCommand.Feedback()
        feedback.current_state = "received"
        feedback.progress = 0.0
        goal_handle.publish_feedback(feedback)

        # 执行命令
        success = False
        message = ""
        error_code = 0

        try:
            if request.command_type == "move":
                success, message, error_code = await self._execute_move(
                    goal_handle,
                    request.target_position.x,
                    request.target_position.y,
                    request.target_position.z
                )
            elif request.command_type == "gripper":
                success, message, error_code = await self._execute_gripper(
                    goal_handle,
                    request.gripper_close
                )
            else:
                success = False
                message = f"未知命令类型: {request.command_type}"
                error_code = 1

        except Exception as e:
            success = False
            message = f"执行异常: {str(e)}"
            error_code = 99
            self.get_logger().error(message)

        # 发送结果
        result = MotionCommand.Result()
        result.success = success
        result.message = message
        result.error_code = error_code

        if success:
            goal_handle.succeed()
            self.get_logger().info(f"✓ 执行成功: {message}")
        else:
            goal_handle.abort()
            self.get_logger().error(f"✗ 执行失败: {message}")

        return result

    def _normalize_angle(self, angle):
        """将角度归一化到 [-π, π] 范围"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def _compute_joint_distance(self, joint_positions):
        """计算目标关节角度与当前关节角度的距离"""
        if self._current_joint_state is None:
            return float('inf')

        # 获取当前关节角度 (只取前6个关节,忽略夹爪)
        current_joints = self._current_joint_state.position[:6]

        # 计算距离 (考虑角度周期性)
        distance = 0.0
        for i in range(6):
            # 归一化角度差
            diff = self._normalize_angle(joint_positions[i] - current_joints[i])
            distance += diff ** 2

        return math.sqrt(distance)

    async def _compute_best_ik_solution(self, pose_goal):
        """
        调用 IK 服务,选择最接近当前配置的解
        返回: (success, joint_positions, error_message)
        """
        # 构建 IK 请求
        ik_request = GetPositionIK.Request()
        ik_request.ik_request.group_name = "ur5_arm"
        ik_request.ik_request.pose_stamped = pose_goal
        ik_request.ik_request.timeout.sec = 1
        ik_request.ik_request.timeout.nanosec = 0

        # 如果有当前关节状态,作为种子
        if self._current_joint_state is not None:
            ik_request.ik_request.robot_state.joint_state = self._current_joint_state

        # 调用 IK 服务
        self.get_logger().info("调用 IK 服务计算关节角度...")
        future = self._ik_client.call_async(ik_request)
        await future

        ik_response = future.result()

        if ik_response is None:
            return False, None, "IK 服务调用失败"

        if ik_response.error_code.val != 1:  # 1 = SUCCESS
            return False, None, f"IK 求解失败,错误代码: {ik_response.error_code.val}"

        # 获取解
        joint_positions = ik_response.solution.joint_state.position[:6]

        # 计算与当前配置的距离
        distance = self._compute_joint_distance(joint_positions)

        self.get_logger().info(f"IK 求解成功,距离当前配置: {distance:.3f} rad")

        return True, joint_positions, ""

    async def _execute_move(self, goal_handle, x, y, z):
        """
        执行移动命令 - 使用关节空间规划避免"拧麻花"
        """
        self.get_logger().info(f"规划移动到: ({x:.3f}, {y:.3f}, {z:.3f})")

        # 发送规划反馈
        feedback = MotionCommand.Feedback()
        feedback.current_state = "planning"
        feedback.progress = 0.2
        goal_handle.publish_feedback(feedback)

        # 1. 构建目标位姿
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "base_link"
        pose_goal.header.stamp = self.get_clock().now().to_msg()
        pose_goal.pose.position.x = x
        pose_goal.pose.position.y = y
        pose_goal.pose.position.z = z

        # 设置姿态（末端垂直向下）
        pose_goal.pose.orientation.x = 1.0
        pose_goal.pose.orientation.y = 0.0
        pose_goal.pose.orientation.z = 0.0
        pose_goal.pose.orientation.w = 0.0

        # 2. 调用 IK 服务计算最优关节角度
        success, joint_positions, error_msg = await self._compute_best_ik_solution(pose_goal)

        if not success:
            return False, f"IK 求解失败: {error_msg}", -4

        # 打印目标关节角度
        joint_degrees = [math.degrees(j) for j in joint_positions]
        self.get_logger().info(f"目标关节角度(度): {[f'{d:.1f}' for d in joint_degrees]}")

        # 3. 创建 MoveGroup Goal - 使用���节约束
        moveit_goal = MoveGroup.Goal()
        moveit_goal.request.group_name = "ur5_arm"
        moveit_goal.request.num_planning_attempts = 10
        moveit_goal.request.allowed_planning_time = 5.0
        moveit_goal.request.max_velocity_scaling_factor = 0.1
        moveit_goal.request.max_acceleration_scaling_factor = 0.1

        # 4. 设置关节约束 (而不是位姿约束)
        joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                       "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]

        constraints = Constraints()
        for joint_name, joint_value in zip(joint_names, joint_positions):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_name
            joint_constraint.position = joint_value
            joint_constraint.tolerance_above = 0.01
            joint_constraint.tolerance_below = 0.01
            joint_constraint.weight = 1.0
            constraints.joint_constraints.append(joint_constraint)

        moveit_goal.request.goal_constraints.append(constraints)

        # 5. 发送 MoveIt goal
        self.get_logger().info("发送关节空间规划请求...")
        send_goal_future = self._moveit_client.send_goal_async(moveit_goal)

        # 等待 goal 被接受
        await send_goal_future
        moveit_goal_handle = send_goal_future.result()

        if not moveit_goal_handle.accepted:
            return False, "MoveIt 规划请求被拒绝", 2

        self.get_logger().info("规划请求已接受，等待执行...")

        # 发送执行反馈
        feedback.current_state = "executing"
        feedback.progress = 0.5
        goal_handle.publish_feedback(feedback)

        # 等待执行完成
        result_future = moveit_goal_handle.get_result_async()
        await result_future
        moveit_result = result_future.result().result

        # 检查执行结果
        if moveit_result.error_code.val == 1:  # SUCCESS
            feedback.current_state = "completed"
            feedback.progress = 1.0
            goal_handle.publish_feedback(feedback)
            return True, "移动成功", 0
        else:
            return False, f"MoveIt 规划/执行失败，错误代码: {moveit_result.error_code.val}", moveit_result.error_code.val

    async def _execute_gripper(self, goal_handle, close):
        """
        执行夹爪命令（异步）
        """
        gripper_action = "闭合" if close else "打开"
        self.get_logger().info(f"执行夹爪{gripper_action}...")

        # 发送反馈
        feedback = MotionCommand.Feedback()
        feedback.current_state = "executing"
        feedback.progress = 0.5
        goal_handle.publish_feedback(feedback)

        # 发布夹爪命令
        msg = Float64MultiArray()
        msg.data = [0.8 if close else 0.0]
        self._gripper_pub.publish(msg)

        # 等待夹爪动作完成（使用 ROS 2 timer）
        await self._async_sleep(1.5)

        # 完成反馈
        feedback.current_state = "completed"
        feedback.progress = 1.0
        goal_handle.publish_feedback(feedback)

        return True, f"夹爪{gripper_action}完成", 0


def main(args=None):
    rclpy.init(args=args)
    controller = MotionControllerAction()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
