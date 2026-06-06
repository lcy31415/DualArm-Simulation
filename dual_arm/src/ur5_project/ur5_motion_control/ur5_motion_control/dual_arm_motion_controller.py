#!/usr/bin/env python3
"""
双臂运动控制器 — 流水线版（独立臂 + 实时碰撞预检）
- 左右臂完全独立运行，各自完成任务后立即开始下一个，不排队等待
- 每条新序列开始时，将第一段规划轨迹与对臂当前正在执行的剩余轨迹做碰撞预检
- 规划(plan_only) 与执行(FollowJointTrajectory) 解耦，双臂真正并发执行
- 运行时：20Hz TF 监控末端距离，触碰安全阈值时立即急停双臂
- MultiThreadedExecutor + ReentrantCallbackGroup 支持并发回调
"""
import json
import math
import threading
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse, ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, RobotState
from moveit_msgs.srv import GetPositionIK, GetStateValidity
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration as RosDuration
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray, Bool, Float32
from sensor_msgs.msg import JointState
from ur5_interfaces.action import MotionCommand
from rclpy.task import Future
import tf2_ros


LEFT_JOINTS = [
    "left_shoulder_pan_joint", "left_shoulder_lift_joint", "left_elbow_joint",
    "left_wrist_1_joint", "left_wrist_2_joint", "left_wrist_3_joint",
]
RIGHT_JOINTS = [
    "right_shoulder_pan_joint", "right_shoulder_lift_joint", "right_elbow_joint",
    "right_wrist_1_joint", "right_wrist_2_joint", "right_wrist_3_joint",
]

SAFETY_DISTANCE = 0.12
WARN_DISTANCE   = 0.20
MONITOR_HZ      = 20.0

# 对臂剩余轨迹时间低于此阈值时视为"即将结束"，直接放行（避免伪冲突）
LIVE_CHECK_SKIP_REMAINING = 1.0   # 秒


class DualArmMotionController(Node):
    def __init__(self):
        super().__init__("dual_arm_motion_controller")

        cb = ReentrantCallbackGroup()

        # ── MoveIt 规划客户端 ──
        self._moveit_left  = ActionClient(self, MoveGroup, "/move_action", callback_group=cb)
        self._moveit_right = ActionClient(self, MoveGroup, "/move_action", callback_group=cb)

        # ── IK 服务 ──
        self._ik_client = self.create_client(GetPositionIK, "/compute_ik", callback_group=cb)

        # ── 状态有效性（规划时碰撞预检） ──
        self._validity_client = self.create_client(
            GetStateValidity, "/check_state_validity", callback_group=cb)

        # ── 轨迹执行客户端（直接对接 ros2_control，绕过 MoveIt 执行队列） ──
        self._exec_left  = ActionClient(
            self, FollowJointTrajectory,
            "/left_arm_controller/follow_joint_trajectory", callback_group=cb)
        self._exec_right = ActionClient(
            self, FollowJointTrajectory,
            "/right_arm_controller/follow_joint_trajectory", callback_group=cb)

        # ── 关节状态 ──
        self._current_js = None
        self.create_subscription(JointState, "/joint_states", self._js_cb, 10, callback_group=cb)

        # ── 夹爪发布者 ──
        self._left_grip_pub  = self.create_publisher(
            Float64MultiArray, "/left_gripper_controller/commands",  10)
        self._right_grip_pub = self.create_publisher(
            Float64MultiArray, "/right_gripper_controller/commands", 10)

        # ── TF ──
        self._tf_buf      = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buf, self, spin_thread=False)

        # ── 急停 ──
        self._estop     = False
        self._estop_pub = self.create_publisher(Bool,    "/collision_estop",      10)
        self._dist_pub  = self.create_publisher(Float32, "/dual_arm_ee_distance", 10)
        self._active_handles: dict[str, object] = {}
        self._handle_lock = threading.Lock()

        # ── Per-arm 互斥锁（同一臂的 goal 串行执行） ──
        self._left_lock  = threading.Lock()
        self._right_lock = threading.Lock()

        # ── 运行时碰撞监控 ──
        self._executing_arms: set[str] = set()
        self._exec_set_lock = threading.Lock()
        self.create_timer(1.0 / MONITOR_HZ, self._monitor_cb, callback_group=cb)

        # ── 流水线：实时轨迹追踪 ──
        # 每臂在执行 FJT 期间，将正在执行的 JointTrajectory 和开始时间记录在此，
        # 供对臂在规划新任务时做"对臂剩余轨迹碰撞预检"使用。
        self._active_traj_lock = threading.Lock()
        self._active_traj: dict = {
            "left":  {"jt": None, "start_ns": 0},
            "right": {"jt": None, "start_ns": 0},
        }

        # ── Action Server ──
        self._action_server = ActionServer(
            self, MotionCommand, "/motion_command",
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=cb,
        )

        self.get_logger().info("等待服务器就绪...")
        self._moveit_left.wait_for_server()
        self._moveit_right.wait_for_server()
        self._ik_client.wait_for_service()
        self._validity_client.wait_for_service()
        self._exec_left.wait_for_server()
        self._exec_right.wait_for_server()
        self.get_logger().info("双臂控制器已启动（流水线 + 实时碰撞预检）")

    # ─────────────────────── 基础回调 ───────────────────────

    def _js_cb(self, msg: JointState):
        self._current_js = msg

    def _goal_callback(self, goal_request):
        if self._estop:
            self.get_logger().warn("急停中，拒绝新 goal")
            return GoalResponse.REJECT
        self.get_logger().info(
            f"收到 goal: arm={goal_request.arm}, type={goal_request.command_type}")
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    async def _execute_callback(self, goal_handle):
        req  = goal_handle.request
        arm  = req.arm
        lock = self._left_lock if arm == "left" else self._right_lock

        while not lock.acquire(blocking=False):
            await self._async_sleep(0.05)

        fb = MotionCommand.Feedback()
        fb.current_state = "received"
        fb.progress = 0.0
        goal_handle.publish_feedback(fb)

        success, message, error_code = False, "", 0
        try:
            if req.command_type == "move":
                success, message, error_code = await self._execute_move(
                    goal_handle, arm,
                    req.target_position.x, req.target_position.y, req.target_position.z)
            elif req.command_type == "gripper":
                success, message, error_code = await self._execute_gripper(
                    goal_handle, arm, req.gripper_close)
            elif req.command_type == "sequence":
                success, message, error_code = await self._execute_sequence(
                    goal_handle, arm, req.sequence_data)
            else:
                success, message, error_code = False, f"未知命令: {req.command_type}", 1
        except Exception as e:
            success, message, error_code = False, f"异常: {e}", 99
            self.get_logger().error(message)
        finally:
            lock.release()

        result = MotionCommand.Result()
        result.success    = success
        result.message    = message
        result.error_code = error_code
        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    # ─────────────────────── 运行时碰撞监控 ───────────────────────

    def _monitor_cb(self):
        with self._exec_set_lock:
            active = len(self._executing_arms) > 0
        if not active or self._estop:
            return
        try:
            t    = self._tf_buf.lookup_transform(
                "left_tool0", "right_tool0", rclpy.time.Time())
            tx   = t.transform.translation
            dist = math.sqrt(tx.x**2 + tx.y**2 + tx.z**2)
        except Exception:
            return

        dm      = Float32()
        dm.data = float(dist)
        self._dist_pub.publish(dm)

        if dist < SAFETY_DISTANCE:
            self._trigger_estop(
                f"末端距离 {dist:.3f}m < 急停阈值 {SAFETY_DISTANCE}m，双臂锁死")
        elif dist < WARN_DISTANCE:
            self.get_logger().warn(
                f"[碰撞预警] 末端距离 {dist:.3f}m < 警告阈值 {WARN_DISTANCE}m")

    def _trigger_estop(self, reason: str):
        if self._estop:
            return
        self._estop = True
        self.get_logger().error(f"[急停] {reason}")
        with self._handle_lock:
            for arm, handle in self._active_handles.items():
                handle.cancel_goal_async()
                self.get_logger().warn(f"[急停] 已取消 {arm} 臂执行")
            self._active_handles.clear()
        msg      = Bool()
        msg.data = True
        self._estop_pub.publish(msg)

    def _set_executing(self, arm: str, executing: bool):
        with self._exec_set_lock:
            if executing:
                self._executing_arms.add(arm)
            else:
                self._executing_arms.discard(arm)

    # ─────────────────────── 规划时轨迹碰撞预检 ───────────────────────

    @staticmethod
    def _traj_time(point) -> float:
        return point.time_from_start.sec + point.time_from_start.nanosec * 1e-9

    @staticmethod
    def _interp_joint_traj(jt: JointTrajectory, dt: float = 0.15) -> list:
        """在固定时间间隔 dt 采样 JointTrajectory，返回 list[list[float]]。"""
        pts = jt.points
        if not pts:
            return []
        t_end   = DualArmMotionController._traj_time(pts[-1])
        samples, t = [], 0.0
        while t <= t_end + 1e-6:
            if t <= DualArmMotionController._traj_time(pts[0]):
                pos = list(pts[0].positions)
            else:
                pos = list(pts[-1].positions)
                for i in range(1, len(pts)):
                    t1 = DualArmMotionController._traj_time(pts[i - 1])
                    t2 = DualArmMotionController._traj_time(pts[i])
                    if t <= t2:
                        alpha = (max(0.0, min(1.0, (t - t1) / (t2 - t1)))
                                 if t2 > t1 else 1.0)
                        pos = [p1 + alpha * (p2 - p1)
                               for p1, p2 in zip(pts[i - 1].positions, pts[i].positions)]
                        break
            samples.append(pos)
            t += dt
        return samples

    @staticmethod
    def _slice_joint_traj(jt: JointTrajectory, elapsed_sec: float):
        """截取 elapsed_sec 之后的剩余轨迹段，时间重新从 0 开始。不足 2 点返回 None。"""
        tt = DualArmMotionController._traj_time
        remaining = [pt for pt in jt.points if tt(pt) >= elapsed_sec]
        if len(remaining) < 2:
            return None

        new_jt             = JointTrajectory()
        new_jt.joint_names = list(jt.joint_names)
        for pt in remaining:
            new_pt            = JointTrajectoryPoint()
            new_pt.positions  = list(pt.positions)
            new_pt.velocities = list(pt.velocities) if pt.velocities else []
            new_t             = max(0.0, tt(pt) - elapsed_sec)
            new_pt.time_from_start = RosDuration(
                sec=int(new_t),
                nanosec=int((new_t % 1.0) * 1_000_000_000))
            new_jt.points.append(new_pt)
        return new_jt

    async def _check_trajectory_pair(self,
                                      left_jt: JointTrajectory,
                                      right_jt: JointTrajectory) -> bool:
        """
        时间对齐两条 JointTrajectory，逐帧调用 GetStateValidity 检查双臂合并状态。
        返回 True = 无碰撞，False = 存在碰撞。
        """
        left_pts  = self._interp_joint_traj(left_jt)
        right_pts = self._interp_joint_traj(right_jt)
        if not left_pts or not right_pts:
            return True

        max_len = max(len(left_pts), len(right_pts))
        while len(left_pts)  < max_len: left_pts.append(left_pts[-1])
        while len(right_pts) < max_len: right_pts.append(right_pts[-1])

        for lp, rp in zip(left_pts, right_pts):
            if self._estop:
                return True

            rs             = RobotState()
            js             = JointState()
            js.name        = LEFT_JOINTS + RIGHT_JOINTS
            js.position    = lp + rp
            rs.joint_state = js

            req             = GetStateValidity.Request()
            req.robot_state = rs
            req.group_name  = "both_arms"

            fut  = self._validity_client.call_async(req)
            await fut
            resp = fut.result()
            if resp is None:
                continue
            if not resp.valid:
                contacts = ([c.contact_body_1 + "/" + c.contact_body_2
                             for c in resp.contacts[:3]] if resp.contacts else [])
                self.get_logger().warn(f"[预检] 轨迹碰撞！接触: {contacts}")
                return False

        self.get_logger().info("[预检] 双臂轨迹无碰撞，允许执行")
        return True

    async def _check_against_live(self, arm: str, new_rt) -> bool:
        """
        流水线碰撞预检：将新规划的 RobotTrajectory 与对臂当前正在执行的剩余轨迹对比。

        放行条件：
          1. 对臂当前空闲（无正在执行的轨迹）
          2. 对臂剩余执行时间 < LIVE_CHECK_SKIP_REMAINING（即将结束，执行前必然清场）
          3. FCL 检测无碰撞
        返回 False 时调用方应返回 error_code=-10，Demo 换下一个目标块重试。
        """
        other = "right" if arm == "left" else "left"

        with self._active_traj_lock:
            info = dict(self._active_traj[other])

        if info["jt"] is None:
            self.get_logger().info(f"[{arm}] 对臂空闲，跳过碰撞预检")
            return True

        now_ns  = self.get_clock().now().nanoseconds
        elapsed = (now_ns - info["start_ns"]) * 1e-9

        pts     = info["jt"].points
        dur     = self._traj_time(pts[-1]) if pts else 0.0
        remaining_time = dur - elapsed

        if remaining_time < LIVE_CHECK_SKIP_REMAINING:
            self.get_logger().info(
                f"[{arm}] 对臂剩余 {remaining_time:.2f}s（< {LIVE_CHECK_SKIP_REMAINING}s），视为即将结束，放行")
            return True

        remaining_jt = self._slice_joint_traj(info["jt"], elapsed)
        if remaining_jt is None:
            return True

        self.get_logger().info(
            f"[{arm}] 对臂还有 {remaining_time:.1f}s，与剩余轨迹做碰撞预检...")

        new_jt = new_rt.joint_trajectory
        if arm == "left":
            return await self._check_trajectory_pair(new_jt, remaining_jt)
        else:
            return await self._check_trajectory_pair(remaining_jt, new_jt)

    # ─────────────────────── 规划（plan_only） ───────────────────────

    async def _plan_trajectory(self, arm: str, moveit_goal):
        """向 MoveIt 请求仅规划，返回 (success, RobotTrajectory, error_msg)。"""
        moveit_goal.planning_options.plan_only = True
        client = self._moveit_left if arm == "left" else self._moveit_right
        self.get_logger().info(f"[{arm}] 发送规划请求...")

        send_fut = client.send_goal_async(moveit_goal)
        await send_fut
        handle = send_fut.result()
        if not handle or not handle.accepted:
            return False, None, "MoveIt 规划请求被拒绝"

        res_fut = handle.get_result_async()
        await res_fut
        result  = res_fut.result().result
        if result.error_code.val != 1:
            return False, None, f"MoveIt 规划失败 (code={result.error_code.val})"

        self.get_logger().info(f"[{arm}] 规划成功")
        return True, result.planned_trajectory, ""

    # ─────────────────────── 执行（FollowJointTrajectory） ───────────────────────

    async def _execute_trajectory(self, arm: str, trajectory) -> tuple[bool, str]:
        """执行 RobotTrajectory，同时将其注册为该臂的活跃轨迹供对臂预检使用。"""
        if self._estop:
            return False, "急停激活，拒绝执行"

        client = self._exec_left if arm == "left" else self._exec_right
        goal   = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory.joint_trajectory

        send_fut = client.send_goal_async(goal)
        await send_fut
        handle = send_fut.result()
        if not handle or not handle.accepted:
            return False, "FJT goal 被拒绝"

        with self._handle_lock:
            self._active_handles[arm] = handle

        # 注册当前轨迹，供对臂做实时碰撞预检
        start_ns = self.get_clock().now().nanoseconds
        with self._active_traj_lock:
            self._active_traj[arm] = {
                "jt":       trajectory.joint_trajectory,
                "start_ns": start_ns,
            }

        try:
            res_fut = handle.get_result_async()
            await res_fut
            if self._estop:
                return False, "急停中断执行"
            result = res_fut.result().result
            if result.error_code == 0:
                self.get_logger().info(f"[{arm}] 轨迹执行完成")
                return True, ""
            return False, f"FJT 执行失败 (code={result.error_code})"
        finally:
            with self._handle_lock:
                self._active_handles.pop(arm, None)
            # 执行结束，清除活跃轨迹记录
            with self._active_traj_lock:
                self._active_traj[arm] = {"jt": None, "start_ns": 0}

    async def _plan_and_execute(self, arm: str, moveit_goal) -> tuple[bool, str]:
        ok, traj, err = await self._plan_trajectory(arm, moveit_goal)
        if not ok:
            return False, err
        self._set_executing(arm, True)
        exec_ok, exec_err = await self._execute_trajectory(arm, traj)
        self._set_executing(arm, False)
        return exec_ok, exec_err

    # ─────────────────────── IK + 构造 MoveGroup.Goal ───────────────────────

    def _normalize_angle(self, a):
        while a >  math.pi: a -= 2 * math.pi
        while a < -math.pi: a += 2 * math.pi
        return a

    def _joint_distance(self, positions, arm):
        if self._current_js is None:
            return float("inf")
        names = LEFT_JOINTS if arm == "left" else RIGHT_JOINTS
        d = 0.0
        for i, n in enumerate(names):
            try:
                idx = self._current_js.name.index(n)
                cur = self._current_js.position[idx]
            except (ValueError, IndexError):
                cur = 0.0
            d += self._normalize_angle(positions[i] - cur) ** 2
        return math.sqrt(d)

    async def _compute_ik(self, pose_goal, arm, retry=False, seed=None):
        x, y, z = (pose_goal.pose.position.x,
                   pose_goal.pose.position.y,
                   pose_goal.pose.position.z)
        if math.sqrt(x*x + y*y + z*z) > 0.92:
            return False, None, "目标超出工作空间"

        req = GetPositionIK.Request()
        req.ik_request.group_name   = f"{arm}_arm"
        req.ik_request.pose_stamped = pose_goal
        req.ik_request.timeout.sec  = 2

        if seed is not None:
            js          = JointState()
            js.name     = LEFT_JOINTS if arm == "left" else RIGHT_JOINTS
            js.position = list(seed)
            req.ik_request.robot_state.joint_state = js
        elif self._current_js is not None and not retry:
            req.ik_request.robot_state.joint_state = self._current_js

        fut  = self._ik_client.call_async(req)
        await fut
        resp = fut.result()
        if resp is None or resp.error_code.val != 1:
            code = resp.error_code.val if resp else "None"
            return False, None, f"IK 失败 (code={code})"

        names = LEFT_JOINTS if arm == "left" else RIGHT_JOINTS
        sol   = dict(zip(resp.solution.joint_state.name,
                         resp.solution.joint_state.position))
        pos   = [sol.get(n, 0.0) for n in names]
        dist  = self._joint_distance(pos, arm)

        if dist < 0.1 and not retry:
            self.get_logger().warn(f"[{arm}] IK 解过近 ({dist:.3f} rad)，重试无种子")
            return await self._compute_ik(pose_goal, arm, retry=True, seed=seed)

        self.get_logger().info(f"[{arm}] IK 成功，关节距当前 {dist:.3f} rad")
        return True, pos, ""

    async def _prepare_move_goal(self, arm, x, y, z, yaw_deg=0.0, seed=None):
        """IK + 构造 MoveGroup.Goal，返回 (ok, goal, joint_positions, err)。"""
        pose = PoseStamped()
        pose.header.frame_id       = f"{arm}_base_link"
        pose.header.stamp          = self.get_clock().now().to_msg()
        pose.pose.position.x       = x
        pose.pose.position.y       = y
        pose.pose.position.z       = z
        # q = R_x(π) * R_z(yaw) = (w=0, x=cos(yaw/2), y=-sin(yaw/2), z=0)
        # tool-z 朝下，绕 z 轴旋转 yaw_deg 对齐物体姿态
        th = math.radians(yaw_deg) * 0.5
        pose.pose.orientation.w    = 0.0
        pose.pose.orientation.x    = math.cos(th)
        pose.pose.orientation.y    = -math.sin(th)
        pose.pose.orientation.z    = 0.0

        ok, jpos, err = await self._compute_ik(pose, arm, seed=seed)
        if not ok:
            return False, None, None, err

        names = LEFT_JOINTS if arm == "left" else RIGHT_JOINTS
        g = MoveGroup.Goal()
        g.request.group_name                      = f"{arm}_arm"
        g.request.num_planning_attempts           = 10
        g.request.allowed_planning_time           = 5.0
        g.request.max_velocity_scaling_factor     = 0.1
        g.request.max_acceleration_scaling_factor = 0.1

        c = Constraints()
        for n, v in zip(names, jpos):
            jc                 = JointConstraint()
            jc.joint_name      = n
            jc.position        = v
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight          = 1.0
            c.joint_constraints.append(jc)
        g.request.goal_constraints.append(c)

        return True, g, jpos, ""

    # ─────────────────────── 单步 move ───────────────────────

    async def _execute_move(self, goal_handle, arm, x, y, z):
        if self._estop:
            return False, "急停激活", -1
        if arm not in ("left", "right"):
            return False, f"未知机械臂: {arm}", 10

        self.get_logger().info(f"[{arm}] 移动到 base_link({x:.3f},{y:.3f},{z:.3f})")

        fb = MotionCommand.Feedback()
        fb.current_state = "planning"; fb.progress = 0.2
        goal_handle.publish_feedback(fb)

        ok, moveit_goal, _, err = await self._prepare_move_goal(arm, x, y, z)
        if not ok:
            return False, f"IK 失败: {err}", -4

        fb.current_state = "executing"; fb.progress = 0.5
        goal_handle.publish_feedback(fb)

        exec_ok, exec_err = await self._plan_and_execute(arm, moveit_goal)
        if exec_ok:
            fb.current_state = "completed"; fb.progress = 1.0
            goal_handle.publish_feedback(fb)
            return True, f"[{arm}] 移动成功", 0
        return False, exec_err, -5

    # ─────────────────────── 夹爪 ───────────────────────

    async def _execute_gripper(self, goal_handle, arm, close):
        if arm not in ("left", "right"):
            return False, f"未知机械臂: {arm}", 10

        fb = MotionCommand.Feedback()
        fb.current_state = "executing"; fb.progress = 0.5
        goal_handle.publish_feedback(fb)

        msg      = Float64MultiArray()
        msg.data = [0.8 if close else 0.0]
        pub = self._left_grip_pub if arm == "left" else self._right_grip_pub
        pub.publish(msg)
        await self._async_sleep(1.5)

        fb.current_state = "completed"; fb.progress = 1.0
        goal_handle.publish_feedback(fb)
        return True, f"[{arm}] 夹爪{'闭合' if close else '打开'}完成", 0

    # ─────────────────────── 序列（流水线：实时预检 + 预规划） ───────────────────────

    @staticmethod
    def _next_move_step(steps, start):
        for i in range(start, len(steps)):
            if steps[i].get("type") == "move":
                return i, steps[i]
        return None, None

    async def _execute_sequence(self, goal_handle, arm, steps_json):
        raw   = json.loads(steps_json)
        steps = raw["steps"] if isinstance(raw, dict) else raw

        total = len(steps)
        fb    = MotionCommand.Feedback()
        fb.current_state = "executing"

        precomputed_traj = None
        live_check_done  = False   # 每条序列仅在第一次预规划时做一次实时预检

        for idx, step in enumerate(steps):
            if self._estop:
                return False, "急停激活", -1

            fb.progress = float(idx) / total
            goal_handle.publish_feedback(fb)

            # ── 夹爪步骤 ──
            if step["type"] == "gripper":
                close    = step.get("close", False)
                msg      = Float64MultiArray()
                msg.data = [0.8 if close else 0.0]
                pub = self._left_grip_pub if arm == "left" else self._right_grip_pub
                pub.publish(msg)

                t0        = time.time()
                ni, nstep = self._next_move_step(steps, idx + 1)

                # 利用夹爪等待时间预规划下一段轨迹，并在首次时做实时碰撞预检
                if nstep and precomputed_traj is None:
                    ok, g, _, err = await self._prepare_move_goal(
                        arm, nstep["x"], nstep["y"], nstep["z"],
                        yaw_deg=nstep.get("yaw_deg", 0.0))
                    if ok:
                        plan_ok, traj, perr = await self._plan_trajectory(arm, g)
                        if plan_ok:
                            if not live_check_done:
                                live_check_done = True
                                no_col = await self._check_against_live(arm, traj)
                                if not no_col:
                                    return False, "实时碰撞预检失败，换目标块重试", -10
                            precomputed_traj = traj
                        else:
                            self.get_logger().warn(f"[{arm}] 预规划失败: {perr}")

                elapsed = time.time() - t0
                if elapsed < 1.5:
                    await self._async_sleep(1.5 - elapsed)

            # ── 移动步骤 ──
            elif step["type"] == "move":
                if precomputed_traj is not None:
                    traj             = precomputed_traj
                    precomputed_traj = None
                    self.get_logger().info(f"[{arm}] 步骤 {idx}: 使用预规划轨迹")
                else:
                    ok, g, _, err = await self._prepare_move_goal(
                        arm, step["x"], step["y"], step["z"],
                        yaw_deg=step.get("yaw_deg", 0.0))
                    if not ok:
                        return False, f"IK 失败: {err}", -4
                    plan_ok, traj, perr = await self._plan_trajectory(arm, g)
                    if not plan_ok:
                        return False, f"规划失败: {perr}", -5

                ni, nstep = self._next_move_step(steps, idx + 1)

                self._set_executing(arm, True)
                exec_ok, exec_err = await self._execute_trajectory(arm, traj)
                self._set_executing(arm, False)

                if not exec_ok:
                    return False, exec_err, -6

                # 执行完后预规划下一段，消除下轮规划等待延迟
                if nstep and precomputed_traj is None:
                    ok2, g2, _, _ = await self._prepare_move_goal(
                        arm, nstep["x"], nstep["y"], nstep["z"],
                        yaw_deg=nstep.get("yaw_deg", 0.0))
                    if ok2:
                        plan_ok2, traj2, _ = await self._plan_trajectory(arm, g2)
                        if plan_ok2:
                            precomputed_traj = traj2
                        else:
                            self.get_logger().warn(f"[{arm}] 预规划步骤 {ni} 失败")

            else:
                return False, f"未知步骤类型: {step['type']}", -7

        fb.progress = 1.0
        goal_handle.publish_feedback(fb)
        return True, f"[{arm}] 序列完成 ({total} 步)", 0

    # ─────────────────────── 工具方法 ───────────────────────

    async def _async_sleep(self, duration):
        fut = Future()
        def _cb():
            if not fut.done():
                fut.set_result(None)
        t = self.create_timer(duration, _cb)
        try:
            await fut
        finally:
            t.cancel()
            self.destroy_timer(t)


def main(args=None):
    rclpy.init(args=args)
    controller = DualArmMotionController()
    executor   = MultiThreadedExecutor()
    executor.add_node(controller)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
