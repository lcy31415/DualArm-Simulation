#!/usr/bin/env python3
"""
sorting_demo_1.py — 双臂自动分拣

流程：
  1. 归位：左右臂并发移动到启动位置，完成后发布 /detection_enable=True
  2. 分拣：订阅 /detections/results，按物品类别分配机械臂并发执行序列
     - Cube / Ball   → 右臂
     - Cylinder / T-block → 左臂

分拣序列（{arm}_base_link 坐标）：
  open → approach(x1,y1,z1+L1,yaw) → grasp(x1,y1,z1+L1-L2,yaw)
       → close → carry(x2,y2,z1+0.28,yaw) → release

坐标变换（世界 → {arm}_base_link）：
  left : (wx+0.6,  wy,  wz)
  right: (0.6-wx, -wy,  wz)

分拣目标（世界坐标）：
  Cube     → 绿色区 C(+0.3, -0.5)
  Cylinder → 蓝色区 B(-0.3, +0.5)
  T-block  → 黄色区 D(-0.3, -0.5)
  Ball     → 红色区 A(+0.3, +0.5)
"""

import json
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from std_msgs.msg import Bool, String
from ur5_interfaces.action import MotionCommand

# ── 启动位置（{arm}_base_link 局部坐标） ─────────────────────────────────────────
# 世界坐标: left(-0.35,0,0.4)  right(+0.35,0,0.4)
HOME_X, HOME_Y, HOME_Z = 0.25, 0.0, 0.4

# ── 分拣参数 ────────────────────────────────────────────────────────────────────
L1 = 0.20    # 接近高度偏移：tool0 在 z1+0.20，指尖在 z1+0.07
L2 = 0.075  # 下降深度：tool0 在 z1+0.125，指尖到达 z1（夹爪长 0.13 m）

# ── 分拣目标（世界坐标 x,y） ───────────────────────────────────────────────────
SORT_TARGETS = {
    "Cube":     ( 0.3, -0.5),   # 绿色区 C
    "Cylinder": (-0.3,  0.5),   # 蓝色区 B
    "T-block":  (-0.3, -0.5),   # 黄色区 D
    "Ball":     ( 0.3,  0.5),   # 红色区 A
}

# ── 物品 → 机械臂分配（基于区域可达性） ─────────────────────────────────────────
CLASS_TO_ARM = {
    "Cube":     "right",
    "Cylinder": "left",
    "T-block":  "left",
    "Ball":     "right",
}

# ── 夹爪角度补偿（OBB 主轴为长轴，部分物体需要转 90° 改为垂直长轴夹取） ────────
GRASP_ANGLE_OFFSET = {
    "T-block": 90.0,   # 沿短轴张开夹爪，垂直 T 形长边
}

# ── 检测节点输出的中文类名 → 英文 ─────────────────────────────────────────────
CLASS_CN_TO_EN = {
    "T型块":  "T-block",
    "正方体": "Cube",
    "圆柱体": "Cylinder",
    "足球":   "Ball",
    "圆环":   "Ring",
}


def world_to_arm(arm: str, wx: float, wy: float, wz: float) -> tuple:
    """世界坐标 → {arm}_base_link 局部坐标"""
    if arm == "left":
        return wx + 0.6, wy, wz
    else:
        return 0.6 - wx, -wy, wz


def build_sort_sequence(arm: str,
                        x1: float, y1: float, z1: float,
                        angle_deg: float,
                        x2: float, y2: float) -> str:
    """
    构建分拣序列 JSON（坐标已转换为 {arm}_base_link）。

    步骤：
      1. 打开夹爪
      2. 移到接近点 (x1,y1,z1+L1)
      3. 下降到抓取点 (x1,y1,z1+L1-L2)  → 指尖到 z1
      4. 闭合夹爪
      5. 移到搬运点 (x2,y2,z1+0.28)     → 指尖在桌面以上 ~15 cm
      6. 打开夹爪
    """
    ax, ay, az_a = world_to_arm(arm, x1, y1, z1 + L1)
    _,  _,  az_g = world_to_arm(arm, x1, y1, z1 + L1 - L2)
    dx, dy, dz_c = world_to_arm(arm, x2, y2, z1 + 0.15)

    steps = [
        {"type": "gripper", "close": False},
        {"type": "move", "x": ax,  "y": ay,  "z": az_a, "yaw_deg": angle_deg},
        {"type": "move", "x": ax,  "y": ay,  "z": az_g, "yaw_deg": angle_deg},
        {"type": "gripper", "close": True},
        {"type": "move", "x": dx,  "y": dy,  "z": dz_c, "yaw_deg": angle_deg},
        {"type": "gripper", "close": False},
    ]
    return json.dumps({"steps": steps})


class SortingDemoNode(Node):
    def __init__(self):
        super().__init__("ur5_sorting_demo")

        cb = ReentrantCallbackGroup()

        self._client = ActionClient(
            self, MotionCommand, "/motion_command", callback_group=cb)

        # TRANSIENT_LOCAL：detect_node 迟到时仍能收到最后一次消息
        _latch_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._enable_pub = self.create_publisher(Bool, "/detection_enable", _latch_qos)

        # 臂状态
        self._arm_busy: dict[str, bool] = {"left": False, "right": False}
        self._pending_home: set[str]    = set()
        self._homed   = False
        self._started = False

        self._init_timer = self.create_timer(0.5, self._check_server, callback_group=cb)
        self.get_logger().info("等待 /motion_command server...")

    # ── 归位流程 ──────────────────────────────────────────────────────────────

    def _check_server(self):
        if self._started:
            return
        if not self._client.server_is_ready():
            return
        self._started = True
        self._init_timer.cancel()
        self.get_logger().info("server 就绪，发送双臂归位指令...")
        self._send_home("left")
        self._send_home("right")

    def _send_home(self, arm: str):
        goal = MotionCommand.Goal()
        goal.command_type      = "move"
        goal.arm               = arm
        goal.target_position.x = HOME_X
        goal.target_position.y = HOME_Y
        goal.target_position.z = HOME_Z

        self._pending_home.add(arm)
        self.get_logger().info(
            f"[{arm}] 归位 → base_link({HOME_X},{HOME_Y},{HOME_Z})")

        fut = self._client.send_goal_async(goal)
        fut.add_done_callback(lambda f, a=arm: self._on_home_accepted(f, a))

    def _on_home_accepted(self, future, arm: str):
        handle = future.result()
        if not handle or not handle.accepted:
            self.get_logger().error(f"[{arm}] 归位 goal 被拒绝")
            self._on_home_done(arm)
            return
        res_fut = handle.get_result_async()
        res_fut.add_done_callback(lambda f, a=arm: self._on_home_result(f, a))

    def _on_home_result(self, future, arm: str):
        result = future.result().result
        if result.success:
            self.get_logger().info(f"[{arm}] 归位完成")
        else:
            self.get_logger().warn(f"[{arm}] 归位失败: {result.message}")
        self._on_home_done(arm)

    def _on_home_done(self, arm: str):
        self._pending_home.discard(arm)
        if self._pending_home:
            return  # 另一臂还未完成

        self.get_logger().info("✓ 双臂已就位，开启检测与分拣")
        self._homed = True

        msg = Bool()
        msg.data = True
        self._enable_pub.publish(msg)

        cb = ReentrantCallbackGroup()
        self.create_subscription(
            String, "/detections/results", self._on_detections, 10,
            callback_group=cb)

    # ── 分拣回调 ──────────────────────────────────────────────────────────────

    def _on_detections(self, msg: String):
        if not self._homed:
            return

        try:
            detects = json.loads(msg.data)
        except Exception:
            return

        if not detects:
            return

        for det in detects:
            class_cn = det.get("class", "")
            class_en = CLASS_CN_TO_EN.get(class_cn, class_cn)

            if class_en not in CLASS_TO_ARM:
                continue

            arm = CLASS_TO_ARM[class_en]
            if self._arm_busy[arm]:
                continue

            target = SORT_TARGETS.get(class_en)
            if target is None:
                continue

            xyz       = det.get("xyz", [0.0, 0.0, 0.0])
            angle_deg = float(det.get("angle_deg", 0.0))
            angle_deg += GRASP_ANGLE_OFFSET.get(class_en, 0.0)
            x1, y1, z1 = float(xyz[0]), float(xyz[1]), float(xyz[2])
            x2, y2     = float(target[0]), float(target[1])

            seq_json = build_sort_sequence(arm, x1, y1, z1, angle_deg, x2, y2)

            self._arm_busy[arm] = True
            self.get_logger().info(
                f"[{arm}] 分拣 {class_en} "
                f"({x1:.3f},{y1:.3f},{z1:.3f}) angle={angle_deg:.1f}° "
                f"→ ({x2},{y2})")

            goal = MotionCommand.Goal()
            goal.command_type  = "sequence"
            goal.arm           = arm
            goal.sequence_data = seq_json

            fut = self._client.send_goal_async(goal)
            fut.add_done_callback(
                lambda f, a=arm, c=class_en: self._on_sort_accepted(f, a, c))

    def _on_sort_accepted(self, future, arm: str, cls: str):
        handle = future.result()
        if not handle or not handle.accepted:
            self.get_logger().error(f"[{arm}] 分拣 goal 被拒绝")
            self._arm_busy[arm] = False
            return
        res_fut = handle.get_result_async()
        res_fut.add_done_callback(
            lambda f, a=arm, c=cls: self._on_sort_result(f, a, c))

    def _on_sort_result(self, future, arm: str, cls: str):
        result = future.result().result
        if result.success:
            self.get_logger().info(f"[{arm}] {cls} 分拣完成")
        else:
            self.get_logger().warn(
                f"[{arm}] {cls} 分拣失败 "
                f"(code={result.error_code}): {result.message}")
        self._arm_busy[arm] = False


def main(args=None):
    rclpy.init(args=args)
    node = SortingDemoNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
