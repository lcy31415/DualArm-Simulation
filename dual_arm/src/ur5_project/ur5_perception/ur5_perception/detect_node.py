#!/usr/bin/env python3
"""
detect_node.py — YOLOv8 OBB overhead camera detection

订阅：
  /overhead_camera/rgb/image_raw   (sensor_msgs/Image)
  /overhead_camera/depth/image_raw (sensor_msgs/Image, 可选)

发布：
  /detections/image   (sensor_msgs/Image)   — 标注后的图像
  /detections/results (std_msgs/String)     — JSON 检测结果

单独 OpenCV 窗口实时显示。

坐标变换（相机在世界 (0,0,1.0)，俯视正下方）：
  world_x = -(v - CY) * depth / FY
  world_y = -(u - CX) * depth / FX
  world_z =  1.0 - depth
"""

import os
import sys
import json
import time
import numpy as np

# ─── 路径 ────────────────────────────────────────────────────────────────────
_HERE = os.path.dirname(os.path.abspath(__file__))
# ur5_perception/ur5_perception/ → dual_arm/models/best.pt（上溯4级）
_DEFAULT_MODEL = os.path.normpath(
    os.path.join(_HERE, "../../../../models/best.pt")
)

# ─── 相机内参 ─────────────────────────────────────────────────────────────────
FX = FY = 554.25
CX, CY = 320.0, 240.0
CAM_HEIGHT = 1.0  # 相机在世界坐标系 z = 1.0 m

# ─── 类别定义 ─────────────────────────────────────────────────────────────────
CLASS_NAMES_CN = ["T型块", "正方体", "圆柱体", "足球", "圆环"]
CLASS_NAMES_EN = ["T-block", "Cube", "Cylinder", "Ball", "Ring"]
CLASS_COLORS   = [          # BGR
    (  0, 165, 255),        # T-block  — 橙
    (  0, 200,   0),        # Cube     — 绿
    (255,  80,   0),        # Cylinder — 蓝
    (  0, 220, 220),        # Ball     — 黄
    (200,   0, 200),        # Ring     — 品红
]
OBJ_Z = {0: 0.020, 1: 0.030, 2: 0.040, 3: 0.040, 4: 0.020}  # 质心高度 m

# ─── 机械臂遮罩参数 ───────────────────────────────────────────────────────────
# 从 TF 树读取各连杆世界坐标，投影到图像平面并置黑，防止臂体被误检
ARM_LINKS = [
    # 左臂主体
    "left_shoulder_link",   "left_upper_arm_link",  "left_forearm_link",
    "left_wrist_1_link",    "left_wrist_2_link",    "left_wrist_3_link",
    # 左夹爪（全部连杆）
    "left_robotiq_85_base_link",
    "left_robotiq_85_left_knuckle_link",  "left_robotiq_85_right_knuckle_link",
    "left_robotiq_85_left_finger_link",   "left_robotiq_85_right_finger_link",
    "left_robotiq_85_left_inner_knuckle_link", "left_robotiq_85_right_inner_knuckle_link",
    "left_robotiq_85_left_finger_tip_link",    "left_robotiq_85_right_finger_tip_link",
    # 右臂主体
    "right_shoulder_link",  "right_upper_arm_link", "right_forearm_link",
    "right_wrist_1_link",   "right_wrist_2_link",   "right_wrist_3_link",
    # 右夹爪（全部连杆）
    "right_robotiq_85_base_link",
    "right_robotiq_85_left_knuckle_link",  "right_robotiq_85_right_knuckle_link",
    "right_robotiq_85_left_finger_link",   "right_robotiq_85_right_finger_link",
    "right_robotiq_85_left_inner_knuckle_link", "right_robotiq_85_right_inner_knuckle_link",
    "right_robotiq_85_left_finger_tip_link",    "right_robotiq_85_right_finger_tip_link",
]
ARM_RADIUS_M  = 0.05   # 遮罩物理半径（m），覆盖连杆直径
ARM_RADIUS_MIN = 20    # 最小像素半径
ARM_RADIUS_MAX = 120   # 最大像素半径

# ─── 工作区 ROI（世界坐标） ───────────────────────────────────────────────────
# 工作台 0.6×0.6 m，以世界原点为中心；相机正俯视，桌面 z≈0
# 投影公式（与 _build_arm_mask 一致）：
#   u = CX - wy * FX / depth    （world_y → 像素 u）
#   v = CY - wx * FY / depth    （world_x → 像素 v）
WORKSPACE_X = (-0.30, 0.30)   # 世界 x 轴范围（m）→ 图像 v 方向
WORKSPACE_Y = (-0.30, 0.30)   # 世界 y 轴范围（m）→ 图像 u 方向

# ─── 显示窗口 ─────────────────────────────────────────────────────────────────
WIN = "UR5 Real-time Detection"
WIN_W, WIN_H = 800, 600

# ─── ROS 2 ───────────────────────────────────────────────────────────────────
import rclpy
from rclpy.node import Node
import rclpy.duration
import tf2_ros
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
import cv2


# ─── 轻量 Image 转换（不依赖 cv_bridge，避免 numpy ABI 冲突） ─────────────────
def imgmsg_to_bgr(msg: Image) -> np.ndarray | None:
    """ROS2 Image → BGR uint8 ndarray"""
    enc = msg.encoding.lower()
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    if enc in ("bgr8", "rgb8"):
        img = buf.reshape(msg.height, msg.width, 3)
        return img[:, :, ::-1].copy() if enc == "rgb8" else img.copy()
    if enc == "mono8":
        gray = buf.reshape(msg.height, msg.width)
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    return None


def imgmsg_to_depth(msg: Image) -> np.ndarray | None:
    """ROS2 Image → float32 depth ndarray (单位 m)"""
    enc = msg.encoding.lower()
    if enc == "32fc1":
        return np.frombuffer(msg.data, dtype=np.float32).reshape(
            msg.height, msg.width).copy()
    if enc == "16uc1":
        d16 = np.frombuffer(msg.data, dtype=np.uint16).reshape(
            msg.height, msg.width)
        return (d16 / 1000.0).astype(np.float32)
    return None


def bgr_to_imgmsg(img: np.ndarray, header=None) -> Image:
    """BGR uint8 ndarray → ROS2 Image"""
    msg = Image()
    msg.height, msg.width = img.shape[:2]
    msg.encoding = "bgr8"
    msg.step = int(img.shape[1] * 3)
    msg.data = img.tobytes()
    if header is not None:
        msg.header = header
    return msg


# ─── 坐标变换 ─────────────────────────────────────────────────────────────────
def pixel_to_world(u: float, v: float, depth: float):
    """像素坐标 + 深度 → 世界坐标 (m)"""
    x_opt = (u - CX) * depth / FX
    y_opt = (v - CY) * depth / FY
    return -y_opt, -x_opt, CAM_HEIGHT - depth


# ─── 检测节点 ─────────────────────────────────────────────────────────────────
class DetectNode(Node):
    def __init__(self, model_path: str, auto_start: bool = True):
        super().__init__("ur5_perception")

        # ── 立即打开窗口（在加载模型之前） ──
        self._win_ok = self._open_window("Loading YOLO model...", (0, 200, 255))

        # ── 检查权重 ──
        if not os.path.exists(model_path):
            self.get_logger().error(f"找不到模型权重: {model_path}")
            self.get_logger().error("请先执行训练: bash run_train.sh")
            sys.exit(1)

        # ── 加载模型（窗口已可见，用户可以看到进度） ──
        self.get_logger().info(f"正在加载模型: {model_path} ...")
        from ultralytics import YOLO
        self._model = YOLO(model_path)
        self.get_logger().info("模型加载完毕，开始检测")

        self._latest_depth: np.ndarray | None = None
        self._frame_count  = 0
        self._last_log_t   = 0.0
        self._detection_enabled = auto_start
        self._roi_mask: np.ndarray | None = None   # 首帧计算后缓存

        # ── TF：用于生成机械臂动态遮罩 ──
        self._tf_buf      = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buf, self)

        # ── 订阅 ──
        self.create_subscription(Image, "/overhead_camera/rgb/image_raw",  self._rgb_cb,   10)
        self.create_subscription(Image, "/overhead_camera/depth/image_raw", self._depth_cb, 10)

        # ── 检测使能（TRANSIENT_LOCAL，确保后来的订阅者收到最后一次发布） ──
        if not auto_start:
            _qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.create_subscription(Bool, "/detection_enable", self._enable_cb, _qos)
            self.get_logger().info("检测已暂停，等待 /detection_enable 信号...")

        # ── 发布 ──
        self._pub_img = self.create_publisher(Image,  "/detections/image",   10)
        self._pub_res = self.create_publisher(String, "/detections/results", 10)

        # ── 定时器：未收到图像时刷新窗口 ──
        self.create_timer(0.1, self._timer_cb)

        self.get_logger().info("ur5_perception 已启动，等待相机图像…")

    # ── 工具 ──────────────────────────────────────────────────────────────────
    def _open_window(self, msg: str, color: tuple) -> bool:
        try:
            cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(WIN, WIN_W, WIN_H)
            blank = np.zeros((WIN_H, WIN_W, 3), dtype=np.uint8)
            cv2.putText(blank, msg,
                        (max(0, (WIN_W - len(msg) * 14) // 2), WIN_H // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2, cv2.LINE_AA)
            cv2.imshow(WIN, blank)
            cv2.waitKey(1)
            return True
        except Exception as e:
            print(f"[warn] 无法创建 OpenCV 窗口: {e}", file=sys.stderr)
            return False

    # ── 定时器回调 ────────────────────────────────────────────────────────────
    def _timer_cb(self):
        if self._win_ok and self._frame_count == 0:
            blank = np.zeros((WIN_H, WIN_W, 3), dtype=np.uint8)
            cv2.putText(blank, "Waiting for camera...",
                        (165, WIN_H // 2 - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 200, 100), 2, cv2.LINE_AA)
            cv2.putText(blank, "/overhead_camera/rgb/image_raw",
                        (95, WIN_H // 2 + 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (160, 160, 160), 1, cv2.LINE_AA)
            cv2.imshow(WIN, blank)
        cv2.waitKey(1)

    # ── 机械臂动态遮罩 ────────────────────────────────────────────────────────
    def _build_arm_mask(self, h: int, w: int) -> np.ndarray:
        """
        从 TF 树读取各臂连杆世界坐标，投影到像素平面，返回 uint8 掩码（255=遮罩）。

        世界坐标 → 像素逆变换（相机在 world(0,0,1.0) 俯视）：
          depth = CAM_HEIGHT - wz
          u     = CX - wy * FX / depth
          v     = CY - wx * FY / depth
        """
        mask    = np.zeros((h, w), dtype=np.uint8)
        timeout = rclpy.duration.Duration(seconds=0.02)
        t_query = rclpy.time.Time()   # 最新可用变换

        for link in ARM_LINKS:
            try:
                tf = self._tf_buf.lookup_transform(
                    "world", link, t_query, timeout=timeout)
                wx = tf.transform.translation.x
                wy = tf.transform.translation.y
                wz = tf.transform.translation.z

                depth = CAM_HEIGHT - wz
                if depth < 0.05:          # 连杆在相机平面以上，跳过
                    continue

                u = int(CX - wy * FX / depth)
                v = int(CY - wx * FY / depth)

                # 像素半径随深度缩放（近大远小）
                r = int(FX * ARM_RADIUS_M / depth)
                r = max(ARM_RADIUS_MIN, min(ARM_RADIUS_MAX, r))

                cv2.circle(mask, (u, v), r, 255, -1)
            except Exception:
                pass   # TF 未就绪或连杆不在树中，静默跳过

        return mask

    # ── 工作区 ROI 掩码（首帧计算，之后缓存） ────────────────────────────────
    def _build_roi_mask(self, h: int, w: int) -> np.ndarray:
        """
        把世界坐标工作区矩形投影到像素平面。
        返回掩码：ROI 内 = 255，ROI 外 = 0。
        """
        depth = CAM_HEIGHT          # 桌面 z≈0，depth = 1.0 - 0 = 1.0 m
        x_min, x_max = WORKSPACE_X
        y_min, y_max = WORKSPACE_Y
        # u = CX - wy*FX/depth → y_max → u 最小（左边），y_min → u 最大（右边）
        u_left  = max(0, int(CX - y_max * FX / depth))
        u_right = min(w, int(CX - y_min * FX / depth))
        # v = CY - wx*FY/depth → x_max → v 最小（上边），x_min → v 最大（下边）
        v_top   = max(0, int(CY - x_max * FY / depth))
        v_bot   = min(h, int(CY - x_min * FY / depth))
        mask = np.zeros((h, w), dtype=np.uint8)
        mask[v_top:v_bot, u_left:u_right] = 255
        return mask

    # ── 检测使能回调 ──────────────────────────────────────────────────────────
    def _enable_cb(self, msg: Bool):
        if msg.data and not self._detection_enabled:
            self._detection_enabled = True
            self.get_logger().info("收到启用信号，开始 YOLOv8 检测！")

    # ── 深度回调 ──────────────────────────────────────────────────────────────
    def _depth_cb(self, msg: Image):
        d = imgmsg_to_depth(msg)
        if d is not None:
            self._latest_depth = d

    # ── RGB 主回调 ────────────────────────────────────────────────────────────
    def _rgb_cb(self, msg: Image):
        frame = imgmsg_to_bgr(msg)
        if frame is None:
            self.get_logger().warn(f"不支持的图像格式: {msg.encoding}")
            return

        self._frame_count += 1

        # ── 检测未启用：显示原始画面 + 等待提示 ──────────────────────────────
        if not self._detection_enabled:
            if self._win_ok:
                display = frame.copy()
                cv2.rectangle(display, (0, 0), (display.shape[1], 40), (0, 0, 0), -1)
                cv2.putText(display, "等待机械臂就绪...",
                            (10, 28), cv2.FONT_HERSHEY_SIMPLEX,
                            0.85, (0, 200, 255), 2, cv2.LINE_AA)
                cv2.imshow(WIN, display)
                cv2.waitKey(1)
            return

        # ── 构建推理帧：ROI 外置黑 + 臂体置黑 ───────────────────────────────
        h, w = frame.shape[:2]

        # ROI 掩码（首帧计算后缓存，工作区以外全部置黑）
        if self._roi_mask is None or self._roi_mask.shape != (h, w):
            self._roi_mask = self._build_roi_mask(h, w)

        arm_mask = self._build_arm_mask(h, w)
        infer_frame = frame.copy()
        infer_frame[self._roi_mask == 0] = 0   # ROI 外置黑
        if arm_mask.any():
            infer_frame[arm_mask > 0] = 0       # 臂体区域置黑

        results  = self._model.predict(infer_frame, conf=0.40, verbose=False)
        display  = frame.copy()

        # ROI 外半透明暗化 + 青色边框（调试可见）
        dark = display.copy()
        dark[self._roi_mask == 0] = (20, 20, 20)
        cv2.addWeighted(dark, 0.55, display, 0.45, 0, display)
        roi_cnts, _ = cv2.findContours(
            self._roi_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(display, roi_cnts, -1, (0, 255, 255), 2)

        # 臂体遮罩半透明深色 + 灰色轮廓
        if arm_mask.any():
            dark2 = display.copy()
            dark2[arm_mask > 0] = (30, 30, 30)
            cv2.addWeighted(dark2, 0.45, display, 0.55, 0, display)
            arm_cnts, _ = cv2.findContours(
                arm_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(display, arm_cnts, -1, (180, 180, 180), 1)

        detects  = []

        if results and results[0].obb is not None and len(results[0].obb):
            obb      = results[0].obb
            corners  = obb.xyxyxyxy.cpu().numpy()   # (N, 4, 2)
            cls_ids  = obb.cls.cpu().numpy().astype(int)
            confs    = obb.conf.cpu().numpy()

            for i in range(len(cls_ids)):
                cid  = int(cls_ids[i])
                conf = float(confs[i])
                pts  = corners[i]                   # (4, 2)

                cu   = float(pts[:, 0].mean())
                cv_  = float(pts[:, 1].mean())
                d    = self._get_depth(cu, cv_, cid)
                wx, wy, wz = pixel_to_world(cu, cv_, d)

                # ROI 世界坐标兜底过滤（防止边缘误检漏过掩码）
                if not (WORKSPACE_X[0] <= wx <= WORKSPACE_X[1] and
                        WORKSPACE_Y[0] <= wy <= WORKSPACE_Y[1]):
                    continue

                # ── OBB 主轴角度 ──────────────────────────────────────────────
                # 取两条邻边，选较长者作为主轴（对非正方形物体更稳定）
                e1 = pts[1] - pts[0]
                e2 = pts[3] - pts[0]
                axis_img = e1 if np.linalg.norm(e1) >= np.linalg.norm(e2) else e2
                alen = np.linalg.norm(axis_img)
                if alen > 1e-6:
                    axis_img = axis_img / alen
                else:
                    axis_img = np.array([1.0, 0.0])

                # 世界坐标角（与世界 x 轴正方向夹角）
                # 相机变换: Δworld_x = -Δv, Δworld_y = -Δu
                world_ang = np.degrees(
                    np.arctan2(-axis_img[0], -axis_img[1])  # arctan2(Δwy, Δwx)
                )
                # 归一化到 (-90°, 90°]（OBB 主轴 180° 对称）
                if world_ang > 90.0:
                    world_ang -= 180.0
                elif world_ang <= -90.0:
                    world_ang += 180.0

                col     = CLASS_COLORS[cid] if cid < len(CLASS_COLORS) else (255, 255, 255)
                name_en = CLASS_NAMES_EN[cid] if cid < len(CLASS_NAMES_EN) else str(cid)
                name_cn = CLASS_NAMES_CN[cid] if cid < len(CLASS_NAMES_CN) else str(cid)

                detects.append({
                    "class":      name_cn,
                    "class_id":   cid,
                    "confidence": round(conf, 3),
                    "xyz":        [round(wx, 3), round(wy, 3), round(wz, 3)],
                    "angle_deg":  round(float(world_ang), 1),
                })

                # ── 绘制 OBB ──────────────────────────────────────────────────
                overlay = display.copy()
                cv2.fillPoly(overlay, [pts.astype(int)], col)
                cv2.addWeighted(overlay, 0.25, display, 0.75, 0, display)
                cv2.polylines(display, [pts.astype(int)], True, col, 2, cv2.LINE_AA)

                # ── 质心点 ────────────────────────────────────────────────────
                cpt = (int(cu), int(cv_))
                cv2.circle(display, cpt, 5, (255, 255, 255), -1)   # 白色实心
                cv2.circle(display, cpt, 5, col, 1)                 # 彩色轮廓

                # ── 方向箭头（沿主轴，长 40px） ──────────────────────────────
                ARROW = 40
                tip = (int(cu + axis_img[0] * ARROW),
                       int(cv_ + axis_img[1] * ARROW))
                cv2.arrowedLine(display, cpt, tip,
                                (255, 255, 255), 2, cv2.LINE_AA, tipLength=0.35)

                # ── 文字标签 ──────────────────────────────────────────────────
                lx, ly = int(cu), int(cv_) - 28
                cv2.putText(display, f"{name_en} {conf:.2f}",
                            (lx, ly), cv2.FONT_HERSHEY_SIMPLEX,
                            0.55, col, 2, cv2.LINE_AA)
                cv2.putText(display, f"({wx:.3f},{wy:.3f},{wz:.3f})",
                            (lx, ly + 18), cv2.FONT_HERSHEY_SIMPLEX,
                            0.42, (255, 255, 255), 1, cv2.LINE_AA)
                cv2.putText(display, f"{world_ang:+.1f}deg",
                            (lx, ly + 34), cv2.FONT_HERSHEY_SIMPLEX,
                            0.42, (200, 200, 0), 1, cv2.LINE_AA)

        if self._win_ok:
            cv2.imshow(WIN, display)

        # ── 终端输出（每秒一次） ──
        now = time.time()
        if now - self._last_log_t >= 1.0:
            self._last_log_t = now
            self._log_detections(detects)

        try:
            self._pub_img.publish(bgr_to_imgmsg(display, msg.header))
        except Exception:
            pass

        s = String()
        s.data = json.dumps(detects, ensure_ascii=False)
        self._pub_res.publish(s)

    # ── 终端打印 ──────────────────────────────────────────────────────────────
    def _log_detections(self, detects: list):
        SEP = "─" * 66
        if not detects:
            print(f"\n{SEP}")
            print("  未检测到物体")
            print(SEP, flush=True)
            return

        print(f"\n{SEP}")
        print(f"  检测到 {len(detects)} 个物体  [Gazebo 世界坐标 / m，角度相对世界 x 轴]")
        print(SEP)
        print(f"  {'类别':<8} {'x':>8} {'y':>8} {'z':>8}   {'角度(°)':>8}   conf")
        print(f"  {'─'*7} {'─'*8} {'─'*8} {'─'*8}   {'─'*8}   {'─'*5}")
        for d in detects:
            wx, wy, wz = d["xyz"]
            ang = d.get("angle_deg", 0.0)
            print(f"  {d['class']:<8} {wx:+8.3f} {wy:+8.3f} {wz:+8.3f}   {ang:+8.1f}   {d['confidence']:.2f}")
        print(SEP, flush=True)

    # ── 深度读取（带回退） ─────────────────────────────────────────────────────
    def _get_depth(self, u: float, v: float, cls_id: int) -> float:
        if self._latest_depth is not None:
            h, w = self._latest_depth.shape
            iu, iv = int(round(u)), int(round(v))
            if 0 <= iu < w and 0 <= iv < h:
                d = float(self._latest_depth[iv, iu])
                if 0.05 < d < 1.45:
                    return d
        return CAM_HEIGHT - OBJ_Z.get(cls_id, 0.0)

    def destroy_node(self):
        if self._win_ok:
            cv2.destroyAllWindows()
        super().destroy_node()


# ─── 入口 ─────────────────────────────────────────────────────────────────────
def main(args=None):
    import argparse
    p = argparse.ArgumentParser(description="UR5 YOLOv8 OBB 检测节点")
    p.add_argument("--model", default=_DEFAULT_MODEL, help="YOLOv8 OBB 权重路径")
    p.add_argument("--auto-start", default="true",
                   help="true=加载后立即检测; false=等待 /detection_enable 信号")
    parsed, ros_args = p.parse_known_args()

    auto_start = parsed.auto_start.lower() != "false"

    rclpy.init(args=ros_args)
    node = DetectNode(parsed.model, auto_start=auto_start)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
