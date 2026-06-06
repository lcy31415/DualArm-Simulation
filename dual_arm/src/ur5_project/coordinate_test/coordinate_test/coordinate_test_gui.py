#!/usr/bin/env python3
"""
双臂坐标测试 GUI
两个输入区域分别控制左臂和右臂，输入 Gazebo 世界坐标，驱动末端移动
"""
import threading
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Point
from ur5_interfaces.action import MotionCommand


class CoordinateTestGui:
    def __init__(self, node: Node):
        self.node = node
        self.root = tk.Tk()
        self.root.title("Dual Arm Coordinate Test")
        self.root.resizable(False, False)

        main = ttk.Frame(self.root, padding=10)
        main.grid(row=0, column=0)

        # ==== Left Arm ====
        left_frame = ttk.LabelFrame(main, text="Left Arm (世界坐标, base 在 -0.6,0,0)", padding=8)
        left_frame.grid(row=0, column=0, padx=10, pady=5, sticky="n")

        self.left_xyz = {}
        for i, axis in enumerate("XYZ"):
            ttk.Label(left_frame, text=f"{axis}:").grid(row=i, column=0, sticky="e", padx=3, pady=2)
            var = tk.StringVar(value="0.0")
            ttk.Entry(left_frame, textvariable=var, width=10).grid(row=i, column=1, padx=3, pady=2)
            self.left_xyz[axis.lower()] = var

        btn_frame_l = ttk.Frame(left_frame)
        btn_frame_l.grid(row=3, column=0, columnspan=2, pady=5)
        ttk.Button(btn_frame_l, text="Move Left", command=self.move_left).pack(side=tk.LEFT, padx=3)

        gripper_frame_l = ttk.Frame(left_frame)
        gripper_frame_l.grid(row=4, column=0, columnspan=2, pady=3)
        ttk.Button(gripper_frame_l, text="Gripper Open", command=self.gripper_left_open).pack(side=tk.LEFT, padx=3)
        ttk.Button(gripper_frame_l, text="Gripper Close", command=self.gripper_left_close).pack(side=tk.LEFT, padx=3)

        # ==== Right Arm ====
        right_frame = ttk.LabelFrame(main, text="Right Arm (世界坐标, base 在 0.6,0,0)", padding=8)
        right_frame.grid(row=0, column=1, padx=10, pady=5, sticky="n")

        self.right_xyz = {}
        for i, axis in enumerate("XYZ"):
            ttk.Label(right_frame, text=f"{axis}:").grid(row=i, column=0, sticky="e", padx=3, pady=2)
            var = tk.StringVar(value="0.0")
            ttk.Entry(right_frame, textvariable=var, width=10).grid(row=i, column=1, padx=3, pady=2)
            self.right_xyz[axis.lower()] = var

        btn_frame_r = ttk.Frame(right_frame)
        btn_frame_r.grid(row=3, column=0, columnspan=2, pady=5)
        ttk.Button(btn_frame_r, text="Move Right", command=self.move_right).pack(side=tk.LEFT, padx=3)

        gripper_frame_r = ttk.Frame(right_frame)
        gripper_frame_r.grid(row=4, column=0, columnspan=2, pady=3)
        ttk.Button(gripper_frame_r, text="Gripper Open", command=self.gripper_right_open).pack(side=tk.LEFT, padx=3)
        ttk.Button(gripper_frame_r, text="Gripper Close", command=self.gripper_right_close).pack(side=tk.LEFT, padx=3)

        # ==== Combined buttons ====
        combo_frame = ttk.Frame(main)
        combo_frame.grid(row=1, column=0, columnspan=2, pady=10)
        ttk.Button(combo_frame, text="Move Both", command=self.move_both).pack(side=tk.LEFT, padx=10)

        # ==== Status ====
        self.status_var = tk.StringVar(value="就绪")
        ttk.Label(main, textvariable=self.status_var, relief=tk.SUNKEN, anchor=tk.W, padding=5).grid(
            row=2, column=0, columnspan=2, sticky="we", pady=5
        )

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.running = True

    def _parse(self, arm):
        xyz = self.left_xyz if arm == "left" else self.right_xyz
        try:
            return (float(xyz["x"].get()), float(xyz["y"].get()), float(xyz["z"].get()))
        except ValueError:
            return None

    def _send_move(self, arm):
        coords = self._parse(arm)
        if coords is None:
            self.status_var.set(f"[{arm}] 请输入有效数字")
            return
        x, y, z = coords
        if arm == "left":
            bx, by, bz = x + 0.6, y, z
        else:
            bx, by, bz = 0.6 - x, -y, z
        self.status_var.set(f"[{arm}] 世界({x:.2f},{y:.2f},{z:.2f}) → base_link({bx:.2f},{by:.2f},{bz:.2f})")
        self.node.send_move(arm, bx, by, bz)

    def _send_gripper(self, arm, close):
        action = "闭合" if close else "打开"
        self.status_var.set(f"[{arm}] 夹爪{action}...")
        self.node.send_gripper(arm, close)

    def move_left(self):
        self._send_move("left")

    def move_right(self):
        self._send_move("right")

    def move_both(self):
        self.move_left()
        self.root.after(200, self.move_right)

    def gripper_left_open(self):
        self._send_gripper("left", False)

    def gripper_left_close(self):
        self._send_gripper("left", True)

    def gripper_right_open(self):
        self._send_gripper("right", False)

    def gripper_right_close(self):
        self._send_gripper("right", True)

    def on_close(self):
        self.running = False
        self.root.destroy()

    def update(self):
        if self.running:
            try:
                self.root.update()
            except tk.TclError:
                self.running = False


class CoordinateTestNode(Node):
    def __init__(self):
        super().__init__("coordinate_test")
        self._action_client = ActionClient(self, MotionCommand, "/motion_command")
        self.get_logger().info("等待 MotionCommand Action 服务器...")
        self._action_client.wait_for_server()
        self.get_logger().info("coordinate_test 节点已启动！")

    def send_move(self, arm, x, y, z):
        goal = MotionCommand.Goal()
        goal.command_type = "move"
        goal.arm = arm
        goal.target_position = Point(x=x, y=y, z=z)

        self._action_client.send_goal_async(goal).add_done_callback(
            lambda f: self._goal_response(f, arm, f"move({x:.2f},{y:.2f},{z:.2f})")
        )

    def send_gripper(self, arm, close):
        goal = MotionCommand.Goal()
        goal.command_type = "gripper"
        goal.arm = arm
        goal.gripper_close = close

        action = "close" if close else "open"
        self._action_client.send_goal_async(goal).add_done_callback(
            lambda f: self._goal_response(f, arm, f"gripper_{action}")
        )

    def _goal_response(self, future, arm, desc):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f"[{arm}] {desc}: 被拒绝")
            return
        self.get_logger().info(f"[{arm}] {desc}: 已接受，等待执行...")
        goal_handle.get_result_async().add_done_callback(
            lambda f: self._result_callback(f, arm, desc)
        )

    def _result_callback(self, future, arm, desc):
        result = future.result().result
        if result.success:
            self.get_logger().info(f"[{arm}] {desc}: 成功")
        else:
            self.get_logger().error(f"[{arm}] {desc}: 失败 — {result.message}")


def ros_spin(node):
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)


def main(args=None):
    rclpy.init(args=args)
    ros_node = CoordinateTestNode()
    gui = CoordinateTestGui(ros_node)

    ros_thread = threading.Thread(target=ros_spin, args=(ros_node,), daemon=True)
    ros_thread.start()

    while gui.running and rclpy.ok():
        gui.update()

    rclpy.shutdown()
    ros_thread.join(timeout=2)


if __name__ == "__main__":
    main()
