#!/usr/bin/env python3
"""
Dual arm debug GUI — two groups of 6 sliders for manual joint control.

Sends Float64MultiArray to /left_arm_controller/commands and /right_arm_controller/commands.
Initial joint angles match the simulation pose.
"""

import threading

import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


ARM_JOINTS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow",
    "wrist_1",
    "wrist_2",
    "wrist_3",
]

# Match simulation initial state
INITIAL_POSE = {
    "shoulder_pan": 0.0,
    "shoulder_lift": -1.57,
    "elbow": 1.57,
    "wrist_1": -1.57,
    "wrist_2": -1.57,
    "wrist_3": 0.0,
}

JOINT_LIMITS = {
    "shoulder_pan": (-6.2832, 6.2832),
    "shoulder_lift": (-6.2832, 6.2832),
    "elbow": (-3.1416, 3.1416),
    "wrist_1": (-6.2832, 6.2832),
    "wrist_2": (-6.2832, 6.2832),
    "wrist_3": (-6.2832, 6.2832),
}


class DualArmDebugGui:
    def __init__(self, node: Node):
        self.node = node

        self.root = tk.Tk()
        self.root.title("Dual UR5 Arm Debug — Manual Joint Control")
        self.root.resizable(True, True)

        main_frame = ttk.Frame(self.root, padding=10)
        main_frame.grid(row=0, column=0, sticky="nsew")

        # Left arm
        left_frame = ttk.LabelFrame(main_frame, text="Left Arm (base at -0.5, 0, 0)", padding=5)
        left_frame.grid(row=0, column=0, padx=10, pady=5, sticky="nsew")
        self.left_values = {}
        self.left_sliders = self._create_sliders(left_frame, "left")

        # Right arm
        right_frame = ttk.LabelFrame(main_frame, text="Right Arm (base at 0.5, 0, 0, rotated 180°)", padding=5)
        right_frame.grid(row=0, column=1, padx=10, pady=5, sticky="nsew")
        self.right_values = {}
        self.right_sliders = self._create_sliders(right_frame, "right")

        # Button
        btn_frame = ttk.Frame(main_frame, padding=5)
        btn_frame.grid(row=1, column=0, columnspan=2, pady=5)
        ttk.Button(btn_frame, text="Reset to Initial Pose", command=self.reset_initial).pack(side=tk.LEFT, padx=5)

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.running = True

    def _create_sliders(self, parent, side: str):
        sliders = {}
        pub = self.node.left_pub if side == "left" else self.node.right_pub

        for idx, jname in enumerate(ARM_JOINTS):
            frame = ttk.Frame(parent, padding=2)
            frame.pack(fill=tk.X, pady=1)

            ttk.Label(frame, text=jname, width=16, anchor=tk.W).pack(side=tk.LEFT)

            initial = INITIAL_POSE[jname]
            lo, hi = JOINT_LIMITS[jname]
            value_var = tk.DoubleVar(value=initial)
            value_label = ttk.Label(frame, text=f"{initial:.3f}", width=8, anchor=tk.E)
            value_label.pack(side=tk.RIGHT, padx=5)

            def make_callback(publisher, sliders_dict, i, lbl):
                def cb(val):
                    rad = float(val)
                    lbl.config(text=f"{rad:.3f}")
                    positions = [sliders_dict[j]["var"].get() for j in ARM_JOINTS]
                    positions[i] = rad
                    msg = Float64MultiArray()
                    msg.data = positions
                    publisher.publish(msg)
                return cb

            scale = ttk.Scale(
                frame, from_=lo, to=hi,
                orient=tk.HORIZONTAL, length=280,
                variable=value_var,
                command=make_callback(pub, sliders, idx, value_label)
            )
            scale.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)

            sliders[jname] = {
                "scale": scale,
                "var": value_var,
                "label": value_label,
            }

        return sliders

    def reset_initial(self):
        for sliders, pub in [(self.left_sliders, self.node.left_pub),
                              (self.right_sliders, self.node.right_pub)]:
            for jname, widgets in sliders.items():
                v = INITIAL_POSE[jname]
                widgets["var"].set(v)
                widgets["label"].config(text=f"{v:.3f}")
            msg = Float64MultiArray()
            msg.data = [INITIAL_POSE[j] for j in ARM_JOINTS]
            pub.publish(msg)

    def on_close(self):
        self.running = False
        self.root.destroy()

    def update(self):
        if self.running:
            try:
                self.root.update()
            except tk.TclError:
                self.running = False


class DebugCommandPublisher(Node):
    def __init__(self):
        super().__init__("dual_arm_debug_gui")
        self.left_pub = self.create_publisher(
            Float64MultiArray, "/left_arm_controller/commands", 10
        )
        self.right_pub = self.create_publisher(
            Float64MultiArray, "/right_arm_controller/commands", 10
        )


def ros_spin(node):
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)


def main():
    rclpy.init()

    pub_node = DebugCommandPublisher()

    gui = DualArmDebugGui(pub_node)

    ros_thread = threading.Thread(target=ros_spin, args=(pub_node,), daemon=True)
    ros_thread.start()

    while gui.running and rclpy.ok():
        gui.update()

    rclpy.shutdown()
    ros_thread.join(timeout=2)


if __name__ == "__main__":
    main()
