#!/usr/bin/env python3
"""
Publish a persistent CameraInfo message so RViz Camera display works
and perception nodes always receive camera intrinsics.

Gazebo Harmonic camera sensors do not auto-publish camera_info on a
separate topic that ros_gz_bridge can forward, so we provide it here.

Runs as a persistent node (not one-shot) so TRANSIENT_LOCAL survives.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import CameraInfo


class CameraInfoPublisher(Node):
    def __init__(self):
        super().__init__('overhead_camera_info_publisher')

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub = self.create_publisher(CameraInfo, '/overhead_camera/rgb/camera_info', qos)

        msg = CameraInfo()
        msg.header.frame_id = 'overhead_camera_optical_frame'
        msg.width = 640
        msg.height = 480
        msg.k = [554.25, 0.0, 320.0, 0.0, 554.25, 240.0, 0.0, 0.0, 1.0]
        msg.p = [554.25, 0.0, 320.0, 0.0, 0.0, 554.25, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        msg.distortion_model = 'plumb_bob'

        self.pub.publish(msg)
        self.get_logger().info('CameraInfo published (persistent, TRANSIENT_LOCAL)')

        # 每 5 秒重发一次，确保新订阅者一定能收到
        self.timer = self.create_timer(5.0, lambda: self.pub.publish(msg))


def main():
    rclpy.init()
    node = CameraInfoPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
