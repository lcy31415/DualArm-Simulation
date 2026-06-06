#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class ImageViewer(Node):
    def __init__(self):
        super().__init__("image_viewer")

        self.bridge = CvBridge()

        # 订阅可视化图像话题
        self.image_sub = self.create_subscription(
            Image,
            '/detected_object_viz',
            self._image_callback,
            10
        )

        self.get_logger().info("图像查看器已启动！")
        self.get_logger().info("订阅话题: /detected_object_viz")
        self.get_logger().info("按 'q' 键退出查看器")

    def _image_callback(self, msg):
        try:
            # 将 ROS Image 转换为 OpenCV 格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # 显示图像
            cv2.imshow('Detection Visualization', cv_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"显示图像错误: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    viewer = ImageViewer()

    try:
        rclpy.spin(viewer)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        viewer.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
