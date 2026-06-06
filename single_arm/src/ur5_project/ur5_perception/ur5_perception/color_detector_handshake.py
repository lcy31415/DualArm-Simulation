#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, PointStamped
from ur5_interfaces.msg import DetectedObject, DetectionBatch
from ur5_interfaces.srv import DetectionControl
from cv_bridge import CvBridge
import cv2
import numpy as np
import message_filters
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs


class ColorDetector(Node):
    def __init__(self):
        super().__init__("color_detector")

        # 创建 CV Bridge
        self.bridge = CvBridge()

        # 相机内参矩阵
        self.camera_matrix = None
        self.image_width = None
        self.image_height = None

        # TF2 坐标变换
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.target_frame = 'base_link'
        self.camera_frame = 'camera_optical_frame'
        self.base_offset_x = 0.0

        # 检测控制状态
        self.detection_enabled = True  # 是否启用检测
        self.detection_history = []    # 检测历史记录
        self.stable_threshold = 5      # 连续稳定帧数
        self.z_offset = 0.1            # z轴抓取偏移

        # 创建检测控制服务
        self.detection_control_srv = self.create_service(
            DetectionControl,
            '/detection_control',
            self._detection_control_callback
        )

        # 订阅相机信息
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/rgb/camera_info',
            self._camera_info_callback,
            10
        )

        # 使用 message_filters 同步图像
        self.rgb_sub = message_filters.Subscriber(self, Image, '/camera/rgb/image_raw')
        self.depth_sub = message_filters.Subscriber(self, Image, '/camera/depth/image_raw')
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], queue_size=10, slop=0.1
        )
        self.ts.registerCallback(self._synchronized_callback)

        # 发布批量检测结果
        self.batch_pub = self.create_publisher(DetectionBatch, '/detection_batch', 10)

        # 发布可视化图像
        self.viz_pub = self.create_publisher(Image, '/detected_object_viz', 10)

        # HSV 颜色范围
        self.color_ranges = {
            'red': [
                {'lower': np.array([0, 120, 70]), 'upper': np.array([10, 255, 255]), 'color_bgr': (0, 0, 255), 'name': 'Red'},
                {'lower': np.array([170, 120, 70]), 'upper': np.array([180, 255, 255]), 'color_bgr': (0, 0, 255), 'name': 'Red'}
            ],
            'green': [{'lower': np.array([40, 40, 40]), 'upper': np.array([80, 255, 255]), 'color_bgr': (0, 255, 0), 'name': 'Green'}],
            'blue': [{'lower': np.array([100, 100, 70]), 'upper': np.array([130, 255, 255]), 'color_bgr': (255, 0, 0), 'name': 'Blue'}]
        }

        self.declare_parameter('min_contour_area', 300)
        self.declare_parameter('enable_visualization', True)

        self.get_logger().info("颜色检测节点已启动(握手通讯模式)！")
        self.get_logger().info("服务: /detection_control (DetectionControl)")
        self.get_logger().info("发布: /detection_batch (DetectionBatch)")

    def _detection_control_callback(self, request, response):
        """
        检测控制服务回调
        """
        self.detection_enabled = request.enable
        if request.enable:
            self.detection_history.clear()
            self.get_logger().info("✓ 检测已启用")
            response.message = "Detection enabled"
        else:
            self.get_logger().info("✗ 检测已停止")
            response.message = "Detection disabled"
        response.success = True
        return response

    def _camera_info_callback(self, msg):
        if self.camera_matrix is None:
            K = np.array(msg.k).reshape(3, 3)
            self.camera_matrix = K
            self.image_width = msg.width
            self.image_height = msg.height
            self.get_logger().info(f"相机内参: fx={K[0,0]:.2f}, fy={K[1,1]:.2f}")

    def _pixel_to_3d(self, cx, cy, depth_value):
        if self.camera_matrix is None or depth_value <= 0:
            return None

        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx_cam = self.camera_matrix[0, 2]
        cy_cam = self.camera_matrix[1, 2]

        X_cam = (cx - cx_cam) * depth_value / fx
        Y_cam = (cy - cy_cam) * depth_value / fy
        Z_cam = depth_value

        try:
            point_cam = PointStamped()
            point_cam.header.frame_id = self.camera_frame
            point_cam.header.stamp = self.get_clock().now().to_msg()
            point_cam.point.x = X_cam
            point_cam.point.y = Y_cam
            point_cam.point.z = Z_cam

            point_base = self.tf_buffer.transform(point_cam, self.target_frame, timeout=rclpy.duration.Duration(seconds=0.1))

            X_actual = point_base.point.x - self.base_offset_x
            Y_actual = point_base.point.y
            Z_actual = point_base.point.z + self.z_offset  # 增加 z 轴偏移

            return (X_actual, Y_actual, Z_actual)
        except Exception as e:
            return None

    def _is_detection_stable(self, current_objects):
        """
        判断检测结果是否稳定
        """
        self.detection_history.append(len(current_objects))
        if len(self.detection_history) > 10:
            self.detection_history.pop(0)

        if len(self.detection_history) < self.stable_threshold:
            return False

        # 检查最近几帧的物体数量是否一致
        recent_counts = self.detection_history[-self.stable_threshold:]
        return len(set(recent_counts)) == 1 and recent_counts[0] > 0

    def _synchronized_callback(self, rgb_msg, depth_msg):
        if not self.detection_enabled:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            min_area = self.get_parameter('min_contour_area').value
            enable_viz = self.get_parameter('enable_visualization').value
            viz_image = cv_image.copy() if enable_viz else None

            detected_objects = []

            for color_name, color_list in self.color_ranges.items():
                combined_mask = None
                for color_info in color_list:
                    mask = cv2.inRange(hsv_image, color_info['lower'], color_info['upper'])
                    combined_mask = mask if combined_mask is None else cv2.bitwise_or(combined_mask, mask)

                kernel = np.ones((5, 5), np.uint8)
                combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel)
                combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, kernel)

                contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                valid_contours = [c for c in contours if cv2.contourArea(c) > min_area]

                for contour in valid_contours:
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])

                        if 0 <= cy < depth_image.shape[0] and 0 <= cx < depth_image.shape[1]:
                            depth_value = float(depth_image[cy, cx])
                            coords_3d = self._pixel_to_3d(cx, cy, depth_value)

                            if coords_3d is not None:
                                X, Y, Z = coords_3d
                                detected_objects.append({
                                    'color': color_name,
                                    'center_2d': (cx, cy),
                                    'center_3d': (X, Y, Z),
                                    'depth': depth_value,
                                    'contour': contour,
                                    'color_info': color_list[0]
                                })

                                if enable_viz and viz_image is not None:
                                    color_bgr = color_list[0]['color_bgr']
                                    cv2.drawContours(viz_image, [contour], -1, color_bgr, 2)
                                    cv2.circle(viz_image, (cx, cy), 8, color_bgr, -1)
                                    label = f"{color_list[0]['name']}: ({X:.2f},{Y:.2f},{Z:.2f})"
                                    cv2.putText(viz_image, label, (cx + 15, cy - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color_bgr, 1)

            # 判断检测是否稳定
            is_stable = self._is_detection_stable(detected_objects)

            if is_stable and detected_objects:
                # 发送稳定的批量检测结果
                batch_msg = DetectionBatch()
                batch_msg.is_final = True

                for obj in detected_objects:
                    det_obj = DetectedObject()
                    det_obj.color = obj['color']
                    det_obj.x = obj['center_3d'][0]
                    det_obj.y = obj['center_3d'][1]
                    det_obj.z = obj['center_3d'][2]
                    batch_msg.objects.append(det_obj)

                self.batch_pub.publish(batch_msg)
                self.get_logger().info(f"发送稳定检测结果: {len(detected_objects)} 个物体")

                # 停止检测,等待分类器完成
                self.detection_enabled = False

            if enable_viz and viz_image is not None:
                status = "稳定" if is_stable else f"检测中({len(self.detection_history)}/{self.stable_threshold})"
                cv2.putText(viz_image, f"Status: {status}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                viz_msg = self.bridge.cv2_to_imgmsg(viz_image, encoding='bgr8')
                self.viz_pub.publish(viz_msg)

        except Exception as e:
            self.get_logger().error(f"图像处理错误: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    detector = ColorDetector()
    try:
        rclpy.spin(detector)
    except KeyboardInterrupt:
        pass
    finally:
        detector.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
