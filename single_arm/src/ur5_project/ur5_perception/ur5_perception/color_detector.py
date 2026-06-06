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

        # 相机内参矩阵（从 camera_info 获取）
        self.camera_matrix = None
        self.image_width = None
        self.image_height = None

        # TF2 坐标变换
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.target_frame = 'base_link'  # 目标坐标系：机械臂基座
        self.camera_frame = 'camera_optical_frame'  # 相机光学坐标系

        # 机械臂基座实际偏移量（相对于 URDF 中的位置）
        self.base_offset_x = 0.0  # 机械臂基座在 x=0 的位置

        # 检测控制状态
        self.detection_enabled = True  # 是否启用检测
        self.detection_history = []    # 检测历史记录(用于稳定性判断)
        self.stable_threshold = 3      # 连续稳定帧数
        self.z_offset = 0.1            # z轴抓取偏移(0.1m)

        # 订阅相机信息话题（获取相机内参）
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/rgb/camera_info',
            self._camera_info_callback,
            10
        )

        # 创建检测控制服务
        self.detection_control_srv = self.create_service(
            DetectionControl,
            '/detection_control',
            self._detection_control_callback
        )

        # 使用 message_filters 同步 RGB 和深度图像
        self.rgb_sub = message_filters.Subscriber(
            self,
            Image,
            '/camera/rgb/image_raw'
        )
        self.depth_sub = message_filters.Subscriber(
            self,
            Image,
            '/camera/depth/image_raw'
        )

        # 时间同步器（允许 0.1 秒的时间差）
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub],
            queue_size=10,
            slop=0.1
        )
        self.ts.registerCallback(self._synchronized_callback)

        # 发布批量检测结果
        self.batch_pub = self.create_publisher(
            DetectionBatch,
            '/detection_batch',
            10
        )

        # 发布可视化图像（带标注）
        self.viz_pub = self.create_publisher(
            Image,
            '/detected_object_viz',
            10
        )

        # 配置多颜色检测的 HSV 范围
        # 注意：红色在 HSV 中会环绕，需要两个范围
        self.color_ranges = {
            'red': [
                {
                    'lower': np.array([0, 120, 70]),
                    'upper': np.array([10, 255, 255]),
                    'color_bgr': (0, 0, 255),
                    'name': 'Red'
                },
                {
                    'lower': np.array([170, 120, 70]),
                    'upper': np.array([180, 255, 255]),
                    'color_bgr': (0, 0, 255),
                    'name': 'Red'
                }
            ],
            'green': [
                {
                    'lower': np.array([40, 40, 40]),
                    'upper': np.array([80, 255, 255]),
                    'color_bgr': (0, 255, 0),
                    'name': 'Green'
                }
            ],
            'blue': [
                {
                    'lower': np.array([100, 100, 70]),
                    'upper': np.array([130, 255, 255]),
                    'color_bgr': (255, 0, 0),
                    'name': 'Blue'
                }
            ]
        }

        # 最小轮廓面积（过滤噪声）
        self.declare_parameter('min_contour_area', 300)

        # 是否发布可视化图像
        self.declare_parameter('enable_visualization', True)

        self.get_logger().info("颜色检测节点已启动！")
        self.get_logger().info("订阅话题: /camera/rgb/image_raw, /camera/depth/image_raw")
        self.get_logger().info("订阅话题: /camera/rgb/camera_info")
        self.get_logger().info(f"发布话题: /detected_object_center (在 {self.target_frame} 坐标系下的3D坐标)")
        self.get_logger().info("可视化话题: /detected_object_viz")
        self.get_logger().info("检测颜色: 红色、绿色、蓝色")

    def _camera_info_callback(self, msg):
        """
        接收相机内参信息
        """
        if self.camera_matrix is None:
            # 解析相机内参矩阵 K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
            K = np.array(msg.k).reshape(3, 3)
            self.camera_matrix = K
            self.image_width = msg.width
            self.image_height = msg.height

            self.get_logger().info(f"相机内参已获取:")
            self.get_logger().info(f"  fx={K[0,0]:.2f}, fy={K[1,1]:.2f}")
            self.get_logger().info(f"  cx={K[0,2]:.2f}, cy={K[1,2]:.2f}")
            self.get_logger().info(f"  分辨率: {self.image_width}x{self.image_height}")

    def _pixel_to_3d(self, cx, cy, depth_value):
        """
        将像素坐标和深度值转换为机械臂基座坐标系下的3D坐标

        参数:
            cx, cy: 像素坐标
            depth_value: 深度值（米）

        返回:
            (X, Y, Z): 机械臂基座坐标系下的3D坐标（米）
        """
        if self.camera_matrix is None or depth_value <= 0:
            return None

        # 步骤1: 像素坐标转换为相机光学坐标系下的3D坐标
        # 相机内参
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx_cam = self.camera_matrix[0, 2]
        cy_cam = self.camera_matrix[1, 2]

        # 针孔相机模型: 相机光学坐标系 (Z向前, X向右, Y向下)
        X_cam = (cx - cx_cam) * depth_value / fx
        Y_cam = (cy - cy_cam) * depth_value / fy
        Z_cam = depth_value

        # 步骤2: 使用 TF2 将相机坐标系转换到机械臂基座坐标系
        try:
            # 创建 PointStamped 消息
            point_cam = PointStamped()
            point_cam.header.frame_id = self.camera_frame
            point_cam.header.stamp = self.get_clock().now().to_msg()
            point_cam.point.x = X_cam
            point_cam.point.y = Y_cam
            point_cam.point.z = Z_cam

            # 转换到目标坐标系
            point_base = self.tf_buffer.transform(point_cam, self.target_frame, timeout=rclpy.duration.Duration(seconds=0.1))

            # 应用机械臂基座的实际偏移量
            # URDF 中 base_link 在 (0,0,0)，但实际在 (0.4,0,0)
            # 所以需要减去偏移量
            X_actual = point_base.point.x - self.base_offset_x
            Y_actual = point_base.point.y
            Z_actual = point_base.point.z

            return (X_actual, Y_actual, Z_actual)

        except Exception as e:
            self.get_logger().warning(f"TF 转换失败: {str(e)}")
            return None

    def _synchronized_callback(self, rgb_msg, depth_msg):
        """
        处理同步的 RGB 和深度图像，检测多种颜色的物体
        """
        try:
            # 1. 将 ROS Image 消息转换为 OpenCV 格式
            cv_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

            # 2. BGR 转 HSV
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            # 获取参数
            min_area = self.get_parameter('min_contour_area').value
            enable_viz = self.get_parameter('enable_visualization').value

            # 创建可视化图像
            viz_image = cv_image.copy() if enable_viz else None

            # 存储所有检测到的物体
            detected_objects = []

            # 3. 遍历每种颜色进行检测
            for color_name, color_list in self.color_ranges.items():
                # 对于每种颜色（可能有多个 HSV 范围，如红色）
                combined_mask = None

                for color_info in color_list:
                    # 创建颜色掩膜
                    mask = cv2.inRange(hsv_image, color_info['lower'], color_info['upper'])

                    # 合并掩膜（对于红色的两个范围）
                    if combined_mask is None:
                        combined_mask = mask
                    else:
                        combined_mask = cv2.bitwise_or(combined_mask, mask)

                # 4. 形态学操作（去除噪声）
                kernel = np.ones((5, 5), np.uint8)
                combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel)
                combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, kernel)

                # 5. 寻找轮廓
                contours, _ = cv2.findContours(
                    combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
                )

                # 6. 处理所有有效轮廓（不只是最大的）
                valid_contours = [c for c in contours if cv2.contourArea(c) > min_area]

                for contour in valid_contours:
                    # 计算轮廓的矩（Moments）
                    M = cv2.moments(contour)

                    if M["m00"] != 0:
                        # 计算中心点
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        area = cv2.contourArea(contour)

                        # 从深度图读取深度值
                        if 0 <= cy < depth_image.shape[0] and 0 <= cx < depth_image.shape[1]:
                            depth_value = float(depth_image[cy, cx])

                            # 转换为3D坐标
                            coords_3d = self._pixel_to_3d(cx, cy, depth_value)

                            if coords_3d is not None:
                                X, Y, Z = coords_3d

                                # 存储检测结果
                                detected_objects.append({
                                    'color': color_name,
                                    'center_2d': (cx, cy),
                                    'center_3d': (X, Y, Z),
                                    'depth': depth_value,
                                    'area': area,
                                    'contour': contour,
                                    'color_info': color_list[0]  # 使用第一个颜色信息用于可视化
                                })

                                # 发布3D中心点坐标
                                center_point = Point()
                                center_point.x = X
                                center_point.y = Y
                                center_point.z = Z
                                self.center_pub.publish(center_point)

            # 7. 日志输出检测结果
            if detected_objects:
                self.get_logger().info(
                    f"检测到 {len(detected_objects)} 个物体: " +
                    ", ".join([
                        f"{obj['color']}(2D:[{obj['center_2d'][0]},{obj['center_2d'][1]}], "
                        f"3D:[{obj['center_3d'][0]:.3f},{obj['center_3d'][1]:.3f},{obj['center_3d'][2]:.3f}])"
                        for obj in detected_objects
                    ])
                )

                # 8. 可��化
                if enable_viz and viz_image is not None:
                    for obj in detected_objects:
                        color_bgr = obj['color_info']['color_bgr']
                        cx, cy = obj['center_2d']
                        X, Y, Z = obj['center_3d']

                        # 绘制轮廓
                        cv2.drawContours(viz_image, [obj['contour']], -1, color_bgr, 2)

                        # 绘制中心点
                        cv2.circle(viz_image, (cx, cy), 8, color_bgr, -1)
                        cv2.circle(viz_image, (cx, cy), 10, (255, 255, 255), 2)

                        # 添加标签（2D像素坐标）
                        label_2d = f"{obj['color_info']['name']}: ({cx},{cy})"
                        cv2.putText(viz_image, label_2d, (cx + 15, cy - 25),
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.4, color_bgr, 1)

                        # 添加3D坐标标签
                        label_3d = f"3D: ({X:.2f},{Y:.2f},{Z:.2f})m"
                        cv2.putText(viz_image, label_3d, (cx + 15, cy - 10),
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.4, color_bgr, 1)

                    # 添加统计信息
                    stats_text = f"Total: {len(detected_objects)} objects"
                    cv2.putText(viz_image, stats_text, (10, 30),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

                    # 发布可视化图像
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
