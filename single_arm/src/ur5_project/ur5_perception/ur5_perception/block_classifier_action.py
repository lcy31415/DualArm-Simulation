#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Point
from ur5_interfaces.msg import DetectionBatch
from ur5_interfaces.srv import DetectionControl
from ur5_interfaces.action import MotionCommand
import threading
import asyncio


class BlockClassifierAction(Node):
    def __init__(self):
        super().__init__('block_classifier_action')

        # 码垛区坐标定义 (从 workspace.sdf 中获取的 pallet_zone 坐标)
        self.stacking_zones = {
            'red': (-0.3, 0.6, 0.05),     # pallet_zone1
            'green': (0.01, 0.65, 0.05),      # pallet_zone2
            'blue': (0.3, 0.6, 0.05)     # pallet_zone3
        }

        # 每个颜色的码垛高度计数器
        self.stack_height = {'red': 0, 'green': 0, 'blue': 0}
        self.block_height = 0.07  # 5cm

        # 订阅批量检测结果
        self.batch_sub = self.create_subscription(
            DetectionBatch,
            '/detection_batch',
            self._batch_callback,
            10
        )

        # 创建检测控制服务客户端
        self.detection_control_client = self.create_client(
            DetectionControl,
            '/detection_control'
        )

        # 创建 Action 客户端（替代原有的 topic 发布器）
        self._motion_action_client = ActionClient(
            self,
            MotionCommand,
            '/motion_command'
        )

        # 处理队列和状态
        self.object_queue = []
        self.is_processing = False
        self.home_position = (0.5, 0.0, 0.5)  # 机械臂初始位置

        # 等待服务和 Action 服务器可用
        while not self.detection_control_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待检测控制服务...')

        self.get_logger().info('等待运动控制 Action 服务器...')
        self._motion_action_client.wait_for_server()

        self.get_logger().info("物块分类节点已启动(Action 模式)！")
        self.get_logger().info("订阅: /detection_batch")
        self.get_logger().info(f"码垛区配置:")
        for color, pos in self.stacking_zones.items():
            self.get_logger().info(f"  {color}: {pos}")

        # 启动后主动请求启用检测
        self.get_logger().info("请求启用检测...")
        self._enable_detection(True)

    def _enable_detection(self, enable):
        """
        控制检测开关(异步调用)
        """
        request = DetectionControl.Request()
        request.enable = enable
        future = self.detection_control_client.call_async(request)
        future.add_done_callback(
            lambda f: self.get_logger().info(
                f"检测控制: {'启用' if enable else '停止'} - {'成功' if f.result() and f.result().success else '失败'}"
            ) if f.result() is not None else self.get_logger().error("检测控制服务调用失败")
        )

    def _batch_callback(self, msg: DetectionBatch):
        """
        接收批量检测结果
        """
        if not msg.is_final:
            return

        self.get_logger().info(f"收到稳定检测结果: {len(msg.objects)} 个物体")

        # 将检测结果加入队列
        self.object_queue = [(obj.color, obj.x, obj.y, obj.z) for obj in msg.objects]

        # 开始处理
        if not self.is_processing:
            # 在独立线程中处理,避免阻塞执行器
            processing_thread = threading.Thread(target=self._process_all_objects)
            processing_thread.daemon = True
            processing_thread.start()

    def _process_all_objects(self):
        """
        处理所有检测到的物体
        """
        if len(self.object_queue) == 0:
            self.get_logger().info("所有物体处理完成,回到初始位置")
            self.is_processing = False

            # 回到初始位置
            self._send_move_command(*self.home_position)

            # 重新启用检测
            self._enable_detection(True)
            return

        self.is_processing = True

        # 取出第一个物体
        color, x, y, z = self.object_queue.pop(0)
        self.get_logger().info(f"开始处理 {color} 物块: ({x:.3f}, {y:.3f}, {z:.3f})")

        # 执行抓取和放置
        self._execute_pick_and_place((x, y, z), color)

        # 继续处理下一个
        self._process_all_objects()

    def _execute_pick_and_place(self, pick_pos, color):
        """
        执行抓取和放置序列
        """
        x, y, z = pick_pos

        # 计算放置位置
        stack_x, stack_y, stack_base_z = self.stacking_zones[color]
        stack_z = stack_base_z + self.stack_height[color] * self.block_height

        self.get_logger().info(f"抓取: ({x:.3f}, {y:.3f}, {z:.3f}) → 放置: ({stack_x:.3f}, {stack_y:.3f}, {stack_z:.3f})")

        # 1. 打开夹爪
        self._send_gripper_command(False)

        # 2. 移动到物体上方
        approach_z = z + 0.2
        self._send_move_command(x, y, approach_z)

        # 3. 下降到抓取位置
        self._send_move_command(x, y, z)

        # 4. 闭合夹爪
        self._send_gripper_command(True)

        # 5. 提升物体
        self._send_move_command(x, y, approach_z)

        # 6. 移动到码垛区上方
        approach_stack_z = stack_z + 0.15
        self._send_move_command(stack_x, stack_y, approach_stack_z)


        # 8. 打开夹爪
        self._send_gripper_command(False)
        
        self._send_move_command(stack_x, stack_y, approach_stack_z+0.06)

        # 更新码垛高度
        self.stack_height[color] += 1
        self.get_logger().info(f"{color} 码垛高度: {self.stack_height[color]}")

    def _send_move_command(self, x, y, z):
        """
        发送移动命令并等待完成（使用 spin_until_future_complete）
        """
        goal_msg = MotionCommand.Goal()
        goal_msg.command_type = "move"
        goal_msg.target_position = Point(x=x, y=y, z=z)

        self.get_logger().info(f"发送移动命令: ({x:.3f}, {y:.3f}, {z:.3f})")

        # 发送目标
        send_goal_future = self._motion_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('移动命令被拒绝')
            return False

        # 等待结果
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result

        if result.success:
            self.get_logger().info(f'✓ 移动完成: {result.message}')
            return True
        else:
            self.get_logger().error(f'✗ 移动失败: {result.message} (错误码: {result.error_code})')
            return False

    def _send_gripper_command(self, close):
        """
        发送夹爪命令并等待完成（使用 spin_until_future_complete）
        """
        goal_msg = MotionCommand.Goal()
        goal_msg.command_type = "gripper"
        goal_msg.gripper_close = close

        self.get_logger().info(f"发送夹爪命令: {'闭合' if close else '打开'}")

        # 发送目标
        send_goal_future = self._motion_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('夹爪命令被拒绝')
            return False

        # 等待结果
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result

        if result.success:
            self.get_logger().info(f'✓ 夹爪操作完成: {result.message}')
            return True
        else:
            self.get_logger().error(f'✗ 夹爪操作失败: {result.message}')
            return False


def main(args=None):
    rclpy.init(args=args)
    classifier = BlockClassifierAction()

    # 使用多线程执行器以支持异步 Action 调用
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(classifier)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        classifier.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
