"""
mock_arm_node.py
/amr_robot_command Service 서버 mock.
request를 받으면 0.5초 후 success=True 응답.

인터페이스: arm_interfaces.srv.ArmCommand (실제/최신 정의 기준)
    Request:
        string action
        int32[] object_ids
        string location
    Response:
        bool success
        int32[] slots
        int32[] object_ids
        string message
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from sml_msgs.srv import ArmCommand


class MockArmNode(Node):

    def __init__(self):
        super().__init__('mock_arm_node')
        self.cbg = ReentrantCallbackGroup()

        self.srv = self.create_service(
            ArmCommand, '/amr_robot_command',
            self.arm_command_cb,
            callback_group=self.cbg)

        self.get_logger().info('[MOCK ARM] mock_arm_node 시작')

    def arm_command_cb(self, request, response):
        slide_ids = list(getattr(request, 'slide_ids', []))
        self.get_logger().info(
            f'[MOCK ARM] {request.action} '
            f'object_ids={list(request.object_ids)} '
            f'location={request.location} '
            f'slide_ids={slide_ids}')

        time.sleep(0.5)

        response.success = True
        response.slots = slide_ids if slide_ids else list(range(1, len(request.object_ids) + 1))
        response.object_ids = list(request.object_ids)
        response.message = 'mock success'

        self.get_logger().info(
            f'[MOCK ARM] {request.action} 완료 '
            f'slots={list(response.slots)} object_ids={list(response.object_ids)}')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = MockArmNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()