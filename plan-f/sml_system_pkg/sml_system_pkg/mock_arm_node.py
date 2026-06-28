"""
mock_arm_node.py
/amr_robot_command Service 서버 mock.

Plan D 시간 모델:
  - LOAD/UNLOAD: 물체 개수 × per-item 시간
  - ASSEMBLE: product_id의 연결 개수 × AMR 조립 시간

인터페이스:
    Request:
        string action
        int32[] object_ids
        int32 location
        int32[] slide_ids
    Response:
        bool success
        int32[] slots
        int32[] object_ids
        string message
"""

import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from sml_msgs.srv import ArmCommand


PRODUCT_MATERIALS = {
    34: [3, 4],
    13: [1, 3],
    81: [8, 1],
    442: [4, 4, 2],
    241: [2, 4, 1],
    462: [4, 6, 2],
    711: [1, 1, 7],
    4482: [4, 4, 8, 2],
    8518: [8, 5, 1, 8],
    48132: [4, 8, 1, 3, 2],
    46262: [4, 6, 2, 6, 2],
}


class MockArmNode(Node):

    def __init__(self):
        super().__init__('mock_arm_node')
        self.cbg = ReentrantCallbackGroup()

        self.declare_parameter('load_time_sec_per_item', 2.0)
        self.declare_parameter('unload_time_sec_per_item', 2.0)
        self.declare_parameter('amr_assemble_time_sec_per_connection', 4.0)
        self.declare_parameter('amr_assemble_base_time_sec', 0.0)
        self.declare_parameter('fallback_delay_sec', 0.5)
        self.declare_parameter('max_delay_sec', 600.0)

        self.srv = self.create_service(
            ArmCommand,
            '/amr_robot_command',
            self.arm_command_cb,
            callback_group=self.cbg,
        )

        self.get_logger().info(
            '[MOCK ARM] mock_arm_node 시작 | '
            f'load={self._p("load_time_sec_per_item"):.2f}s/item, '
            f'unload={self._p("unload_time_sec_per_item"):.2f}s/item, '
            f'amr_assemble={self._p("amr_assemble_time_sec_per_connection"):.2f}s/connection'
        )

    def _p(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _connection_count_for_product(self, product_id: int, fallback_inputs: int = 0) -> int:
        materials = PRODUCT_MATERIALS.get(int(product_id))
        if materials:
            return max(0, len(materials) - 1)
        return max(0, int(fallback_inputs) - 1)

    def _compute_delay(self, request) -> float:
        action = str(request.action).upper()
        n_objects = len(request.object_ids)

        if action == 'LOAD':
            delay = n_objects * self._p('load_time_sec_per_item')
        elif action == 'UNLOAD':
            delay = n_objects * self._p('unload_time_sec_per_item')
        elif action == 'ASSEMBLE':
            base = self._p('amr_assemble_base_time_sec')
            per_connection = self._p('amr_assemble_time_sec_per_connection')
            # 보통 object_ids=[product_id] 형태다.
            connections = 0
            fallback_inputs = len(request.slide_ids)
            for product_id in request.object_ids:
                connections += self._connection_count_for_product(
                    int(product_id),
                    fallback_inputs=fallback_inputs,
                )
            delay = base + connections * per_connection
        else:
            delay = self._p('fallback_delay_sec')

        return min(max(0.0, float(delay)), max(0.0, self._p('max_delay_sec')))

    def arm_command_cb(self, request, response):
        delay_sec = self._compute_delay(request)

        self.get_logger().info(
            f'[MOCK ARM] {request.action} '
            f'object_ids={list(request.object_ids)} '
            f'location={request.location} '
            f'slide_ids={list(request.slide_ids)} '
            f'delay={delay_sec:.2f}s'
        )

        time.sleep(delay_sec)

        response.success = True
        response.slots = (
            list(request.slide_ids)
            if len(request.slide_ids) > 0
            else list(range(1, len(request.object_ids) + 1))
        )
        response.object_ids = list(request.object_ids)
        response.message = 'mock success'

        self.get_logger().info(
            f'[MOCK ARM] {request.action} 완료 '
            f'slots={list(response.slots)} object_ids={list(response.object_ids)}'
        )
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
