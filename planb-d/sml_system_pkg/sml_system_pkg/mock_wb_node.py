"""Mock Workbench action server for Plan D tests.

시간 모델:
  - delay_sec >= 0 이면 기존처럼 고정 지연 시간을 사용한다.
  - delay_sec < 0 이면 product_id와 work_type에 따라
    조립/분해 연결 개수 × per-connection 시간으로 계산한다.
"""

import time

import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from sml_msgs.action import WbTask


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


class MockWbNode(Node):
    def __init__(self):
        super().__init__('mock_wb_node')
        self.cbg = ReentrantCallbackGroup()

        # 기존 테스트 호환용. 0 이상이면 무조건 고정 지연 시간으로 사용.
        # 제품별 시간 모델을 쓰려면 -p delay_sec:=-1.0 으로 실행한다.
        self.declare_parameter('delay_sec', -1.0)
        self.declare_parameter('wb_produce_time_sec_per_connection', 10.0)
        self.declare_parameter('wb_recycle_time_sec_per_connection', 10.0)
        self.declare_parameter('wb_base_time_sec', 0.0)
        self.declare_parameter('max_delay_sec', 900.0)

        self._action_server = ActionServer(
            self,
            WbTask,
            'wb_task',
            execute_callback=self._execute_cb,
            callback_group=self.cbg,
        )
        self.get_logger().info(
            '[MOCK WB] wb_task 서버 시작 | '
            'delay_sec<0이면 제품별 시간 모델 사용'
        )

    def _p(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _connection_count(self, product_id: int) -> int:
        materials = PRODUCT_MATERIALS.get(int(product_id), [])
        return max(0, len(materials) - 1)

    def _compute_delay(self, work_type: str, product_id: int) -> float:
        fixed_delay = self._p('delay_sec')
        if fixed_delay >= 0.0:
            return min(fixed_delay, max(0.0, self._p('max_delay_sec')))

        work_type = work_type.upper()
        connections = self._connection_count(product_id)
        base = self._p('wb_base_time_sec')

        if work_type == 'RECYCLE':
            per_connection = self._p('wb_recycle_time_sec_per_connection')
        else:
            per_connection = self._p('wb_produce_time_sec_per_connection')

        delay = base + connections * per_connection
        return min(max(0.0, delay), max(0.0, self._p('max_delay_sec')))

    def _execute_cb(self, goal_handle):
        work_type = goal_handle.request.work_type
        product_id = int(goal_handle.request.product_id)
        delay_sec = self._compute_delay(work_type, product_id)

        self.get_logger().info(
            f'[MOCK WB] goal 수신: work_type={work_type}, product_id={product_id}, '
            f'delay={delay_sec:.2f}s'
        )

        fb = WbTask.Feedback()
        fb.status = 'PROCESSING'
        goal_handle.publish_feedback(fb)
        time.sleep(max(0.0, delay_sec / 2.0))

        fb.status = work_type
        goal_handle.publish_feedback(fb)
        time.sleep(max(0.0, delay_sec / 2.0))

        goal_handle.succeed()
        result = WbTask.Result()
        result.success = True
        result.fail_reason = ''
        self.get_logger().info(f'[MOCK WB] 완료: {work_type} product_id={product_id}')
        return result


def main(args=None):
    rclpy.init(args=args)
    node = MockWbNode()
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
