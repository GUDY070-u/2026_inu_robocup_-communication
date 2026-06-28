"""
mock_wb_node.py
wb_task Action 서버 mock.
goal 수신 → PROCESSING 피드백 → delay 후 success=True 반환.
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from sml_plan_d_msgs.action import WbTask

WB_DELAY = 60.0  # 작업 시뮬레이션 시간 (초)


class MockWbNode(Node):

    def __init__(self):
        super().__init__('mock_wb_node')
        self.cbg = ReentrantCallbackGroup()

        self._action_server = ActionServer(
            self,
            WbTask,
            'wb_task',
            execute_callback=self._execute_cb,
            callback_group=self.cbg,
        )
        self.get_logger().info('[MOCK WB] wb_task 서버 시작')

    def _execute_cb(self, goal_handle):
        work_type  = goal_handle.request.work_type
        product_id = goal_handle.request.product_id
        self.get_logger().info(
            f'[MOCK WB] goal 수신: work_type={work_type}, product_id={product_id}')

        # 피드백: PROCESSING
        fb = WbTask.Feedback()
        fb.status = 'PROCESSING'
        goal_handle.publish_feedback(fb)

        time.sleep(WB_DELAY / 2)

        # 피드백: 작업 중 상태
        fb.status = work_type  # "PRODUCING" or "RECYCLING"
        goal_handle.publish_feedback(fb)

        time.sleep(WB_DELAY / 2)

        goal_handle.succeed()

        result = WbTask.Result()
        result.success = True
        result.fail_reason = ''
        self.get_logger().info(
            f'[MOCK WB] 완료: {work_type} product_id={product_id}')
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
