"""Mock server for the ``wb_task`` action.

The simulated work time follows the same per-connection model as the planner:

* PRODUCE: number of material connections * produce time
* RECYCLE: number of material connections * recycle time
"""

import time

import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from sml_msgs.action import WbTask

from sml_system_pkg.planning.planner_config import (
    PRODUCT_MATERIALS,
    WB_PRODUCE_TIME_SEC_PER_CONNECTION,
    WB_RECYCLE_TIME_SEC_PER_CONNECTION,
)


class MockWbNode(Node):

    def __init__(self):
        super().__init__('mock_wb_node')
        self.cbg = ReentrantCallbackGroup()

        self.declare_parameter(
            'produce_time_sec_per_connection',
            WB_PRODUCE_TIME_SEC_PER_CONNECTION,
        )
        self.declare_parameter(
            'recycle_time_sec_per_connection',
            WB_RECYCLE_TIME_SEC_PER_CONNECTION,
        )
        self.declare_parameter('unknown_product_delay_sec', 0.5)
        self.declare_parameter('max_delay_sec', 600.0)

        self._action_server = ActionServer(
            self,
            WbTask,
            'wb_task',
            execute_callback=self._execute_cb,
            callback_group=self.cbg,
        )

        self.get_logger().info(
            '[MOCK WB] wb_task 서버 시작 | '
            f'produce={self._p("produce_time_sec_per_connection"):.2f}s/connection, '
            f'recycle={self._p("recycle_time_sec_per_connection"):.2f}s/connection'
        )

    def _p(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _compute_delay(self, work_type: str, product_id: int) -> float:
        materials = PRODUCT_MATERIALS.get(int(product_id))
        if not materials:
            delay = self._p('unknown_product_delay_sec')
        else:
            connections = max(0, len(materials) - 1)
            parameter = (
                'recycle_time_sec_per_connection'
                if work_type == 'RECYCLE'
                else 'produce_time_sec_per_connection'
            )
            delay = connections * self._p(parameter)

        return min(
            max(0.0, delay),
            max(0.0, self._p('max_delay_sec')),
        )

    def _execute_cb(self, goal_handle):
        work_type = str(goal_handle.request.work_type).upper()
        product_id = int(goal_handle.request.product_id)

        if work_type not in ('PRODUCE', 'RECYCLE'):
            goal_handle.abort()
            result = WbTask.Result()
            result.success = False
            result.fail_reason = f'UNKNOWN_WORK_TYPE:{work_type}'
            self.get_logger().error(
                f'[MOCK WB] 지원하지 않는 work_type={work_type}'
            )
            return result

        delay_sec = self._compute_delay(work_type, product_id)
        self.get_logger().info(
            f'[MOCK WB] {work_type} 시작 '
            f'product_id={product_id}, delay={delay_sec:.2f}s'
        )

        feedback = WbTask.Feedback()
        feedback.status = 'WORKING'
        goal_handle.publish_feedback(feedback)

        time.sleep(delay_sec)

        feedback.status = 'COMPLETED'
        goal_handle.publish_feedback(feedback)
        goal_handle.succeed()

        result = WbTask.Result()
        result.success = True
        result.fail_reason = ''
        self.get_logger().info(
            f'[MOCK WB] {work_type} 완료 product_id={product_id}'
        )
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
