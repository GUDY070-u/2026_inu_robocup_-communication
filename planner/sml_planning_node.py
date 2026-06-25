"""ROS2 planning node wrapper.

The actual planning algorithm lives in sml_system_pkg.planning.*.
This file intentionally keeps only ROS subscription/service wiring.
"""

import rclpy
from rclpy.node import Node

from sml_msgs.msg import Task
from sml_msgs.srv import GetPlan

from .planning.arena_parser import load_station_coord_json
from .planning.planner_config import (
    AMR_SPEED,
    DEFAULT_STATION_COORD_JSON_PATH,
    STATION_COORD_JSON_PARAM,
    PlannerConfig,
)
from .planning.planner_core import PlannerCore


class PlanningNode(Node):

    def __init__(self):
        super().__init__('planning_node')

        self.plan_generated = False
        self.steps = []

        self.declare_parameter('task_topic', '/sml/task')
        self.declare_parameter('use_time_cost', True)
        self.declare_parameter('amr_speed_mps', AMR_SPEED)
        self.declare_parameter(
            STATION_COORD_JSON_PARAM,
            DEFAULT_STATION_COORD_JSON_PATH
        )

        task_topic = self.get_parameter('task_topic').value
        use_time_cost = bool(self.get_parameter('use_time_cost').value)
        amr_speed_mps = float(self.get_parameter('amr_speed_mps').value)
        station_coord_json_path = self.get_parameter(
            STATION_COORD_JSON_PARAM
        ).get_parameter_value().string_value.strip()

        config = PlannerConfig(
            use_time_cost=use_time_cost,
            amr_speed_mps=amr_speed_mps,
            station_coord_json_path=station_coord_json_path,
        )
        station_coords = load_station_coord_json(
            config.station_coord_json_path,
            self.get_logger(),
        )
        self.planner = PlannerCore(
            config=config,
            station_coords=station_coords,
            logger=self.get_logger(),
        )

        self.task_sub = self.create_subscription(
            Task, task_topic, self.task_callback, 10
        )
        self.plan_srv = self.create_service(
            GetPlan, '/sml/get_plan', self.get_plan_callback
        )

        self.get_logger().info(
            f'PlanningNode 시작 | task_topic={task_topic} | '
            f'use_time_cost={use_time_cost} | coords={len(station_coords)}'
        )

    def task_callback(self, task):
        if self.plan_generated:
            return

        self.plan_generated = True
        self.get_logger().info('Task 수신 → 계획 생성 시작')

        try:
            self.steps = self.planner.build_plan(task)
        except Exception as e:
            self.steps = []
            self.plan_generated = False
            self.get_logger().error(f'계획 생성 실패: {e}')

    def get_plan_callback(self, request, response):
        if not self.plan_generated or not self.steps:
            response.success = False
            response.message = '계획이 아직 생성되지 않았습니다'
            return response

        response.steps = self.steps
        response.success = True
        response.message = ''
        self.get_logger().info(f'GetPlan 응답: {len(self.steps)}개 스텝 전달')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = PlanningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
