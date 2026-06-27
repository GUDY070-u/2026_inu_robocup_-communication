"""ROS2 planning node wrapper for Plan C only.

이 버전은 Plan A/B를 전혀 참조하지 않는다.
항상 sml_system_pkg.planning.PlannerCore, 즉 Plan C 플래너만 사용한다.

A/B 경기장 대응:
- 경기 서버/OrderServer가 넘긴 AMR 실제 station id를 PlannerCore 계산용 station id로 변환한다.
- side:=b일 때는 9~16을 계산용 1~8로 바꿔 PlannerCore/기존 JSON을 그대로 사용한다.
- PlannerCore가 만든 Step은 다시 AMR 실제 station id로 복원한다.
- 수신 직후 경기 서버가 준 원본 order_list / arena_layout을 로그로 출력한다.
"""

import rclpy
from rclpy.node import Node

from sml_msgs.msg import Task, Station, Order
from sml_msgs.srv import GetPlan

from .arena_side_utils import (
    normalize_side,
    side_to_fixed_workbench_station,
    amr_station_to_planner_station,
    planner_station_to_amr_station,
)
from .planning.arena_parser import load_station_coord_json
from .planning.planner_config import (
    AMR_SPEED,
    DEFAULT_STATION_COORD_JSON_PATH,
    PRODUCT_NAMES,
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
        self.declare_parameter('side', 'a')
        self.declare_parameter('use_time_cost', True)
        self.declare_parameter('amr_speed_mps', AMR_SPEED)
        self.declare_parameter(
            STATION_COORD_JSON_PARAM,
            DEFAULT_STATION_COORD_JSON_PATH,
        )

        task_topic = self.get_parameter('task_topic').value
        self.side = normalize_side(self.get_parameter('side').value)
        self.fixed_workbench_station = side_to_fixed_workbench_station(self.side)

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
            f'PlanningNode 시작 | task_topic={task_topic} | side={self.side} | '
            f'planner=plan_c_only | '
            f'fixed_workbench_station={self.fixed_workbench_station} | '
            f'use_time_cost={use_time_cost} | coords={len(station_coords)}'
        )

    # ──────────────────────────────────────────────────────
    # 경기 서버 / OrderServer 원본 Task 로그
    # ──────────────────────────────────────────────────────

    def _order_type_label(self, order_type: int) -> str:
        if order_type == Order.OT_PRODUCE:
            return 'P'
        if order_type == Order.OT_RECYCLE:
            return 'R'
        if order_type == getattr(Order, 'OT_LIFECYCLE', 3):
            return 'L'
        return '?'

    def _product_name(self, product_id: int) -> str:
        return PRODUCT_NAMES.get(int(product_id), 'Unknown')

    def _log_task_from_server(self, task: Task):
        """
        경기 서버/OrderServer가 넘긴 원본 Task를 그대로 출력한다.

        side=b인 경우 여기에는 9~16 station_id가 그대로 표시된다.
        PlannerCore 계산용으로 1~8 변환하기 전의 원본 확인용 로그다.
        """
        self.get_logger().info('===== 경기 서버 Task 원본 =====')

        self.get_logger().info('order_list = ')
        self.get_logger().info('{')
        for order in task.order_list:
            order_type = int(order.order_type)
            product_id = int(order.product_id)
            label = self._order_type_label(order_type)
            name = self._product_name(product_id)
            self.get_logger().info(
                f'   order_type = {order_type} ; '
                f'product_id = {product_id:<18} '
                f'# {name:<18} ({label})'
            )
        self.get_logger().info('}')

        self.get_logger().info('arena_layout = ')
        self.get_logger().info('{')
        for station in task.arena_layout:
            material_text = ', '.join(str(x) for x in list(station.material_ids))
            self.get_logger().info(
                f'   station_type = {int(station.station_type)}; '
                f'station_id = {int(station.station_id)}; '
                f'material_ids = {{{material_text}}}'
            )
        self.get_logger().info('}')
        self.get_logger().info('===============================')

    def _task_for_planner_coordinates(self, task: Task) -> Task:
        """
        PlannerCore/JSON 계산용 Task 생성.

        side a:
            station_id를 그대로 사용.

        side b:
            Adapter/OrderServer가 만든 AMR station id 9~16을 PlannerCore 계산용 1~8로 변환.
            예: 15(B 조립로봇) -> 6(A 계산용 조립로봇 좌표)
        """
        if self.side == 'a':
            return task

        planner_task = Task()
        planner_task.order_list = list(task.order_list)
        planner_task.arena_layout = []

        for src_station in task.arena_layout:
            dst_station = Station()
            dst_station.station_type = int(src_station.station_type)
            dst_station.station_id = amr_station_to_planner_station(
                int(src_station.station_id),
                self.side,
            )
            dst_station.material_ids = list(src_station.material_ids)
            planner_task.arena_layout.append(dst_station)

            self.get_logger().info(
                '[PLANNER] station 계산용 변환: '
                f'amr_id={int(src_station.station_id)} -> '
                f'planner_id={dst_station.station_id}, '
                f'type={dst_station.station_type}, '
                f'materials={list(dst_station.material_ids)}'
            )

        return planner_task

    def _steps_to_amr_station_ids(self, steps):
        """
        PlannerCore 결과 Step의 station_id를 AMR 실제 station id로 복원.
        side b:
            6 -> 15, 8 -> 16 등으로 변환.
        """
        if self.side == 'a':
            return steps

        for step in steps:
            old_station = int(step.station_id)
            new_station = planner_station_to_amr_station(old_station, self.side)
            step.station_id = int(new_station)

            if old_station != new_station:
                self.get_logger().info(
                    '[PLANNER] step station AMR용 복원: '
                    f'step={step.step_id}, '
                    f'planner_station={old_station} -> amr_station={new_station}'
                )

        return steps

    def task_callback(self, task):
        if self.plan_generated:
            return

        self.plan_generated = True
        self.get_logger().info('Task 수신 → Plan C 계획 생성 시작')

        try:
            # 경기 서버/OrderServer가 준 원본 task를 먼저 출력한다.
            # side=b이면 9~16 station_id가 그대로 보인다.
            self._log_task_from_server(task)

            planner_task = self._task_for_planner_coordinates(task)
            planned_steps = self.planner.build_plan(planner_task)
            self.steps = self._steps_to_amr_station_ids(planned_steps)
        except Exception as e:
            self.steps = []
            self.plan_generated = False
            self.get_logger().error(f'Plan C 계획 생성 실패: {e}')

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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()