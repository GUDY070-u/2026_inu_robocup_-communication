"""ROS2 wrapper for the Plan D planner."""

import rclpy
from rclpy.node import Node

from sml_msgs.msg import Station, Task
from sml_msgs.srv import GetPlan

from .arena_side_utils import (
    amr_station_to_planner_station,
    normalize_side,
    planner_station_to_amr_station,
    side_to_fixed_workbench_station,
)
from .planning.arena_parser import load_station_coord_json
from .planning.planner_config import (
    AMR_ASSEMBLE_TIME_SEC_PER_CONNECTION,
    AMR_LOAD_TIME_SEC_PER_ITEM,
    AMR_SPEED,
    AMR_UNLOAD_TIME_SEC_PER_ITEM,
    DEFAULT_STATION_COORD_JSON_PATH,
    NAV_OVERHEAD_SEC,
    STATION_COORD_JSON_PARAM,
    WB_PRODUCE_TIME_SEC_PER_CONNECTION,
    WB_RECYCLE_TIME_SEC_PER_CONNECTION,
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
        self.declare_parameter('amr_load_time_sec_per_item', AMR_LOAD_TIME_SEC_PER_ITEM)
        self.declare_parameter('amr_unload_time_sec_per_item', AMR_UNLOAD_TIME_SEC_PER_ITEM)
        self.declare_parameter('amr_assemble_time_sec_per_connection', AMR_ASSEMBLE_TIME_SEC_PER_CONNECTION)
        self.declare_parameter('wb_produce_time_sec_per_connection', WB_PRODUCE_TIME_SEC_PER_CONNECTION)
        self.declare_parameter('wb_recycle_time_sec_per_connection', WB_RECYCLE_TIME_SEC_PER_CONNECTION)
        self.declare_parameter('nav_overhead_sec', NAV_OVERHEAD_SEC)
        self.declare_parameter(STATION_COORD_JSON_PARAM, DEFAULT_STATION_COORD_JSON_PATH)

        task_topic = self.get_parameter('task_topic').value
        self.side = normalize_side(self.get_parameter('side').value)
        self.fixed_workbench_station = side_to_fixed_workbench_station(self.side)
        planner_fixed_wb = amr_station_to_planner_station(
            self.fixed_workbench_station,
            self.side,
        )

        config = PlannerConfig(
            use_time_cost=bool(self.get_parameter('use_time_cost').value),
            amr_speed_mps=float(self.get_parameter('amr_speed_mps').value),
            station_coord_json_path=self.get_parameter(
                STATION_COORD_JSON_PARAM
            ).get_parameter_value().string_value.strip(),
            fixed_workbench_station_id=planner_fixed_wb,
            amr_load_time_sec_per_item=float(
                self.get_parameter('amr_load_time_sec_per_item').value
            ),
            amr_unload_time_sec_per_item=float(
                self.get_parameter('amr_unload_time_sec_per_item').value
            ),
            amr_assemble_time_sec_per_connection=float(
                self.get_parameter('amr_assemble_time_sec_per_connection').value
            ),
            wb_produce_time_sec_per_connection=float(
                self.get_parameter('wb_produce_time_sec_per_connection').value
            ),
            wb_recycle_time_sec_per_connection=float(
                self.get_parameter('wb_recycle_time_sec_per_connection').value
            ),
            nav_overhead_sec=float(self.get_parameter('nav_overhead_sec').value),
        )
        station_coords = load_station_coord_json(config.station_coord_json_path, self.get_logger())
        self.planner = PlannerCore(
            config=config,
            station_coords=station_coords,
            logger=self.get_logger(),
        )

        self.task_sub = self.create_subscription(Task, task_topic, self.task_callback, 10)
        self.plan_srv = self.create_service(GetPlan, '/sml/get_plan', self.get_plan_callback)

        self.get_logger().info(
            f'PlanningNode 시작 | task_topic={task_topic} | side={self.side} | '
            f'fixed_workbench_station={self.fixed_workbench_station} | '
            f'planner_fixed_workbench_station={planner_fixed_wb} | '
            f'use_time_cost={config.use_time_cost} | coords={len(station_coords)} | '
            f'amr_speed={config.amr_speed_mps:.2f}m/s | '
            f'AMR load/unload={config.amr_load_time_sec_per_item:.2f}/'
            f'{config.amr_unload_time_sec_per_item:.2f}s/item | '
            f'AMR assemble={config.amr_assemble_time_sec_per_connection:.2f}s/conn | '
            f'WB produce/recycle={config.wb_produce_time_sec_per_connection:.2f}/'
            f'{config.wb_recycle_time_sec_per_connection:.2f}s/conn'
        )

    def _task_for_planner_coordinates(self, task: Task) -> Task:
        if self.side == 'a':
            return task

        planner_task = Task()
        planner_task.order_list = list(task.order_list)
        planner_task.arena_layout = []

        for src_station in task.arena_layout:
            dst_station = Station()
            dst_station.station_name = str(getattr(src_station, 'station_name', ''))
            dst_station.station_type = int(src_station.station_type)
            dst_station.station_id = amr_station_to_planner_station(
                int(src_station.station_id),
                self.side,
            )
            dst_station.material_ids = list(src_station.material_ids)
            planner_task.arena_layout.append(dst_station)

            self.get_logger().info(
                '[PLANNER] station 계산용 변환: '
                f'amr_id={int(src_station.station_id)} -> planner_id={dst_station.station_id}, '
                f'type={dst_station.station_type}, materials={list(dst_station.material_ids)}'
            )

        return planner_task

    def _steps_to_amr_station_ids(self, steps):
        if self.side == 'a':
            return steps

        for step in steps:
            old_station = int(step.station_id)
            new_station = planner_station_to_amr_station(old_station, self.side)
            step.station_id = int(new_station)
            if old_station != new_station:
                self.get_logger().info(
                    '[PLANNER] step station AMR용 복원: '
                    f'step={step.step_id}, planner_station={old_station} -> amr_station={new_station}'
                )
        return steps

    def task_callback(self, task):
        if self.plan_generated:
            return

        self.plan_generated = True
        self.get_logger().info('Task 수신 → Plan D 계획 생성 시작')

        try:
            planner_task = self._task_for_planner_coordinates(task)
            planned_steps = self.planner.build_plan(planner_task)
            self.steps = self._steps_to_amr_station_ids(planned_steps)
        except Exception as exc:
            self.steps = []
            self.plan_generated = False
            self.get_logger().error(f'Plan D 계획 생성 실패: {exc}')

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
