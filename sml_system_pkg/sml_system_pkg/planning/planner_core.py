"""Pure planner core. This module has no ROS node/subscription/service code."""

import copy

from .arena_parser import ArenaParserMixin
from .cost_model import CostModelMixin
from .material_allocator import MaterialAllocatorMixin
from .order_parser import OrderParserMixin
from .plan_logger import PlanLoggerMixin
from .planner_config import PlannerConfig
from .sequence_planner import SequencePlannerMixin
from .step_generator import StepGeneratorMixin


class _NullLogger:
    def info(self, msg):
        pass

    def warn(self, msg):
        pass

    def error(self, msg):
        pass


class PlannerCore(
    OrderParserMixin,
    ArenaParserMixin,
    MaterialAllocatorMixin,
    SequencePlannerMixin,
    CostModelMixin,
    StepGeneratorMixin,
    PlanLoggerMixin,
):
    """Coordinates the complete planning pipeline."""

    def __init__(self, config=None, station_coords=None, logger=None):
        self.config = config or PlannerConfig()
        self.station_coords = station_coords or {}
        self._logger = logger or _NullLogger()

        # Backward-compatible attributes used by the extracted methods.
        self.use_time_cost = bool(self.config.use_time_cost)
        self.amr_speed_mps = float(self.config.amr_speed_mps)

    def get_logger(self):
        return self._logger

    def build_plan(self, task):
        """Build and return a list of sml_msgs/Step from a sml_msgs/Task."""
        produce_orders, recycle_orders = self._parse_orders(task.order_list)

        material_model = self._build_material_model(
            produce_orders, recycle_orders
        )

        station_items, stock_tokens, waste_target_tokens, wb_id, customer_id, storage_id = \
            self._parse_arena(task.arena_layout, material_model)

        # stock_tokens는 예약하면서 remaining을 차감하므로 deepcopy해서 사용
        virtual_stock_tokens = copy.deepcopy(stock_tokens)
        recycle_releases = self._register_recycle_releases(recycle_orders)
        self._assign_material_sources(
            produce_orders, recycle_releases, virtual_stock_tokens, wb_id
        )
        self._assign_waste_materials(
            recycle_orders, produce_orders,
            copy.deepcopy(waste_target_tokens), storage_id, wb_id
        )

        wb_sequence = self._build_wb_sequence(
            produce_orders, recycle_orders, wb_id, customer_id
        )

        steps = self._generate_steps(
            wb_sequence, station_items,
            wb_id, customer_id, storage_id
        )

        self.get_logger().info(f'계획 생성 완료: {len(steps)}개 스텝')
        self._log_cost_summary(wb_sequence, wb_id, customer_id)
        self._log_material_model(material_model)
        self._log_plan_summary(produce_orders, recycle_orders)
        self._log_steps(steps)

        return steps
