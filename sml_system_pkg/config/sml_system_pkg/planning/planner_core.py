"""Pure Plan C planner core.

Plan C supports AMR-side assembly for allowed products while keeping WB-only
assembly and all RECYCLE operations on the fixed workbench.
"""

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
    """Coordinates the complete Plan C planning pipeline."""

    def __init__(self, config=None, station_coords=None, logger=None):
        self.config = config or PlannerConfig()
        self.station_coords = station_coords or {}
        self._logger = logger or _NullLogger()

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

        virtual_stock_tokens = copy.deepcopy(stock_tokens)
        recycle_releases = self._register_recycle_releases(recycle_orders)

        # Plan C: 스테이션 재고를 먼저 쓰고, 부족한 재료만 RECYCLE 결과로 수급한다.
        self._assign_material_sources_plan_c(
            produce_orders, recycle_releases, virtual_stock_tokens, wb_id
        )
        self._assign_waste_materials(
            recycle_orders, produce_orders,
            copy.deepcopy(waste_target_tokens), storage_id, wb_id
        )

        if recycle_orders and not produce_orders:
            steps = self._generate_plan_c_recycle_only_pipeline_steps(
                recycle_orders, wb_id, customer_id, storage_id
            )
            self.get_logger().info(
                f'[PLAN-C] 분해-only 파이프라인 계획 생성 완료: {len(steps)}개 스텝'
            )
        else:
            task_sequence = self._build_plan_c_sequence(
                produce_orders, recycle_orders, wb_id, customer_id
            )
            steps = self._generate_steps(
                task_sequence, recycle_orders,
                wb_id, customer_id, storage_id
            )
            self.get_logger().info(f'[PLAN-C] 계획 생성 완료: {len(steps)}개 스텝')
            self._log_plan_c_sequence(task_sequence)

        self._log_material_model(material_model)
        self._log_plan_c_summary(produce_orders, recycle_orders)
        self._log_steps(steps)
        return steps

    def _assign_material_sources_plan_c(
        self,
        produce_orders,
        recycle_available,
        stock_tokens,
        wb_id,
    ):
        """Assign material sources for Plan C.

        Difference from the default allocator:
        - Prefer initial station stock first.
        - Use RECYCLE output only when stock cannot supply the material.
        """
        for order in produce_orders:
            for material in order['materials']:
                found = self._find_in_stock(material, stock_tokens, wb_id, order)
                if found is not None:
                    station_id, object_id, token_ref = found
                    order['material_sources'].append(
                        (material, station_id, None, object_id, token_ref)
                    )
                    continue

                if material in recycle_available and recycle_available[material]:
                    recycle_order = recycle_available[material].pop(0)
                    order['material_sources'].append(
                        (material, 'WB', recycle_order, material, None)
                    )
                    continue

                raise RuntimeError(f'[PLAN-C] 재료 {material}를 구할 수 없음')
