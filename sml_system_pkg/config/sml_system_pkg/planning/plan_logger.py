"""Planner log formatting helpers for Plan C."""

from sml_msgs.msg import Order, Step

from .planner_config import PRODUCT_NAMES


class PlanLoggerMixin:
    def _task_label(self, task):
        if task['order_type'] == Order.OT_PRODUCE:
            place = task.get('plan_c_place')
            if place:
                return f'PRODUCE@{place} {task["product_id"]}'
            return f'PRODUCE {task["product_id"]}'
        if task['order_type'] == Order.OT_RECYCLE:
            return f'RECYCLE@WB {task["product_id"]}'
        return f'UNKNOWN {task["product_id"]}'

    def _log_cost_summary(self, wb_sequence, wb_id, customer_id):
        self.get_logger().info('===== 시간 비용 기반 작업 순서 =====')

        for index, task in enumerate(wb_sequence):
            cost = self._estimate_task_cost(task, wb_id, customer_id)
            self.get_logger().info(
                f'{index + 1}. {self._task_label(task)} | '
                f'estimated_cost={cost:.2f}s'
            )

        self.get_logger().info('=================================')

    def _log_material_model(self, model):
        self.get_logger().info('===== lifecycle material model =====')
        self.get_logger().info(f'P produce_materials : {model["produce_materials"]}')
        self.get_logger().info(f'R recycle_materials : {model["recycle_materials"]}')
        self.get_logger().info(f'C common_reuse      : {model["common_reuse"]}')
        self.get_logger().info(f'P-C initial         : {model["produce_initial"]}')
        self.get_logger().info(f'R-C leftover        : {model["recycle_leftover"]}')
        self.get_logger().info('====================================')

    def _log_plan_summary(self, produce_orders, recycle_orders):
        """Compatibility name."""
        return self._log_plan_c_summary(produce_orders, recycle_orders)

    def _log_plan_c_summary(self, produce_orders, recycle_orders):
        def name(pid):
            return PRODUCT_NAMES.get(int(pid), str(pid))

        self.get_logger().info('===== 실행 계획 요약 =====')

        for order in recycle_orders:
            pid = int(order['product_id'])
            self.get_logger().info(f'[RECYCLE@WB] {pid} ({name(pid)})')
            if order.get('source_after_produce', False):
                self.get_logger().info('  -> PRODUCE 결과물을 WB에서 바로 분해')
            else:
                self.get_logger().info('  -> CUSTOMER에서 완성품 LOAD 가정')
            self.get_logger().info(f'  -> 분해 후: {order["materials"]}')

            if order.get('reuse_materials'):
                reuse_info = []
                reuse_remaining = list(order['reuse_materials'])
                for po in produce_orders:
                    for (material, _source, dep_recycle, _object_id, _token_ref) in po['material_sources']:
                        if dep_recycle is order and material in reuse_remaining:
                            reuse_remaining.remove(material)
                            reuse_info.append(
                                f'{material} -> PRODUCE {po["product_id"]}'
                            )
                self.get_logger().info(
                    f'  -> reuse : {order["reuse_materials"]}  ({" / ".join(reuse_info)})'
                )

            if order.get('waste_materials'):
                waste_targets = [
                    f'{item["object_id"]}->station {item["station_id"]}'
                    for item in order.get('waste_items', [])
                ]
                self.get_logger().info(
                    f'  -> waste : {order["waste_materials"]}  ({" / ".join(waste_targets)})'
                )

        for order in produce_orders:
            pid = int(order['product_id'])
            place = order.get('plan_c_place') or self._plan_c_produce_place(order)
            self.get_logger().info(f'[PRODUCE@{place}] {pid} ({name(pid)})')
            self.get_logger().info(f'  -> 재료: {order["materials"]}')
            if place == 'AMR':
                if order.get('has_following_recycle', False):
                    self.get_logger().info('  -> 조립 위치: AMR 내부 조립공간')
                    self.get_logger().info('  -> 조립 타이밍: WB 이동 중 조립')
                else:
                    self.get_logger().info('  -> 조립 위치: AMR 내부 조립공간')
                    self.get_logger().info('  -> 조립 타이밍: CUSTOMER 이동 중 조립')
            else:
                self.get_logger().info('  -> 조립 위치: WB 작업스테이션')

            for (material, source, dep_recycle, object_id, _token_ref) in order['material_sources']:
                if dep_recycle is not None:
                    self.get_logger().info(
                        f'  -> {material} : RECYCLE {dep_recycle["product_id"]} 후 WB에서 재사용'
                    )
                else:
                    self.get_logger().info(
                        f'  -> {material} : station={source} 에서 object_id={object_id} Load'
                    )

        self.get_logger().info('==========================')

    def _log_steps(self, steps):
        type_map = {Step.AMR: 'AMR', Step.WB: 'WB '}
        action_map = {
            Step.LOAD:    'LOAD   ',
            Step.UNLOAD:  'UNLOAD ',
            Step.PRODUCE: 'PRODUCE',
            Step.RECYCLE: 'RECYCLE',
            Step.GOAL:    'GOAL   ',
        }
        self.get_logger().info('===== 스텝 시퀀스 =====')
        for s in steps:
            self.get_logger().info(
                f'[{s.step_id:2d}] {type_map.get(s.type, s.type)} | '
                f'{action_map.get(s.action, s.action)} | '
                f'objects={list(s.object_ids)} | '
                f'station={s.station_id} | '
                f'depends_on={list(s.depends_on)}'
            )
        self.get_logger().info('======================')
