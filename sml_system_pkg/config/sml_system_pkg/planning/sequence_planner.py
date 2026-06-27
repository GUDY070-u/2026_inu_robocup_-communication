"""Plan C task classification and coarse sequence planning."""

from sml_msgs.msg import Order

from .planner_config import (
    AMR_ASSEMBLY_ALLOWED_FLOOR_RAW_IDS,
    AMR_ASSEMBLY_MATERIAL_ORDER,
    AMR_PRODUCE_EXTRA_ALLOWED_PRODUCT_IDS,
    WB_ONLY_PRODUCT_IDS,
)


class SequencePlannerMixin:
    """Classify PRODUCE tasks and build a coarse Plan C sequence."""

    def _plan_c_material_order(self, order):
        product_id = int(order['product_id'])
        return list(
            AMR_ASSEMBLY_MATERIAL_ORDER.get(
                product_id,
                order.get('materials', []),
            )
        )

    def _plan_c_can_produce_on_amr(self, order):
        """Return True if a PRODUCE order can be assembled inside AMR."""
        product_id = int(order['product_id'])

        if product_id in WB_ONLY_PRODUCT_IDS:
            return False

        if product_id in AMR_PRODUCE_EXTRA_ALLOWED_PRODUCT_IDS:
            return True

        materials = self._plan_c_material_order(order)
        return all(
            int(material) in AMR_ASSEMBLY_ALLOWED_FLOOR_RAW_IDS
            for material in materials
        )

    def _plan_c_produce_place(self, order):
        return 'AMR' if self._plan_c_can_produce_on_amr(order) else 'WB'

    def _build_wb_sequence(self, produce_orders, recycle_orders, wb_id, customer_id):
        """Compatibility name used by old planner code."""
        return self._build_plan_c_sequence(
            produce_orders, recycle_orders, wb_id, customer_id
        )

    def _build_plan_c_sequence(self, produce_orders, recycle_orders, wb_id, customer_id):
        """
        Plan C coarse sequence.

        Priority:
        1. RECYCLE orders required by PRODUCE material dependencies
        2. Stock-only PRODUCE orders
        3. PRODUCE orders that depend on RECYCLE output
        4. Standalone RECYCLE orders
        5. PRODUCE-result RECYCLE orders from lifecycle orders
        """
        after_recycle_ids = {
            ro['product_id'] for ro in recycle_orders
            if ro.get('source_after_produce', False)
        }
        for po in produce_orders:
            po['has_following_recycle'] = po['product_id'] in after_recycle_ids
            po['plan_c_place'] = self._plan_c_produce_place(po)

        produce_deps = {}
        for po in produce_orders:
            deps = []
            for (_, _, dep_recycle, _, _) in po.get('material_sources', []):
                if dep_recycle is not None and dep_recycle not in deps:
                    deps.append(dep_recycle)
            produce_deps[id(po)] = deps

        linked_recycles = []
        linked_recycle_ids = set()
        for po in produce_orders:
            for ro in produce_deps[id(po)]:
                if id(ro) not in linked_recycle_ids:
                    linked_recycles.append(ro)
                    linked_recycle_ids.add(id(ro))
                    ro['has_dependent_produce'] = True

        stock_only_produces = [
            po for po in produce_orders if not produce_deps[id(po)]
        ]
        linked_produces = [
            po for po in produce_orders if produce_deps[id(po)]
        ]
        standalone_recycles = [
            ro for ro in recycle_orders
            if id(ro) not in linked_recycle_ids
            and not ro.get('source_after_produce', False)
        ]
        after_recycles = [
            ro for ro in recycle_orders
            if ro.get('source_after_produce', False)
        ]

        if self.use_time_cost:
            stock_only_produces.sort(
                key=lambda po: self._estimate_task_cost(po, wb_id, customer_id)
            )
            linked_produces.sort(
                key=lambda po: self._estimate_task_cost(po, wb_id, customer_id)
            )
            # 분해는 원재료 개수가 적은 순으로 처리.
            linked_recycles.sort(
                key=lambda ro: (len(ro.get('materials', [])), int(ro['product_id']))
            )
            standalone_recycles.sort(
                key=lambda ro: (len(ro.get('materials', [])), int(ro['product_id']))
            )
            after_recycles.sort(
                key=lambda ro: (len(ro.get('materials', [])), int(ro['product_id']))
            )

        sequence = []
        sequence.extend(linked_recycles)
        sequence.extend(stock_only_produces)
        sequence.extend(linked_produces)
        sequence.extend(standalone_recycles)
        sequence.extend(after_recycles)
        return sequence

    def _log_plan_c_sequence(self, task_sequence):
        self.get_logger().info('===== Plan C 작업 순서 =====')
        for index, task in enumerate(task_sequence):
            if task['order_type'] == Order.OT_PRODUCE:
                place = task.get('plan_c_place') or self._plan_c_produce_place(task)
                self.get_logger().info(
                    f'{index + 1}. PRODUCE@{place} {task["product_id"]}'
                )
            elif task['order_type'] == Order.OT_RECYCLE:
                self.get_logger().info(
                    f'{index + 1}. RECYCLE@WB {task["product_id"]}'
                )
            else:
                self.get_logger().info(
                    f'{index + 1}. UNKNOWN {task.get("product_id")}'
                )
        self.get_logger().info('==========================')
