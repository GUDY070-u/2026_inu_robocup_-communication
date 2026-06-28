"""Workbench task sequence planning."""

from sml_plan_d_msgs.msg import Order


class SequencePlannerMixin:
    def _build_wb_sequence(self, produce_orders, recycle_orders, wb_id, customer_id):
        """
        1. 재고만으로 가능한 PRODUCE 먼저
        2. RECYCLE 필요한 PRODUCE → 해당 RECYCLE과 묶어서
        3. 독립 RECYCLE (어떤 PRODUCE와도 무관) → 마지막
        4. PRODUCE 결과물 RECYCLE → 맨 마지막
        """
        produce_ids = {o['product_id'] for o in produce_orders}
        after_recycle_ids = {
            ro['product_id'] for ro in recycle_orders
            if ro.get('source_after_produce', False)
        }
        for po in produce_orders:
            po['has_following_recycle'] = po['product_id'] in after_recycle_ids

        # 각 PRODUCE가 의존하는 RECYCLE 파악
        produce_recycle_deps = {}
        for po in produce_orders:
            deps = []
            for (material, source, dep_recycle, object_id, token_ref) in po['material_sources']:
                if dep_recycle is not None and dep_recycle not in deps:
                    deps.append(dep_recycle)
            produce_recycle_deps[id(po)] = deps

        after_recycles      = []
        standalone_recycles = []

        for ro in recycle_orders:
            if ro.get('source_after_produce', False):
                after_recycles.append(ro)
            else:
                is_linked = any(
                    ro in deps for deps in produce_recycle_deps.values()
                )
                if not is_linked:
                    standalone_recycles.append(ro)

        wb_sequence   = []
        used_recycles = set()

        stock_only_produces = [
            po for po in produce_orders if not produce_recycle_deps[id(po)]
        ]
        linked_produces = [
            po for po in produce_orders if produce_recycle_deps[id(po)]
        ]

        if self.use_time_cost:
            stock_only_produces.sort(
                key=lambda po: self._estimate_task_cost(po, wb_id, customer_id)
            )
            linked_produces.sort(
                key=lambda po: self._estimate_task_cost(po, wb_id, customer_id)
            )
            standalone_recycles.sort(
                key=lambda ro: self._estimate_task_cost(ro, wb_id, customer_id)
            )
            after_recycles.sort(
                key=lambda ro: self._estimate_task_cost(ro, wb_id, customer_id)
            )

        # 1. 재고만으로 가능한 PRODUCE
        wb_sequence.extend(stock_only_produces)

        # 2. RECYCLE 필요한 PRODUCE → 해당 RECYCLE과 묶음
        for po in linked_produces:
            deps = produce_recycle_deps[id(po)]
            if self.use_time_cost:
                deps.sort(key=lambda ro: self._estimate_task_cost(ro, wb_id, customer_id))
            for ro in deps:
                if id(ro) not in used_recycles:
                    wb_sequence.append(ro)
                    used_recycles.add(id(ro))
            wb_sequence.append(po)

        # 3. 독립 RECYCLE
        wb_sequence.extend(standalone_recycles)

        # 4. PRODUCE 결과물 RECYCLE
        wb_sequence.extend(after_recycles)

        if self.use_time_cost:
            for task in wb_sequence:
                cost = self._estimate_task_cost(task, wb_id, customer_id)
                self.get_logger().info(
                    f'비용 기반 선택: {self._task_label(task)} | '
                    f'estimated_cost={cost:.2f}s'
                )

        return wb_sequence

    # --------------------------------------------------------
    # Step 4: 스텝 시퀀스 생성

