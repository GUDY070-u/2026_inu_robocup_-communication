"""Plan C Step generation.

Plan C keeps the existing Step message and adds this semantic meaning:
    Step.AMR + Step.PRODUCE -> AMR internal assembly while moving to station_id
                              object_ids[0] is the produced product_id

Important limitation:
    This file can express dependency-level parallelism. If the manager executes
    Step[] strictly by list order instead of dependency readiness, WB/AMR
    overlap will still be limited by the manager implementation.
"""

from collections import OrderedDict

from sml_msgs.msg import Order, Step

from .planner_config import MAX_RAW_CAPACITY, STATION_START_GOAL


class StepGeneratorMixin:
    def _generate_steps(
        self,
        task_sequence,
        recycle_orders,
        wb_id,
        customer_id,
        storage_id,
    ):
        """Compatibility entry point with the Plan B file structure."""
        return self._generate_plan_c_steps(
            task_sequence, recycle_orders, wb_id, customer_id, storage_id
        )

    def _generate_plan_c_steps(
        self,
        task_sequence,
        recycle_orders,
        wb_id,
        customer_id,
        storage_id,
    ):
        steps = []
        step_id = 0
        current_station = STATION_START_GOAL
        last_wb_step_id = None

        self._plan_c_recycle_step_ids = {}
        self._plan_c_waste_returned = set()
        self._plan_c_product_ready_at_wb = {}

        for task in task_sequence:
            if task['order_type'] == Order.OT_RECYCLE:
                step_id, current_station, last_wb_step_id = \
                    self._append_plan_c_recycle_task(
                        steps, step_id, task,
                        wb_id, customer_id,
                        current_station, last_wb_step_id,
                    )

                # Standalone recycle waste can be returned immediately.
                # Recycle orders used by later PRODUCE are cleaned up after
                # the dependent PRODUCE has had a chance to use the WB materials.
                if not task.get('has_dependent_produce', False):
                    step_id, current_station = self._append_plan_c_waste_return(
                        steps, step_id, task, wb_id,
                        current_station, last_wb_step_id,
                    )

            elif task['order_type'] == Order.OT_PRODUCE:
                if self._plan_c_can_produce_on_amr(task):
                    self.get_logger().info(
                        f'[PLAN-C] PRODUCE {task["product_id"]} -> AMR 조립 선택'
                    )
                    step_id, current_station = self._append_plan_c_amr_produce_task(
                        steps, step_id, task,
                        wb_id, customer_id,
                        current_station, last_wb_step_id,
                    )
                else:
                    self.get_logger().info(
                        f'[PLAN-C] PRODUCE {task["product_id"]} -> WB 조립 선택'
                    )
                    step_id, current_station, last_wb_step_id = \
                        self._append_plan_c_wb_produce_task(
                            steps, step_id, task,
                            wb_id, customer_id,
                            current_station, last_wb_step_id,
                        )

        # Return remaining recycle leftovers that were delayed because the
        # recycle output was partially reused by PRODUCE.
        for recycle_task in recycle_orders:
            if id(recycle_task) in self._plan_c_waste_returned:
                continue
            step_id, current_station = self._append_plan_c_waste_return(
                steps, step_id, recycle_task, wb_id,
                current_station, last_wb_step_id,
            )

        if step_id > 0:
            steps.append(self._make_step(
                step_id, Step.AMR, Step.GOAL,
                [], STATION_START_GOAL, [step_id - 1]
            ))

        return steps

    def _generate_plan_c_recycle_only_pipeline_steps(
        self,
        recycle_orders,
        wb_id,
        customer_id,
        storage_id,
    ):
        """Generate dependency-aware RECYCLE-only pipeline steps.

        The AMR loads the next recycle product from CUSTOMER without waiting for
        the current WB RECYCLE result. Actual WB interaction waits on the WB
        RECYCLE completion through depends_on.
        """
        ordered_recycles = sorted(
            recycle_orders,
            key=lambda ro: (len(ro.get('materials', [])), int(ro['product_id']))
        )

        steps = []
        step_id = 0
        self._plan_c_recycle_step_ids = {}
        self._plan_c_waste_returned = set()
        self._plan_c_product_ready_at_wb = {}

        if not ordered_recycles:
            return steps

        first = ordered_recycles[0]
        load_sid = step_id
        steps.append(self._make_step(
            step_id, Step.AMR, Step.LOAD,
            [first['product_id']], customer_id, []
        ))
        step_id += 1

        unload_sid = step_id
        steps.append(self._make_step(
            step_id, Step.AMR, Step.UNLOAD,
            [first['product_id']], wb_id, [load_sid]
        ))
        step_id += 1

        prev_wb_sid = step_id
        steps.append(self._make_step(
            step_id, Step.WB, Step.RECYCLE,
            [first['product_id']], wb_id, [unload_sid]
        ))
        self._plan_c_recycle_step_ids[id(first)] = prev_wb_sid
        step_id += 1

        # AMR is free after the product has been handed to WB.
        amr_available_dep = unload_sid
        prev_recycle = first

        for next_recycle in ordered_recycles[1:]:
            # WB 분해 완료 여부와 상관없이 다음 분해 완성품을 CUSTOMER에서 수거.
            next_load_sid = step_id
            steps.append(self._make_step(
                step_id, Step.AMR, Step.LOAD,
                [next_recycle['product_id']], customer_id, [amr_available_dep]
            ))
            step_id += 1

            # 다음 완성품을 싣고 WB 쪽으로 이동해 대기한 것으로 보고,
            # 실제 WB 상호작용은 prev_wb_sid 완료 이후에만 수행.
            if prev_recycle.get('waste_items'):
                waste_load_sid = step_id
                waste_objects = [
                    int(item['object_id'])
                    for item in prev_recycle['waste_items']
                ]
                steps.append(self._make_step(
                    step_id, Step.AMR, Step.LOAD,
                    waste_objects, wb_id, [prev_wb_sid, next_load_sid]
                ))
                step_id += 1
                wb_ready_for_next_dep = waste_load_sid
            else:
                wb_ready_for_next_dep = self._latest_dep(prev_wb_sid, next_load_sid)

            next_unload_sid = step_id
            steps.append(self._make_step(
                step_id, Step.AMR, Step.UNLOAD,
                [next_recycle['product_id']], wb_id, [wb_ready_for_next_dep]
            ))
            step_id += 1

            next_wb_sid = step_id
            steps.append(self._make_step(
                step_id, Step.WB, Step.RECYCLE,
                [next_recycle['product_id']], wb_id, [next_unload_sid]
            ))
            self._plan_c_recycle_step_ids[id(next_recycle)] = next_wb_sid
            step_id += 1

            # WB가 다음 주문을 분해하는 동안 이전 waste를 원재료 station으로 반납.
            if prev_recycle.get('waste_items'):
                step_id, amr_available_dep = self._append_loaded_waste_unloads(
                    steps, step_id, prev_recycle,
                    start_dep=next_unload_sid,
                    start_station=wb_id,
                )
                self._plan_c_waste_returned.add(id(prev_recycle))
            else:
                amr_available_dep = next_unload_sid
                self._plan_c_waste_returned.add(id(prev_recycle))

            prev_recycle = next_recycle
            prev_wb_sid = next_wb_sid

        # Final recycle waste return.
        if prev_recycle.get('waste_items'):
            waste_objects = [
                int(item['object_id'])
                for item in prev_recycle['waste_items']
            ]
            final_load_sid = step_id
            steps.append(self._make_step(
                step_id, Step.AMR, Step.LOAD,
                waste_objects, wb_id, [prev_wb_sid, amr_available_dep]
            ))
            step_id += 1
            step_id, amr_available_dep = self._append_loaded_waste_unloads(
                steps, step_id, prev_recycle,
                start_dep=final_load_sid,
                start_station=wb_id,
            )
            self._plan_c_waste_returned.add(id(prev_recycle))
        else:
            amr_available_dep = prev_wb_sid
            self._plan_c_waste_returned.add(id(prev_recycle))

        if step_id > 0:
            steps.append(self._make_step(
                step_id, Step.AMR, Step.GOAL,
                [], STATION_START_GOAL, [amr_available_dep]
            ))

        return steps

    # ------------------------------------------------------------------
    # Individual task appenders
    # ------------------------------------------------------------------

    def _append_plan_c_recycle_task(
        self,
        steps,
        step_id,
        task,
        wb_id,
        customer_id,
        current_station,
        last_wb_step_id,
    ):
        product_id = int(task['product_id'])

        if task.get('source_after_produce', False):
            ready_sid = self._plan_c_product_ready_at_wb.get(product_id)
            deps = self._unique_depends(ready_sid, last_wb_step_id)
            wb_sid = step_id
            steps.append(self._make_step(
                step_id, Step.WB, Step.RECYCLE,
                [product_id], wb_id, deps
            ))
            self._plan_c_recycle_step_ids[id(task)] = wb_sid
            self.get_logger().info(
                f'[PLAN-C] RECYCLE {product_id} -> WB 분해 선택 (생산 결과물 사용)'
            )
            return step_id + 1, wb_id, wb_sid

        load_sid = step_id
        steps.append(self._make_step(
            step_id, Step.AMR, Step.LOAD,
            [product_id], customer_id, []
        ))
        step_id += 1

        unload_deps = self._unique_depends(load_sid, last_wb_step_id)
        unload_sid = step_id
        steps.append(self._make_step(
            step_id, Step.AMR, Step.UNLOAD,
            [product_id], wb_id, unload_deps
        ))
        step_id += 1

        wb_sid = step_id
        steps.append(self._make_step(
            step_id, Step.WB, Step.RECYCLE,
            [product_id], wb_id, [unload_sid]
        ))
        self._plan_c_recycle_step_ids[id(task)] = wb_sid
        self.get_logger().info(f'[PLAN-C] RECYCLE {product_id} -> WB 분해 선택')
        return step_id + 1, wb_id, wb_sid

    def _append_plan_c_amr_produce_task(
        self,
        steps,
        step_id,
        task,
        wb_id,
        customer_id,
        current_station,
        last_wb_step_id,
    ):
        load_sids = []

        # 1) Load initial stock materials from storage/hybrid stations.
        initial_sources = self._group_initial_sources(task)
        ordered_sources = self._order_sources_by_travel(initial_sources, current_station)
        for source, object_ids in ordered_sources:
            load_sid = step_id
            steps.append(self._make_step(
                step_id, Step.AMR, Step.LOAD,
                object_ids, source, []
            ))
            load_sids.append(load_sid)
            current_station = source
            step_id += 1

        # 2) Load reused materials that were generated by WB recycle.
        wb_materials_by_dep = self._group_wb_reuse_sources(task)
        if wb_materials_by_dep:
            for dep_recycle, object_ids in wb_materials_by_dep.items():
                recycle_sid = self._plan_c_recycle_step_ids.get(id(dep_recycle))
                deps = self._unique_depends(recycle_sid, last_wb_step_id)
                load_sid = step_id
                steps.append(self._make_step(
                    step_id, Step.AMR, Step.LOAD,
                    object_ids, wb_id, deps
                ))
                load_sids.append(load_sid)
                current_station = wb_id
                step_id += 1

        if len(self._plan_c_all_material_objects(task)) > MAX_RAW_CAPACITY:
            raise RuntimeError(
                f'PRODUCE {task["product_id"]} 재료 수가 AMR raw slot '
                f'{MAX_RAW_CAPACITY}개를 초과합니다'
            )

        # 3) AMR assembly is executed while driving to the next required
        #    destination.
        #
        #    Normal PRODUCE:
        #        station_id = CUSTOMER station.
        #        Manager should start NAV to CUSTOMER and AMR assembly together,
        #        then mark this Step complete only after both are done.
        #
        #    Lifecycle PRODUCE followed by RECYCLE:
        #        station_id = WB station, because the product must be handed to WB.
        if task.get('has_following_recycle', False):
            produce_destination = wb_id
        else:
            produce_destination = customer_id

        produce_sid = step_id
        produce_deps = self._unique_depends(*load_sids)
        steps.append(self._make_step(
            step_id, Step.AMR, Step.PRODUCE,
            [task['product_id']], produce_destination, produce_deps
        ))
        current_station = produce_destination
        self.get_logger().info(
            f'[PLAN-C] AMR PRODUCE {task["product_id"]}는 '
            f'station {produce_destination} 이동 중 조립으로 계획'
        )
        step_id += 1

        if task.get('has_following_recycle', False):
            # AMR PRODUCE step already includes moving to WB while assembling.
            # After that, only unload the completed product to WB.
            unload_deps = self._unique_depends(produce_sid, last_wb_step_id)
            ready_sid = step_id
            steps.append(self._make_step(
                step_id, Step.AMR, Step.UNLOAD,
                [task['product_id']], wb_id, unload_deps
            ))
            self._plan_c_product_ready_at_wb[int(task['product_id'])] = ready_sid
            current_station = wb_id
            step_id += 1
        else:
            # AMR PRODUCE step already includes moving to CUSTOMER while assembling.
            # After that, only unload the completed product to CUSTOMER.
            deliver_sid = step_id
            steps.append(self._make_step(
                step_id, Step.AMR, Step.UNLOAD,
                [task['product_id']], customer_id, [produce_sid]
            ))
            current_station = customer_id
            step_id += 1

        return step_id, current_station

    def _append_plan_c_wb_produce_task(
        self,
        steps,
        step_id,
        task,
        wb_id,
        customer_id,
        current_station,
        last_wb_step_id,
    ):
        load_sids = []

        initial_sources = self._group_initial_sources(task)
        ordered_sources = self._order_sources_by_travel(initial_sources, current_station)
        for source, object_ids in ordered_sources:
            load_sid = step_id
            steps.append(self._make_step(
                step_id, Step.AMR, Step.LOAD,
                object_ids, source, []
            ))
            load_sids.append(load_sid)
            current_station = source
            step_id += 1

        unload_sid = None
        all_initial_objects = []
        for object_ids in initial_sources.values():
            all_initial_objects.extend(object_ids)

        if all_initial_objects:
            unload_deps = self._unique_depends(*load_sids, last_wb_step_id)
            unload_sid = step_id
            steps.append(self._make_step(
                step_id, Step.AMR, Step.UNLOAD,
                all_initial_objects, wb_id, unload_deps
            ))
            current_station = wb_id
            step_id += 1

        recycle_dep_sids = []
        for dep_recycle in self._plan_c_recycle_deps(task):
            recycle_sid = self._plan_c_recycle_step_ids.get(id(dep_recycle))
            if recycle_sid is not None:
                recycle_dep_sids.append(recycle_sid)

        wb_depends = self._unique_depends(unload_sid, *recycle_dep_sids, last_wb_step_id)
        wb_sid = step_id
        steps.append(self._make_step(
            step_id, Step.WB, Step.PRODUCE,
            [task['product_id']], wb_id, wb_depends
        ))
        last_wb_step_id = wb_sid
        step_id += 1

        if task.get('has_following_recycle', False):
            # Product remains at WB for the following lifecycle RECYCLE.
            self._plan_c_product_ready_at_wb[int(task['product_id'])] = wb_sid
        else:
            load_product_sid = step_id
            steps.append(self._make_step(
                step_id, Step.AMR, Step.LOAD,
                [task['product_id']], wb_id, [wb_sid]
            ))
            step_id += 1

            deliver_sid = step_id
            steps.append(self._make_step(
                step_id, Step.AMR, Step.UNLOAD,
                [task['product_id']], customer_id, [load_product_sid]
            ))
            current_station = customer_id
            step_id += 1

        return step_id, current_station, last_wb_step_id

    def _append_plan_c_waste_return(
        self,
        steps,
        step_id,
        recycle_task,
        wb_id,
        current_station,
        last_wb_step_id,
    ):
        if id(recycle_task) in self._plan_c_waste_returned:
            return step_id, current_station

        if not recycle_task.get('waste_items'):
            self._plan_c_waste_returned.add(id(recycle_task))
            return step_id, current_station

        recycle_sid = self._plan_c_recycle_step_ids.get(id(recycle_task))
        deps = self._unique_depends(recycle_sid, last_wb_step_id)
        waste_objects = [int(item['object_id']) for item in recycle_task['waste_items']]

        load_sid = step_id
        steps.append(self._make_step(
            step_id, Step.AMR, Step.LOAD,
            waste_objects, wb_id, deps
        ))
        step_id += 1

        step_id, _last_dep = self._append_loaded_waste_unloads(
            steps, step_id, recycle_task,
            start_dep=load_sid,
            start_station=wb_id,
        )
        self._plan_c_waste_returned.add(id(recycle_task))
        return step_id, self._last_waste_target_station(recycle_task, default=current_station)

    # ------------------------------------------------------------------
    # Grouping helpers
    # ------------------------------------------------------------------

    def _group_initial_sources(self, produce_task):
        grouped = {}
        for (_, source, dep_recycle, object_id, token_ref) in produce_task.get('material_sources', []):
            if dep_recycle is not None:
                continue
            if not isinstance(source, int):
                continue
            self._add_grouped_object(grouped, source, object_id, token_ref)
        return self._clean_grouped_objects(grouped)

    def _group_wb_reuse_sources(self, produce_task):
        grouped = OrderedDict()
        for (_, _source, dep_recycle, object_id, _) in produce_task.get('material_sources', []):
            if dep_recycle is None:
                continue
            grouped.setdefault(dep_recycle, []).append(object_id)
        return grouped

    def _plan_c_recycle_deps(self, produce_task):
        deps = []
        for (_, _, dep_recycle, _, _) in produce_task.get('material_sources', []):
            if dep_recycle is not None and dep_recycle not in deps:
                deps.append(dep_recycle)
        return deps

    def _plan_c_all_material_objects(self, produce_task):
        return [
            object_id
            for (_, _, _, object_id, _) in produce_task.get('material_sources', [])
        ]

    def _append_loaded_waste_unloads(
        self,
        steps,
        step_id,
        recycle_task,
        start_dep,
        start_station,
    ):
        waste_by_station = self._group_waste_items_by_station(
            recycle_task.get('waste_items', [])
        )
        current_station = start_station
        last_dep = start_dep
        for target_station, object_ids in self._order_sources_by_travel(
                waste_by_station, current_station):
            unload_sid = step_id
            steps.append(self._make_step(
                step_id, Step.AMR, Step.UNLOAD,
                object_ids, target_station, [last_dep]
            ))
            last_dep = unload_sid
            current_station = target_station
            step_id += 1
        return step_id, last_dep

    def _last_waste_target_station(self, recycle_task, default=None):
        items = recycle_task.get('waste_items', [])
        if not items:
            return default
        return int(items[-1]['station_id'])

    def _latest_dep(self, *deps):
        valid = [dep for dep in deps if dep is not None]
        if not valid:
            return None
        return max(valid)

    def _unique_depends(self, *depends):
        result = []
        for dep in depends:
            if dep is None:
                continue
            if dep not in result:
                result.append(dep)
        return result

    # ------------------------------------------------------------------
    # Generic Step helpers
    # ------------------------------------------------------------------

    def _make_step(self, step_id, type_, action, object_ids, station_id, depends_on):
        step = Step()
        step.step_id = int(step_id)
        step.type = type_
        step.action = action
        step.object_ids = list(object_ids)
        step.station_id = station_id if station_id is not None else -1
        # 중복 dependency 제거, 순서 유지
        step.depends_on = list(dict.fromkeys(depends_on))
        return step

    def _add_grouped_object(self, grouped, station_id, object_id, token_ref):
        """같은 raw 사용 token은 중복 방지하고, batch에서 분해된 raw 중복은 허용한다."""
        items = grouped.setdefault(station_id, [])
        refs = grouped.setdefault((station_id, '_refs'), set())

        if token_ref is not None:
            if token_ref in refs:
                return
            refs.add(token_ref)

        items.append(object_id)

    def _clean_grouped_objects(self, grouped):
        return {k: v for k, v in grouped.items() if not isinstance(k, tuple)}

    def _append_slot_object(self, slot_objects, slot_token_refs, object_id, token_ref):
        if token_ref is not None:
            if token_ref in slot_token_refs:
                return
            slot_token_refs.add(token_ref)
        slot_objects.append(object_id)

    def _group_waste_items_by_station(self, waste_items):
        grouped = {}
        seen_refs = set()

        for item in waste_items:
            station_id = int(item['station_id'])
            token_ref = item['token_ref']
            object_id = int(item['object_id'])
            key = (station_id, token_ref)

            if key in seen_refs:
                continue

            seen_refs.add(key)
            grouped.setdefault(station_id, []).append(object_id)

        return grouped

    def _find_wb_recycle_step_id(self, steps, product_id):
        for step in steps:
            if step.type == Step.WB and step.action == Step.RECYCLE:
                if product_id in step.object_ids:
                    return step.step_id
        return None

