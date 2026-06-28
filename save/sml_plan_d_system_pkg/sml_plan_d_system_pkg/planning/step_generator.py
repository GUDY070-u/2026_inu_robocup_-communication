"""Generate sml_msgs/Step sequences from planned workbench tasks."""

from sml_plan_d_msgs.msg import Order, Step

from .planner_config import MAX_RAW_CAPACITY, STATION_START_GOAL


class StepGeneratorMixin:
    def _generate_steps(
        self, wb_sequence, station_items,
        wb_id, customer_id, storage_id
    ):
        steps = []
        step_id = 0
        last_wb_step_id = None

        slot_1           = None
        slot_material    = []
        slot_token_refs  = set()
        pending_loads    = []
        loaded_sources   = set()  # (produce_order_id, material_index)
        current_station  = STATION_START_GOAL

        for wb_index, wb_task in enumerate(wb_sequence):

            # ------------------------------------------------
            # RECYCLE: 분해 대상은 무조건 CUSTOMER에서 Load
            # ------------------------------------------------
            if wb_task['order_type'] == Order.OT_RECYCLE:

                if slot_1 is not None:
                    step_id, last_wb_step_id = self._flush_unload(
                        steps, step_id, pending_loads,
                        slot_1, slot_material, wb_id, last_wb_step_id
                    )
                    slot_1          = None
                    slot_material   = []
                    slot_token_refs = set()
                    pending_loads   = []

                if not wb_task.get('source_after_produce', False):
                    steps.append(self._make_step(
                        step_id, Step.AMR, Step.LOAD,
                        [wb_task['product_id']], customer_id, []
                    ))
                    pending_loads.append(step_id)
                    slot_1 = wb_task['product_id']
                    current_station = customer_id
                    step_id += 1
                else:
                    # 단일 OT_LIFECYCLE order에서 생산 결과물을 WB에 그대로 두고 recycle하는 경우
                    slot_1 = None

                # 다음 PRODUCE 재료 미리 적재 (초기 재고에서 가져올 수 있는 것만)
                preload_by_station = self._collect_future_produce_preloads(
                    wb_sequence, wb_index,
                    slot_material, slot_token_refs, loaded_sources
                )

                if self._clean_grouped_objects(preload_by_station):
                    self.get_logger().info(
                        f'[PRELOAD] {self._task_label(wb_task)} 처리 중 '
                        f'다음 PRODUCE 재료 추가 적재: '
                        f'{self._clean_grouped_objects(preload_by_station)}'
                    )

                ordered_sources = self._order_sources_by_travel(
                    self._clean_grouped_objects(preload_by_station), current_station
                )
                for source, object_ids in ordered_sources:
                    steps.append(self._make_step(
                        step_id, Step.AMR, Step.LOAD,
                        object_ids, source, []
                    ))
                    pending_loads.append(step_id)
                    current_station = source
                    step_id += 1

            # ------------------------------------------------
            # PRODUCE: 재료 Load
            # ------------------------------------------------
            elif wb_task['order_type'] == Order.OT_PRODUCE:

                needs_wb_material = any(
                    dep is not None
                    for (_, _, dep, _, _) in wb_task['material_sources']
                )

                if needs_wb_material and pending_loads:
                    step_id, last_wb_step_id = self._flush_unload(
                        steps, step_id, pending_loads,
                        slot_1, slot_material, wb_id, last_wb_step_id
                    )
                    slot_1          = None
                    slot_material   = []
                    slot_token_refs = set()
                    pending_loads   = []

                load_by_station = {}

                # 1) 현재 PRODUCE에 필요한 초기 재고 재료를 먼저 적재한다.
                for index, (material, source, dep, object_id, token_ref) in enumerate(
                        wb_task['material_sources']):
                    source_key = (id(wb_task), index)
                    if dep is None and source_key not in loaded_sources:
                        self._add_grouped_object(
                            load_by_station, source, object_id, token_ref
                        )
                        self._append_slot_object(
                            slot_material, slot_token_refs, object_id, token_ref
                        )
                        loaded_sources.add(source_key)

                # 2) AMR raw 적재공간에 여유가 있으면 다음 PRODUCE 재료도 미리 적재한다.
                #    이미 WB에 내려둔 재료는 loaded_sources로 표시되므로, 해당 주문 차례에서는
                #    다시 LOAD하지 않고 WB 작업만 수행한다.
                preload_by_station = self._collect_future_produce_preloads(
                    wb_sequence, wb_index,
                    slot_material, slot_token_refs, loaded_sources
                )
                if self._clean_grouped_objects(preload_by_station):
                    self.get_logger().info(
                        f'[PRELOAD] {self._task_label(wb_task)} 처리 중 '
                        f'다음 PRODUCE 재료 추가 적재: '
                        f'{self._clean_grouped_objects(preload_by_station)}'
                    )

                for station_id, object_ids in self._clean_grouped_objects(
                        preload_by_station).items():
                    for object_id in object_ids:
                        self._add_grouped_object(
                            load_by_station, station_id, object_id, None
                        )

                ordered_sources = self._order_sources_by_travel(
                    self._clean_grouped_objects(load_by_station), current_station
                )
                for source, object_ids in ordered_sources:
                    steps.append(self._make_step(
                        step_id, Step.AMR, Step.LOAD,
                        object_ids, source, []
                    ))
                    pending_loads.append(step_id)
                    current_station = source
                    step_id += 1

            # ------------------------------------------------
            # WB Unload
            # ------------------------------------------------
            all_objects = (
                ([slot_1] if slot_1 is not None else []) + slot_material
            )

            if all_objects:
                unload_depends = list(pending_loads)
                if last_wb_step_id is not None:
                    unload_depends.append(last_wb_step_id)

                unload_step_id = step_id
                steps.append(self._make_step(
                    step_id, Step.AMR, Step.UNLOAD,
                    all_objects, wb_id, unload_depends
                ))
                current_station = wb_id
                step_id += 1
            else:
                unload_step_id = None

            # ------------------------------------------------
            # WB 작업 (PRODUCE or RECYCLE)
            # ------------------------------------------------
            wb_action  = Step.PRODUCE \
                if wb_task['order_type'] == Order.OT_PRODUCE else Step.RECYCLE
            # PRODUCE: 완성품 ID / RECYCLE: 분해 대상 ID
            wb_objects = [wb_task['product_id']]

            wb_depends = []
            if unload_step_id is not None:
                wb_depends.append(unload_step_id)
            if last_wb_step_id is not None:
                wb_depends.append(last_wb_step_id)

            if wb_task['order_type'] == Order.OT_PRODUCE:
                for (_, _, dep_recycle, _, _) in wb_task['material_sources']:
                    if dep_recycle is not None:
                        recycle_sid = self._find_wb_recycle_step_id(
                            steps, dep_recycle['product_id']
                        )
                        if recycle_sid is not None and recycle_sid not in wb_depends:
                            wb_depends.append(recycle_sid)

            steps.append(self._make_step(
                step_id, Step.WB, wb_action,
                wb_objects, wb_id, wb_depends
            ))
            last_wb_step_id = step_id
            step_id += 1

            # ------------------------------------------------
            # PRODUCE 완료 후: 완성품 납품
            # ------------------------------------------------
            if wb_task['order_type'] == Order.OT_PRODUCE:
                if wb_task.get('has_following_recycle', False):
                    self.get_logger().info(
                        f'PRODUCE {wb_task["product_id"]} 결과물은 lifecycle RECYCLE을 위해 WB에 유지'
                    )
                else:
                    load_sid = step_id
                    steps.append(self._make_step(
                        step_id, Step.AMR, Step.LOAD,
                        [wb_task['product_id']], wb_id, [last_wb_step_id]
                    ))
                    step_id += 1
                    steps.append(self._make_step(
                        step_id, Step.AMR, Step.UNLOAD,
                        [wb_task['product_id']], customer_id, [load_sid]
                    ))
                    current_station = customer_id
                    step_id += 1

            # ------------------------------------------------
            # RECYCLE 완료 후: 불필요 재료를 target station에 반납
            # ------------------------------------------------
            if wb_task['order_type'] == Order.OT_RECYCLE \
                    and wb_task['waste_items']:
                waste_by_station = self._group_waste_items_by_station(
                    wb_task['waste_items']
                )
                ordered_waste_targets = self._order_sources_by_travel(
                    waste_by_station, wb_id
                )
                for target_station, object_ids in ordered_waste_targets:
                    load_sid = step_id
                    steps.append(self._make_step(
                        step_id, Step.AMR, Step.LOAD,
                        object_ids, wb_id, [last_wb_step_id]
                    ))
                    current_station = wb_id
                    step_id += 1
                    steps.append(self._make_step(
                        step_id, Step.AMR, Step.UNLOAD,
                        object_ids, target_station, [load_sid]
                    ))
                    current_station = target_station
                    step_id += 1

            slot_1          = None
            slot_material   = []
            slot_token_refs = set()
            pending_loads   = []

        # ------------------------------------------------
        # 모든 작업 완료 후: AMR이 START/GOAL(00)으로 복귀
        # ------------------------------------------------
        if step_id > 0:
            last_step_id = step_id - 1
            steps.append(self._make_step(
                step_id, Step.AMR, Step.GOAL,
                [], STATION_START_GOAL, [last_step_id]
            ))
            step_id += 1

        return steps

    # --------------------------------------------------------
    # 헬퍼 함수
    # --------------------------------------------------------

    def _collect_future_produce_preloads(
        self, wb_sequence, current_index,
        slot_material, slot_token_refs, loaded_sources
    ):
        """
        AMR raw 적재공간에 남는 칸이 있으면 뒤쪽 PRODUCE 재료를 미리 적재한다.

        조건
        - 현재 WB task 이후의 PRODUCE만 대상
        - 초기 재고에서 가져올 수 있는 재료(dep is None)만 대상
        - RECYCLE 후 WB에서 재사용해야 하는 재료는 미리 LOAD하지 않음
        - MAX_RAW_CAPACITY를 넘지 않음
        - 같은 token_ref는 중복 적재하지 않음
        """
        preload_by_station = {}

        for future_task in wb_sequence[current_index + 1:]:
            if future_task['order_type'] != Order.OT_PRODUCE:
                continue

            for index, (material, source, dep, object_id, token_ref) in enumerate(
                    future_task['material_sources']):
                if len(slot_material) >= MAX_RAW_CAPACITY:
                    return preload_by_station

                source_key = (id(future_task), index)

                if dep is not None:
                    continue
                if not isinstance(source, int):
                    continue
                if source_key in loaded_sources:
                    continue

                self._add_grouped_object(
                    preload_by_station, source, object_id, token_ref
                )
                self._append_slot_object(
                    slot_material, slot_token_refs, object_id, token_ref
                )
                loaded_sources.add(source_key)

        return preload_by_station

    def _make_step(self, step_id, type_, action, object_ids, station_id, depends_on, slide_ids=None):
        step = Step()
        step.step_id    = step_id
        step.type       = type_
        step.action     = action
        step.object_ids = list(object_ids)
        step.station_id = station_id if station_id is not None else -1
        step.depends_on = list(depends_on)
        step.slide_ids = list(slide_ids or [])
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
            station_id = item['station_id']
            token_ref = item['token_ref']
            object_id = item['object_id']
            key = (station_id, token_ref)
            if key in seen_refs:
                continue
            seen_refs.add(key)
            grouped.setdefault(station_id, []).append(object_id)
        return grouped

    def _flush_unload(
        self, steps, step_id, pending_loads,
        slot_1, slot_material, wb_id, last_wb_step_id
    ):
        unload_depends = list(pending_loads)
        if last_wb_step_id is not None:
            unload_depends.append(last_wb_step_id)
        all_objects = (([slot_1] if slot_1 is not None else []) + slot_material)
        steps.append(self._make_step(
            step_id, Step.AMR, Step.UNLOAD,
            all_objects, wb_id, unload_depends
        ))
        return step_id + 1, last_wb_step_id

    def _find_wb_recycle_step_id(self, steps, product_id):
        for step in steps:
            if step.type == Step.WB and step.action == Step.RECYCLE:
                if product_id in step.object_ids:
                    return step.step_id
        return None

    # --------------------------------------------------------
    # 로그

