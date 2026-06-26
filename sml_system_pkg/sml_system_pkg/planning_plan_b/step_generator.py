"""Generate sml_msgs/Step sequences from planned workbench tasks.

Plan B 개선 사항:
1. 현재 주문 재료를 우선 적재한다.
2. 다음 주문 preload는 현재 주문 수집 경로에 포함된 station에서만 허용한다.
3. 그 station의 다음 주문 재료를 전부 실을 수 있을 때만 preload한다.
4. 현재 주문 재료를 WB에 하역한 뒤, WB 조립 중 AMR이 다음 주문의 남은 재료를 수집한다.
5. 이전 완성품을 WB에서 수거한 뒤 다음 주문 재료를 WB에 하역하고, 그 뒤 다음 WB 작업을 시작한다.
"""

from sml_msgs.msg import Order, Step

from .planner_config import MAX_RAW_CAPACITY, STATION_START_GOAL


class StepGeneratorMixin:
    def _generate_steps(
        self, wb_sequence, station_items,
        wb_id, customer_id, storage_id
    ):
        steps = []
        step_id = 0

        # last_wb_step_id는 단순히 "마지막 WB 작업"이 아니라,
        # 다음 WB 작업이 시작되기 전에 반드시 끝나야 하는 barrier로 사용한다.
        # 예:
        # - 이전 WB 생산 완료 step
        # - 이전 완성품을 WB에서 LOAD해서 WB를 비운 step
        # - 다음 주문 재료를 WB에 UNLOAD한 step
        last_wb_step_id = None

        slot_1 = None
        slot_material = []
        slot_token_refs = set()
        pending_loads = []
        loaded_sources = set()  # (produce_order_id, material_index)
        current_station = STATION_START_GOAL

        for wb_index, wb_task in enumerate(wb_sequence):

            # ------------------------------------------------
            # RECYCLE: 기존 방식 유지
            # ------------------------------------------------
            if wb_task['order_type'] == Order.OT_RECYCLE:

                if slot_1 is not None:
                    step_id, last_wb_step_id = self._flush_unload(
                        steps, step_id, pending_loads,
                        slot_1, slot_material, wb_id, last_wb_step_id
                    )
                    slot_1 = None
                    slot_material = []
                    slot_token_refs = set()
                    pending_loads = []

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

                wb_depends = []
                if unload_step_id is not None:
                    wb_depends.append(unload_step_id)
                if last_wb_step_id is not None:
                    wb_depends.append(last_wb_step_id)

                steps.append(self._make_step(
                    step_id, Step.WB, Step.RECYCLE,
                    [wb_task['product_id']], wb_id, wb_depends
                ))
                last_wb_step_id = step_id
                step_id += 1

                if wb_task['waste_items']:
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

                slot_1 = None
                slot_material = []
                slot_token_refs = set()
                pending_loads = []
                continue

            # ------------------------------------------------
            # PRODUCE
            # ------------------------------------------------
            if wb_task['order_type'] != Order.OT_PRODUCE:
                continue

            next_produce_task = self._get_immediate_next_produce(
                wb_sequence, wb_index
            )

            needs_wb_material = any(
                dep is not None
                for (_, _, dep, _, _) in wb_task['material_sources']
            )

            if needs_wb_material and pending_loads:
                step_id, last_wb_step_id = self._flush_unload(
                    steps, step_id, pending_loads,
                    slot_1, slot_material, wb_id, last_wb_step_id
                )
                slot_1 = None
                slot_material = []
                slot_token_refs = set()
                pending_loads = []

            load_by_station = {}

            # ------------------------------------------------
            # 1) 현재 PRODUCE에 필요한 초기 재고 재료를 먼저 적재
            # ------------------------------------------------
            for index, (material, source, dep, object_id, token_ref) in enumerate(
                    wb_task['material_sources']):

                source_key = (id(wb_task), index)

                if dep is not None:
                    continue
                if not isinstance(source, int):
                    continue
                if source_key in loaded_sources:
                    continue

                self._add_grouped_object(
                    load_by_station, source, object_id, token_ref
                )
                self._append_slot_object(
                    slot_material, slot_token_refs, object_id, token_ref
                )
                loaded_sources.add(source_key)

            # 현재 주문 때문에 실제 방문하는 station 집합
            current_route_stations = set(
                self._clean_grouped_objects(load_by_station).keys()
            )

            # ------------------------------------------------
            # 2) 현재 경로에 있는 station에서만 다음 주문 재료 preload
            # ------------------------------------------------
            if next_produce_task is not None:
                preload_by_station = self._collect_route_reducing_preloads(
                    next_produce_task,
                    current_route_stations,
                    slot_material,
                    slot_token_refs,
                    loaded_sources,
                )

                if self._clean_grouped_objects(preload_by_station):
                    self.get_logger().info(
                        f'[ROUTE_PRELOAD] {self._task_label(wb_task)} 처리 중 '
                        f'다음 PRODUCE {next_produce_task["product_id"]}의 '
                        f'경로 단축 preload: '
                        f'{self._clean_grouped_objects(preload_by_station)}'
                    )

                for station_id, object_ids in self._clean_grouped_objects(
                        preload_by_station).items():
                    for object_id in object_ids:
                        self._add_grouped_object(
                            load_by_station, station_id, object_id, None
                        )

            # ------------------------------------------------
            # 3) 현재 주문 재료 + 허용된 preload 재료 LOAD
            # ------------------------------------------------
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
            # 4) WB에 현재 주문 재료 + preload 재료 하역
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
            # 5) 현재 PRODUCE WB 작업
            # ------------------------------------------------
            wb_depends = []
            if unload_step_id is not None:
                wb_depends.append(unload_step_id)
            if last_wb_step_id is not None:
                wb_depends.append(last_wb_step_id)

            for (_, _, dep_recycle, _, _) in wb_task['material_sources']:
                if dep_recycle is not None:
                    recycle_sid = self._find_wb_recycle_step_id(
                        steps, dep_recycle['product_id']
                    )
                    if recycle_sid is not None and recycle_sid not in wb_depends:
                        wb_depends.append(recycle_sid)

            current_wb_step_id = step_id
            steps.append(self._make_step(
                step_id, Step.WB, Step.PRODUCE,
                [wb_task['product_id']], wb_id, wb_depends
            ))
            step_id += 1

            # 일단 현재 WB 작업 완료를 barrier로 둔다.
            last_wb_step_id = current_wb_step_id

            # ------------------------------------------------
            # 6) WB가 현재 주문을 조립하는 동안,
            #    AMR이 다음 PRODUCE의 남은 재료를 수집
            # ------------------------------------------------
            future_load_sids = []
            future_material_objects = []

            if next_produce_task is not None:
                next_remaining_by_station, future_material_objects = \
                    self._collect_remaining_produce_loads(
                        next_produce_task,
                        loaded_sources,
                    )

                if self._clean_grouped_objects(next_remaining_by_station):
                    self.get_logger().info(
                        f'[PIPELINE_LOAD] WB가 PRODUCE {wb_task["product_id"]} '
                        f'작업 중 다음 PRODUCE {next_produce_task["product_id"]} '
                        f'남은 재료 수집: '
                        f'{self._clean_grouped_objects(next_remaining_by_station)}'
                    )

                # 현재 주문 재료를 WB에 하역한 뒤부터 다음 주문 남은 재료 수집 가능
                load_depends = []
                if unload_step_id is not None:
                    load_depends.append(unload_step_id)
                elif wb_depends:
                    load_depends.extend(wb_depends)

                ordered_next_sources = self._order_sources_by_travel(
                    self._clean_grouped_objects(next_remaining_by_station),
                    current_station,
                )

                prev_load_sid = None
                for source, object_ids in ordered_next_sources:
                    depends = list(load_depends)
                    if prev_load_sid is not None:
                        depends.append(prev_load_sid)

                    load_sid = step_id
                    steps.append(self._make_step(
                        step_id, Step.AMR, Step.LOAD,
                        object_ids, source, depends
                    ))
                    future_load_sids.append(load_sid)
                    prev_load_sid = load_sid
                    current_station = source
                    step_id += 1

            # ------------------------------------------------
            # 7) 현재 PRODUCE 완료 후 완성품 처리
            # ------------------------------------------------
            if wb_task.get('has_following_recycle', False):
                self.get_logger().info(
                    f'PRODUCE {wb_task["product_id"]} 결과물은 '
                    f'lifecycle RECYCLE을 위해 WB에 유지'
                )
                last_wb_step_id = current_wb_step_id

            else:
                # 7-1) 완성품을 WB에서 LOAD한다.
                # 이 step이 끝나면 WB 위의 이전 완성품은 제거된 것으로 본다.
                product_load_sid = step_id
                steps.append(self._make_step(
                    step_id, Step.AMR, Step.LOAD,
                    [wb_task['product_id']], wb_id, [current_wb_step_id]
                ))
                current_station = wb_id
                step_id += 1

                # 7-2) 다음 주문 재료를 이미 AMR이 수집했다면,
                #      완성품을 WB에서 들어 올린 뒤 바로 다음 주문 재료를 WB에 하역한다.
                #      이렇게 해야 다음 WB 작업이 제품 배송과 병렬로 시작될 수 있다.
                future_unload_sid = None
                if future_material_objects:
                    future_unload_depends = [product_load_sid]
                    future_unload_depends.extend(future_load_sids)

                    future_unload_sid = step_id
                    steps.append(self._make_step(
                        step_id, Step.AMR, Step.UNLOAD,
                        future_material_objects, wb_id, future_unload_depends
                    ))
                    current_station = wb_id
                    step_id += 1

                    self.get_logger().info(
                        f'[PIPELINE_UNLOAD] 다음 PRODUCE '
                        f'{next_produce_task["product_id"]} 재료 '
                        f'{future_material_objects}를 WB에 선하역'
                    )

                    # 다음 WB 작업은 이 재료 하역 이후 시작 가능
                    last_wb_step_id = future_unload_sid
                    product_delivery_dep = future_unload_sid

                else:
                    # 다음 재료 선하역이 없다면, 다음 WB 작업은
                    # 이전 완성품이 WB에서 제거된 시점 이후 가능
                    last_wb_step_id = product_load_sid
                    product_delivery_dep = product_load_sid

                # 7-3) 완성품은 고객 station으로 납품
                steps.append(self._make_step(
                    step_id, Step.AMR, Step.UNLOAD,
                    [wb_task['product_id']], customer_id, [product_delivery_dep]
                ))
                current_station = customer_id
                step_id += 1

            # ------------------------------------------------
            # 8) 현재 task slot 초기화
            # ------------------------------------------------
            slot_1 = None
            slot_material = []
            slot_token_refs = set()
            pending_loads = []

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
    # 새 preload / pipeline helper
    # --------------------------------------------------------

    def _get_immediate_next_produce(self, wb_sequence, current_index):
        """바로 다음 WB task가 PRODUCE이면 반환한다."""
        next_index = current_index + 1
        if next_index >= len(wb_sequence):
            return None

        next_task = wb_sequence[next_index]
        if next_task['order_type'] == Order.OT_PRODUCE:
            return next_task

        return None

    def _collect_route_reducing_preloads(
        self,
        next_task,
        current_route_stations,
        slot_material,
        slot_token_refs,
        loaded_sources,
    ):
        """
        현재 주문 수집 경로에 포함된 station에서만 다음 주문 재료를 preload한다.

        허용 조건:
        - 현재 주문 재료를 다 싣고도 공간이 남아야 함
        - 다음 주문 재료의 source station이 현재 경로에 포함되어야 함
        - 해당 station의 다음 주문 재료를 전부 실을 수 있어야 함
        - 그래야 다음 주문에서 그 station 방문이 제거됨
        """
        preload_by_station = {}

        capacity_left = MAX_RAW_CAPACITY - len(slot_material)
        if capacity_left <= 0:
            return preload_by_station

        next_by_station = {}

        for index, (material, source, dep, object_id, token_ref) in enumerate(
                next_task['material_sources']):

            source_key = (id(next_task), index)

            if dep is not None:
                continue
            if not isinstance(source, int):
                continue
            if source_key in loaded_sources:
                continue

            next_by_station.setdefault(source, []).append(
                (index, object_id, token_ref)
            )

        candidate_stations = [
            station_id for station_id in current_route_stations
            if station_id in next_by_station
        ]

        for station_id in candidate_stations:
            items = next_by_station[station_id]

            # 일부만 실으면 다음 주문에서 해당 station을 다시 방문해야 하므로 preload하지 않음
            if len(items) > capacity_left:
                self.get_logger().info(
                    f'[ROUTE_PRELOAD] station {station_id} 후보 제외 | '
                    f'필요={len(items)}개, 남은 적재공간={capacity_left}개'
                )
                continue

            for index, object_id, token_ref in items:
                source_key = (id(next_task), index)

                self._add_grouped_object(
                    preload_by_station, station_id, object_id, token_ref
                )
                self._append_slot_object(
                    slot_material, slot_token_refs, object_id, token_ref
                )
                loaded_sources.add(source_key)

            capacity_left = MAX_RAW_CAPACITY - len(slot_material)

            self.get_logger().info(
                f'[ROUTE_PRELOAD] 다음 PRODUCE {next_task["product_id"]}의 '
                f'station {station_id} 방문 제거 preload: '
                f'{[object_id for _, object_id, _ in items]}'
            )

            if capacity_left <= 0:
                break

        return preload_by_station

    def _collect_remaining_produce_loads(self, produce_task, loaded_sources):
        """
        다음 PRODUCE에서 아직 WB에 선하역되지 않은 초기 재고 재료를 수집 대상으로 만든다.
        이 함수에서 loaded_sources에 등록하므로, 해당 PRODUCE 차례에서는 다시 LOAD하지 않는다.
        """
        load_by_station = {}
        slot_objects = []
        slot_token_refs = set()

        for index, (material, source, dep, object_id, token_ref) in enumerate(
                produce_task['material_sources']):

            source_key = (id(produce_task), index)

            if dep is not None:
                continue
            if not isinstance(source, int):
                continue
            if source_key in loaded_sources:
                continue

            if len(slot_objects) >= MAX_RAW_CAPACITY:
                self.get_logger().warn(
                    f'[PIPELINE_LOAD] 다음 PRODUCE {produce_task["product_id"]} '
                    f'재료가 AMR raw capacity={MAX_RAW_CAPACITY}를 초과합니다. '
                    f'초과 재료는 현재 pipeline 수집에서 제외됩니다.'
                )
                break

            self._add_grouped_object(
                load_by_station, source, object_id, token_ref
            )
            self._append_slot_object(
                slot_objects, slot_token_refs, object_id, token_ref
            )
            loaded_sources.add(source_key)

        return load_by_station, slot_objects

    # --------------------------------------------------------
    # 기존 helper
    # --------------------------------------------------------

    def _make_step(self, step_id, type_, action, object_ids, station_id, depends_on):
        step = Step()
        step.step_id = step_id
        step.type = type_
        step.action = action
        step.object_ids = list(object_ids)
        step.station_id = station_id if station_id is not None else -1
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
    # 과거 preload 함수는 남겨두되, 새 Plan B에서는 사용하지 않는다.
    # --------------------------------------------------------

    def _collect_future_produce_preloads(
        self, wb_sequence, current_index,
        slot_material, slot_token_refs, loaded_sources
    ):
        """
        Deprecated:
        기존 함수는 현재 주문 경로와 무관하게 다음 PRODUCE 재료를 가능한 만큼 preload했다.
        새 Plan B에서는 _collect_route_reducing_preloads()를 사용한다.
        """
        return {}