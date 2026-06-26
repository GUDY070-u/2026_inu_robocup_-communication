"""Travel, process and task cost model."""

import math

from sml_msgs.msg import Order

from .planner_config import (
    ASSEMBLY_PAIR_TIME,
    DISASSEMBLY_PAIR_TIME,
    LOAD_TIME,
    STATION_START_GOAL,
    UNLOAD_TIME,
    VISION_TIME_AVG,
)


class CostModelMixin:
    def _station_coord(self, station_id):
        if station_id in self.station_coords:
            return self.station_coords[station_id]
        # 좌표가 없는 station은 id를 x좌표로 둔 fallback을 사용한다.
        return (float(station_id), 0.0)

    def _travel_distance(self, from_station, to_station):
        if from_station is None or to_station is None:
            return 0.0
        if from_station == to_station:
            return 0.0
        x1, y1 = self._station_coord(from_station)
        x2, y2 = self._station_coord(to_station)
        return math.hypot(x2 - x1, y2 - y1)

    def _travel_time(self, from_station, to_station):
        if not self.use_time_cost:
            return 0.0
        if from_station is None or to_station is None or from_station == to_station:
            return 0.0
        speed = max(self.amr_speed_mps, 1e-6)
        return self._travel_distance(from_station, to_station) / speed

    def _order_sources_by_travel(self, grouped, start_station):
        if not grouped:
            return []
        remaining = list(grouped.items())
        if not self.use_time_cost:
            return remaining

        ordered = []
        current = start_station
        while remaining:
            idx, item = min(
                enumerate(remaining),
                key=lambda pair: self._travel_time(current, pair[1][0])
            )
            station_id, object_ids = item
            ordered.append((station_id, object_ids))
            current = station_id
            remaining.pop(idx)
        return ordered

    def _estimate_task_cost(self, task, wb_id, customer_id):
        """원래 planner의 시간 비용 항목을 유지한 간단 추정값."""
        process_cost = self._process_cost(task)

        if task['order_type'] == Order.OT_RECYCLE:
            # 별도 OT_RECYCLE은 CUSTOMER에서 완성품을 가져온다고 가정한다.
            if task.get('source_after_produce', False):
                collect_cost = 0.0
            else:
                collect_cost = (
                    self._travel_time(customer_id, wb_id)
                    + VISION_TIME_AVG
                    + LOAD_TIME
                    + UNLOAD_TIME
                )

            waste_items = task.get('waste_items', [])
            waste_by_station = self._group_waste_items_by_station(waste_items)
            waste_cost = 0.0
            current = wb_id
            for target_station, object_ids in self._order_sources_by_travel(
                    waste_by_station, wb_id):
                n_items = len(object_ids)
                waste_cost += (
                    self._travel_time(current, target_station)
                    + n_items * LOAD_TIME
                    + n_items * UNLOAD_TIME
                )
                current = target_station

            return collect_cost + process_cost + waste_cost

        if task['order_type'] == Order.OT_PRODUCE:
            source_to_objects = {}
            for (_, source, dep_recycle, object_id, token_ref) in task.get('material_sources', []):
                if dep_recycle is not None or not isinstance(source, int):
                    continue
                self._add_grouped_object(source_to_objects, source, object_id, token_ref)

            collect_cost = 0.0
            current = STATION_START_GOAL
            for source, object_ids in self._order_sources_by_travel(
                    self._clean_grouped_objects(source_to_objects), current):
                n_items = len(object_ids)
                collect_cost += (
                    self._travel_time(current, source)
                    + VISION_TIME_AVG
                    + n_items * LOAD_TIME
                )
                current = source

            if source_to_objects:
                total_objects = sum(
                    len(v) for k, v in source_to_objects.items()
                    if not isinstance(k, tuple)
                )
                collect_cost += self._travel_time(current, wb_id)
                collect_cost += total_objects * UNLOAD_TIME

            deliver_cost = 0.0
            if customer_id is not None:
                deliver_cost = (
                    self._travel_time(wb_id, customer_id)
                    + LOAD_TIME
                    + UNLOAD_TIME
                )

            return collect_cost + process_cost + deliver_cost

        return float('inf')

    def _process_cost(self, task):
        n = len(task.get('materials', []))
        if task['order_type'] == Order.OT_PRODUCE:
            return max(0, n - 1) * ASSEMBLY_PAIR_TIME
        if task['order_type'] == Order.OT_RECYCLE:
            return max(0, n - 1) * DISASSEMBLY_PAIR_TIME

