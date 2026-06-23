"""
sml_planning_node.py

Task.msg를 수신해서 시간 비용 기반으로 작업 순서를 결정하고,
depends_on 기반 Step 시퀀스를 생성한 뒤 ManagerNode 요청 시 전달하는 노드.

반영 내용:
1. 주문 입력 순서를 그대로 따르지 않음
2. 생산 / 재활용 / 라이프사이클에 공통 적용 가능한 시간 비용 함수 추가
3. AMR 이동 속도, 비전 인식, 적재/하역, 조립/분해 예상 시간 반영
4. JSON 좌표 파일을 받아 station_id별 예상 좌표 사용
5. JSON 경로 기본값을 코드에 내장
6. 비용이 낮은 작업을 우선 선택하되, recycle -> produce 의존성 유지
7. lifecycle은 다음 두 방식 모두 대응
   - order_type == Order.OT_LIFECYCLE 또는 order_type == 3
   - 같은 product_id에 대해 PRODUCE와 RECYCLE이 동시에 존재하는 경우
8. AMR 적재 제약 분리
   - 원재료 슬롯: 최대 5개
   - 완성품/재활용 대상 제품 슬롯: 최대 1개
9. PRODUCE 작업에서도 남는 원재료 슬롯에 미래 PRODUCE 재료를 미리 적재
10. 현재 주문 때문에 이미 방문할 station의 미래 재료를 우선 적재
"""

import copy
import json
import math
import os

import rclpy
from rclpy.node import Node

from sml_msgs.msg import Task, Order, Station, Step
from sml_msgs.srv import GetPlan


PRODUCT_NAMES = {
    34: 'Battery',
    13: 'Magnet',
    81: 'E-Stop',
    442: 'Carrot',
    241: 'Traffic Light',
    462: 'Small Tree',
    711: 'Hammer',
    4482: 'Big Carrot',
    8518: 'Burger',
    48132: 'Ice Cream',
    46262: 'Big Tree',
}

# --------------------------------------------------------
# 경기 / 시스템 시간 가정값
# --------------------------------------------------------

AMR_SPEED = 13.0              # [m/s]
VISION_TIME_AVG = 2.0         # [s]  비전 인식 평균 1~3초 중 평균값
LOAD_TIME = 10.0              # [s/item]
UNLOAD_TIME = 10.0            # [s/item]
ASSEMBLY_PAIR_TIME = 12.5     # [s/connection] 10~15초 평균
DISASSEMBLY_PAIR_TIME = 12.5  # [s/disconnection] 임시로 조립과 동일 가정

# --------------------------------------------------------
# AMR 적재 제약
# --------------------------------------------------------

MAX_RAW_CAPACITY = 5
MAX_PRODUCT_CAPACITY = 1

# 대회 arena_layout에는 포함되지 않는 AMR의 시작/복귀 지점
STATION_START_GOAL = 0

# 현재 메인 Workbench로 사용할 station_id
MAIN_WORKBENCH_STATION_ID = 6

# ROS parameter 이름
# 중요: 여기에 파일 경로를 직접 넣으면 안 됨
STATION_COORD_JSON_PARAM = 'station_coord_json_path'

# JSON 좌표 파일 기본 경로
DEFAULT_STATION_COORD_JSON_PATH = (
    '/home/user/ros2_ws/src/sml_system_pkg/config/station_coordinates_a_zone.json'
)

# 메시지에 OT_LIFECYCLE이 없을 수도 있으므로 3을 fallback으로 사용
ORDER_TYPE_LIFECYCLE = getattr(Order, 'OT_LIFECYCLE', 3)


class PlanningNode(Node):

    def __init__(self):
        super().__init__('planning_node')

        self.plan_generated = False
        self.steps = []

        self.declare_parameter(
            STATION_COORD_JSON_PARAM,
            DEFAULT_STATION_COORD_JSON_PATH
        )

        self.station_coord_overrides = self._load_station_coord_json()

        self.task_sub = self.create_subscription(
            Task,
            '/sml/task',
            self.task_callback,
            10
        )

        self.plan_srv = self.create_service(
            GetPlan,
            '/sml/get_plan',
            self.get_plan_callback
        )

        self.get_logger().info('PlanningNode 시작')

    # --------------------------------------------------------
    # 콜백
    # --------------------------------------------------------

    def task_callback(self, task):
        if self.plan_generated:
            return

        self.plan_generated = True
        self.get_logger().info('Task 수신 → 시간 비용 기반 계획 생성 시작')
        self._build_plan(task)

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

    # --------------------------------------------------------
    # 핵심 계획 생성 로직
    # --------------------------------------------------------

    def _build_plan(self, task):
        try:
            station_materials, station_types, station_coords, wb_id, customer_id, storage_id = \
                self._parse_arena(task.arena_layout)

            produce_orders, recycle_orders = self._parse_orders(task.order_list)

            self._mark_recycle_after_produce(produce_orders, recycle_orders)

            virtual_stock = copy.deepcopy(station_materials)

            produced_product_ids = {o['product_id'] for o in produce_orders}

            recycle_releases = self._register_recycle_releases(
                recycle_orders,
                produced_product_ids
            )

            self._assign_material_sources(
                produce_orders,
                recycle_releases,
                virtual_stock
            )

            self._assign_waste_materials(recycle_orders, produce_orders)

            wb_sequence = self._build_wb_sequence(
                produce_orders,
                recycle_orders,
                station_materials,
                station_coords,
                wb_id,
                customer_id,
                storage_id
            )

            self.steps = self._generate_steps(
                wb_sequence,
                station_materials,
                station_coords,
                wb_id,
                customer_id,
                storage_id
            )

            self.get_logger().info(f'계획 생성 완료: {len(self.steps)}개 스텝')

            self._log_cost_summary(
                wb_sequence,
                station_materials,
                station_coords,
                wb_id,
                customer_id,
                storage_id
            )

            self._log_plan_summary(produce_orders, recycle_orders)
            self._log_steps(self.steps)

        except Exception as e:
            self.get_logger().error(f'계획 생성 실패: {e}')
            self.plan_generated = False

    # --------------------------------------------------------
    # Step 1: JSON 좌표 로딩
    # --------------------------------------------------------

    def _load_station_coord_json(self):
        path = self.get_parameter(
            STATION_COORD_JSON_PARAM
        ).get_parameter_value().string_value.strip()

        if not path:
            self.get_logger().warn(
                'station_coord_json_path가 비어 있습니다. '
                'arena_layout에 좌표가 없으면 이동 시간은 0으로 계산됩니다.'
            )
            return {}

        if not os.path.exists(path):
            self.get_logger().warn(
                f'station 좌표 JSON 파일을 찾을 수 없습니다: {path}'
            )
            return {}

        try:
            with open(path, 'r', encoding='utf-8') as f:
                data = json.load(f)

            coords = {}
            raw = None

            if isinstance(data, dict):
                if 'station_coordinates' in data:
                    raw = data['station_coordinates']
                elif 'stations' in data:
                    raw = data['stations']
                else:
                    raw = data

            if isinstance(raw, dict):
                for station_id_str, entry in raw.items():
                    station_id = int(station_id_str)

                    if isinstance(entry, dict):
                        x = entry.get('x')
                        y = entry.get('y')
                    elif isinstance(entry, list) and len(entry) >= 2:
                        x = entry[0]
                        y = entry[1]
                    else:
                        continue

                    if x is None or y is None:
                        continue

                    coords[station_id] = (float(x), float(y))

            elif isinstance(raw, list):
                for entry in raw:
                    if not isinstance(entry, dict):
                        continue

                    station_id = entry.get('station_id', entry.get('id'))
                    x = entry.get('x')
                    y = entry.get('y')

                    if station_id is None or x is None or y is None:
                        continue

                    coords[int(station_id)] = (float(x), float(y))

            self.get_logger().info(
                f'station 좌표 JSON 로드 완료: '
                f'{len(coords)}개 station, path={path}'
            )

            return coords

        except Exception as e:
            self.get_logger().error(f'station 좌표 JSON 로드 실패: {e}')
            return {}

    # --------------------------------------------------------
    # Step 2: 입력 파싱
    # --------------------------------------------------------

    def _parse_arena(self, arena_layout):
        station_materials = {}
        station_types = {}
        station_coords = {}

        wb_id = None
        customer_id = None
        storage_id = None

        for station in arena_layout:
            station_id = station.station_id

            station_materials[station_id] = list(station.material_ids)
            station_types[station_id] = station.station_type

            json_xy = self.station_coord_overrides.get(station_id)
            msg_xy = self._extract_station_xy(station)
            xy = json_xy if json_xy is not None else msg_xy

            if xy is not None:
                station_coords[station_id] = xy

            if (
                station.station_type == Station.ST_WORKBENCH
                and station.station_id == MAIN_WORKBENCH_STATION_ID
            ):
                wb_id = station.station_id

            elif station.station_type == Station.ST_CUSTOMER:
                customer_id = station.station_id

            elif station.station_type == Station.ST_STORAGE and storage_id is None:
                storage_id = station.station_id

        for station_id, xy in self.station_coord_overrides.items():
            if station_id not in station_coords:
                station_coords[station_id] = xy

        if STATION_START_GOAL not in station_coords:
            station_coords[STATION_START_GOAL] = None

        if wb_id is None:
            raise RuntimeError(
                f'메인 workbench station {MAIN_WORKBENCH_STATION_ID}가 '
                'arena_layout에 없습니다'
            )

        self.get_logger().info(
            f'arena 파싱 완료: stations={len(station_materials)}, '
            f'coords={len([v for v in station_coords.values() if v is not None])}'
        )

        return station_materials, station_types, station_coords, wb_id, customer_id, storage_id

    def _extract_station_xy(self, station):
        if hasattr(station, 'x') and hasattr(station, 'y'):
            return float(station.x), float(station.y)

        if hasattr(station, 'pos_x') and hasattr(station, 'pos_y'):
            return float(station.pos_x), float(station.pos_y)

        if hasattr(station, 'position'):
            pos = station.position
            if hasattr(pos, 'x') and hasattr(pos, 'y'):
                return float(pos.x), float(pos.y)

        if hasattr(station, 'pose'):
            pose = station.pose
            if hasattr(pose, 'position'):
                pos = pose.position
                if hasattr(pos, 'x') and hasattr(pos, 'y'):
                    return float(pos.x), float(pos.y)

        return None

    def _parse_orders(self, order_list):
        produce_orders = []
        recycle_orders = []

        for index, order in enumerate(order_list):
            base = {
                'uid': f'order_{index}',
                'order_type': order.order_type,
                'product_id': order.product_id,
                'materials': self._parse_product_id(order.product_id),
                'material_sources': [],
                'waste_materials': [],
                'reuse_materials': [],
                'source_after_produce': False,
                'is_lifecycle_part': False,
                'has_following_recycle': False,
            }

            if order.order_type == Order.OT_PRODUCE:
                produce_orders.append(base)

            elif order.order_type == Order.OT_RECYCLE:
                recycle_orders.append(base)

            elif order.order_type == ORDER_TYPE_LIFECYCLE:
                produce_part = copy.deepcopy(base)
                recycle_part = copy.deepcopy(base)

                produce_part['uid'] = f'order_{index}_life_produce'
                produce_part['order_type'] = Order.OT_PRODUCE
                produce_part['is_lifecycle_part'] = True

                recycle_part['uid'] = f'order_{index}_life_recycle'
                recycle_part['order_type'] = Order.OT_RECYCLE
                recycle_part['source_after_produce'] = True
                recycle_part['is_lifecycle_part'] = True

                produce_orders.append(produce_part)
                recycle_orders.append(recycle_part)

            else:
                self.get_logger().warn(
                    f'지원하지 않는 order_type={order.order_type}, '
                    f'product_id={order.product_id} → 무시'
                )

        return produce_orders, recycle_orders

    def _parse_product_id(self, product_id):
        return [int(d) for d in str(product_id)]

    def _mark_recycle_after_produce(self, produce_orders, recycle_orders):
        produce_ids = {o['product_id'] for o in produce_orders}

        for ro in recycle_orders:
            if ro['product_id'] in produce_ids:
                ro['source_after_produce'] = True

    # --------------------------------------------------------
    # Step 3: 시간 비용 함수
    # --------------------------------------------------------

    def _coord_of(self, station_id, station_coords):
        return station_coords.get(station_id)

    def _move_time_between(self, station_a, station_b, station_coords):
        if station_a is None or station_b is None:
            return 0.0

        if station_a == station_b:
            return 0.0

        a = self._coord_of(station_a, station_coords)
        b = self._coord_of(station_b, station_coords)

        if a is None or b is None:
            return 0.0

        dx = a[0] - b[0]
        dy = a[1] - b[1]
        dist = math.sqrt(dx * dx + dy * dy)

        return dist / AMR_SPEED

    def _route_move_time(self, route, station_coords):
        total = 0.0

        for i in range(len(route) - 1):
            total += self._move_time_between(route[i], route[i + 1], station_coords)

        return total

    def _greedy_route(self, start_id, source_ids, end_id, station_coords):
        route = [start_id]
        current = start_id
        remaining = list(dict.fromkeys(source_ids))

        while remaining:
            best = min(
                remaining,
                key=lambda sid: self._move_time_between(current, sid, station_coords)
            )
            route.append(best)
            current = best
            remaining.remove(best)

        if end_id is not None:
            route.append(end_id)

        return route

    def _logistics_cost(
        self,
        route,
        n_items,
        n_stations,
        station_coords,
        include_load=True,
        include_unload=True
    ):
        t_move = self._route_move_time(route, station_coords)
        t_vision = n_stations * VISION_TIME_AVG

        t_load = n_items * LOAD_TIME if include_load else 0.0
        t_unload = n_items * UNLOAD_TIME if include_unload else 0.0

        return t_move + t_vision + t_load + t_unload

    def _process_cost(self, task):
        n = len(task['materials'])

        if task['order_type'] == Order.OT_PRODUCE:
            return max(0, n - 1) * ASSEMBLY_PAIR_TIME

        if task['order_type'] == Order.OT_RECYCLE:
            return max(0, n - 1) * DISASSEMBLY_PAIR_TIME

        return 0.0

    def _estimate_task_cost(
        self,
        task,
        current_station,
        station_materials,
        station_coords,
        wb_id,
        customer_id,
        storage_id
    ):
        if task['order_type'] == Order.OT_PRODUCE:
            return self._estimate_produce_cost(
                task,
                current_station,
                station_coords,
                wb_id,
                customer_id
            )

        if task['order_type'] == Order.OT_RECYCLE:
            return self._estimate_recycle_cost(
                task,
                current_station,
                station_materials,
                station_coords,
                wb_id,
                storage_id
            )

        return float('inf')

    def _estimate_produce_cost(
        self,
        task,
        current_station,
        station_coords,
        wb_id,
        customer_id
    ):
        source_to_count = {}

        for material, source, dep in task['material_sources']:
            if dep is None and isinstance(source, int):
                source_to_count[source] = source_to_count.get(source, 0) + 1

        source_ids = list(source_to_count.keys())
        n_collect = sum(source_to_count.values())
        n_stations = len(source_ids)

        route = self._greedy_route(
            current_station,
            source_ids,
            wb_id,
            station_coords
        )

        collect_cost = self._logistics_cost(
            route=route,
            n_items=n_collect,
            n_stations=n_stations,
            station_coords=station_coords,
            include_load=True,
            include_unload=True
        )

        assembly_cost = self._process_cost(task)

        if task.get('has_following_recycle', False):
            deliver_cost = 0.0
        else:
            if customer_id is None:
                deliver_cost = 0.0
            else:
                deliver_route = [wb_id, customer_id]
                deliver_cost = self._logistics_cost(
                    route=deliver_route,
                    n_items=1,
                    n_stations=0,
                    station_coords=station_coords,
                    include_load=True,
                    include_unload=True
                )

        return collect_cost + assembly_cost + deliver_cost

    def _estimate_recycle_cost(
        self,
        task,
        current_station,
        station_materials,
        station_coords,
        wb_id,
        storage_id
    ):
        if task.get('source_after_produce', False):
            collect_product_cost = 0.0
        else:
            src = self._find_in_stock(task['product_id'], station_materials)

            if src is None:
                return float('inf')

            route = self._greedy_route(
                current_station,
                [src],
                wb_id,
                station_coords
            )

            collect_product_cost = self._logistics_cost(
                route=route,
                n_items=1,
                n_stations=1,
                station_coords=station_coords,
                include_load=True,
                include_unload=True
            )

        disassembly_cost = self._process_cost(task)

        waste_count = len(task.get('waste_materials', []))

        if waste_count > 0 and storage_id is not None:
            store_route = [wb_id, storage_id]
            store_waste_cost = self._logistics_cost(
                route=store_route,
                n_items=waste_count,
                n_stations=0,
                station_coords=station_coords,
                include_load=True,
                include_unload=True
            )
        else:
            store_waste_cost = 0.0

        return collect_product_cost + disassembly_cost + store_waste_cost

    def _estimate_end_station(self, task, wb_id, customer_id, storage_id):
        if task['order_type'] == Order.OT_PRODUCE:
            if task.get('has_following_recycle', False):
                return wb_id
            return customer_id if customer_id is not None else wb_id

        if task['order_type'] == Order.OT_RECYCLE:
            if task.get('waste_materials') and storage_id is not None:
                return storage_id
            return wb_id

        return wb_id

    # --------------------------------------------------------
    # Step 4: 재료 출처 분류 + 가상 차감
    # --------------------------------------------------------

    def _register_recycle_releases(self, recycle_orders, produced_product_ids):
        recycle_available = {}

        for order in recycle_orders:
            if order.get('source_after_produce', False):
                continue

            for material in order['materials']:
                recycle_available.setdefault(material, []).append(order)

        return recycle_available

    def _assign_material_sources(
        self,
        produce_orders,
        recycle_available,
        virtual_stock
    ):
        for order in produce_orders:
            for material in order['materials']:

                if material in recycle_available and recycle_available[material]:
                    recycle_order = recycle_available[material].pop(0)
                    order['material_sources'].append(
                        (material, 'WB', recycle_order)
                    )

                else:
                    station_id = self._find_in_stock(material, virtual_stock)

                    if station_id is not None:
                        virtual_stock[station_id].remove(material)
                        order['material_sources'].append(
                            (material, station_id, None)
                        )
                    else:
                        raise RuntimeError(
                            f'PRODUCE {order["product_id"]}에 필요한 '
                            f'재료 {material}를 구할 수 없음'
                        )

    def _assign_waste_materials(self, recycle_orders, produce_orders):
        needed = []

        for order in produce_orders:
            needed.extend(order['materials'])

        remaining_needed = list(needed)

        for order in recycle_orders:
            for material in order['materials']:
                if material in remaining_needed:
                    order['reuse_materials'].append(material)
                    remaining_needed.remove(material)
                else:
                    order['waste_materials'].append(material)

    def _find_in_stock(self, material, stock):
        for station_id, materials in stock.items():
            if material in materials:
                return station_id

        return None

    # --------------------------------------------------------
    # Step 5: 시간 비용 기반 WB 시퀀스 결정
    # --------------------------------------------------------

    def _build_wb_sequence(
        self,
        produce_orders,
        recycle_orders,
        station_materials,
        station_coords,
        wb_id,
        customer_id,
        storage_id
    ):
        recycle_after_produce_ids = {
            ro['product_id']
            for ro in recycle_orders
            if ro.get('source_after_produce', False)
        }

        for po in produce_orders:
            po['has_following_recycle'] = po['product_id'] in recycle_after_produce_ids

        all_tasks = []
        all_tasks.extend(produce_orders)
        all_tasks.extend(recycle_orders)

        remaining = list(all_tasks)
        wb_sequence = []

        done_recycle_ids = set()
        done_product_ids = set()

        current_station = STATION_START_GOAL

        while remaining:
            available = []

            for task in remaining:
                if task['order_type'] == Order.OT_PRODUCE:
                    deps = self._get_recycle_deps(task)

                    if all(id(dep) in done_recycle_ids for dep in deps):
                        available.append(task)

                elif task['order_type'] == Order.OT_RECYCLE:
                    if task.get('source_after_produce', False):
                        if task['product_id'] in done_product_ids:
                            available.append(task)
                    else:
                        available.append(task)

            if not available:
                self.get_logger().warn(
                    '사용 가능한 task가 없습니다. '
                    '의존성 순환 가능성이 있어 남은 작업 중 첫 작업을 강제 선택합니다.'
                )
                available = [remaining[0]]

            best_task = min(
                available,
                key=lambda task: self._estimate_task_cost(
                    task,
                    current_station,
                    station_materials,
                    station_coords,
                    wb_id,
                    customer_id,
                    storage_id
                )
            )

            best_cost = self._estimate_task_cost(
                best_task,
                current_station,
                station_materials,
                station_coords,
                wb_id,
                customer_id,
                storage_id
            )

            self.get_logger().info(
                f'비용 기반 선택: {self._task_label(best_task)} | '
                f'estimated_cost={best_cost:.2f}s'
            )

            wb_sequence.append(best_task)
            remaining.remove(best_task)

            if best_task['order_type'] == Order.OT_PRODUCE:
                done_product_ids.add(best_task['product_id'])

            elif best_task['order_type'] == Order.OT_RECYCLE:
                done_recycle_ids.add(id(best_task))

            current_station = self._estimate_end_station(
                best_task,
                wb_id,
                customer_id,
                storage_id
            )

        return wb_sequence

    def _get_recycle_deps(self, produce_order):
        deps = []

        for _, _, dep_recycle in produce_order['material_sources']:
            if dep_recycle is not None and dep_recycle not in deps:
                deps.append(dep_recycle)

        return deps

    def _task_label(self, task):
        if task['order_type'] == Order.OT_PRODUCE:
            return f'PRODUCE {task["product_id"]}'

        if task['order_type'] == Order.OT_RECYCLE:
            return f'RECYCLE {task["product_id"]}'

        return f'UNKNOWN {task["product_id"]}'

    # --------------------------------------------------------
    # Step 6: 스텝 시퀀스 생성
    # --------------------------------------------------------

    def _generate_steps(
        self,
        wb_sequence,
        station_materials,
        station_coords,
        wb_id,
        customer_id,
        storage_id
    ):
        steps = []
        step_id = 0
        last_wb_step_id = None

        # slot_1: 완성품/재활용 대상 제품 슬롯, 최대 1개
        # slot_material: 원재료 슬롯, 최대 5개
        slot_1 = None
        slot_material = []

        pending_loads = []
        loaded_sources = set()

        current_amr_station = STATION_START_GOAL

        for wb_task in wb_sequence:

            # ------------------------------------------------
            # RECYCLE: 분해 대상 완성품 Load
            # ------------------------------------------------
            if wb_task['order_type'] == Order.OT_RECYCLE:

                if slot_1 is not None or slot_material:
                    step_id, last_wb_step_id = self._flush_unload(
                        steps,
                        step_id,
                        pending_loads,
                        slot_1,
                        slot_material,
                        wb_id,
                        last_wb_step_id
                    )

                    current_amr_station = wb_id
                    slot_1 = None
                    slot_material = []
                    pending_loads = []

                if not wb_task.get('source_after_produce', False):
                    src = self._find_in_stock(
                        wb_task['product_id'],
                        station_materials
                    )

                    if src is None:
                        raise RuntimeError(
                            f'RECYCLE 대상 product {wb_task["product_id"]}를 '
                            '재고에서 찾을 수 없음'
                        )

                    if slot_1 is not None:
                        raise RuntimeError(
                            f'AMR 완성품 적재 용량 초과: '
                            f'RECYCLE {wb_task["product_id"]}'
                        )

                    steps.append(self._make_step(
                        step_id,
                        Step.AMR,
                        Step.LOAD,
                        [wb_task['product_id']],
                        src,
                        []
                    ))

                    pending_loads.append(step_id)
                    step_id += 1
                    current_amr_station = src

                    slot_1 = wb_task['product_id']

                preload_by_station = self._select_preload_materials(
                    wb_sequence=wb_sequence,
                    loaded_sources=loaded_sources,
                    slot_material=slot_material,
                    preferred_sources=None
                )

                sorted_sources = self._sort_load_sources(
                    preload_by_station,
                    current_amr_station,
                    station_coords
                )

                for source in sorted_sources:
                    materials = preload_by_station[source]

                    if not materials:
                        continue

                    steps.append(self._make_step(
                        step_id,
                        Step.AMR,
                        Step.LOAD,
                        materials,
                        source,
                        []
                    ))

                    pending_loads.append(step_id)
                    step_id += 1
                    current_amr_station = source

            # ------------------------------------------------
            # PRODUCE: 원재료 Load
            # ------------------------------------------------
            elif wb_task['order_type'] == Order.OT_PRODUCE:

                needs_wb_material = any(
                    dep is not None
                    for _, _, dep in wb_task['material_sources']
                )

                if needs_wb_material and pending_loads:
                    step_id, last_wb_step_id = self._flush_unload(
                        steps,
                        step_id,
                        pending_loads,
                        slot_1,
                        slot_material,
                        wb_id,
                        last_wb_step_id
                    )

                    current_amr_station = wb_id
                    slot_1 = None
                    slot_material = []
                    pending_loads = []

                load_by_station = {}

                # 1) 현재 PRODUCE에 필요한 원재료 먼저 적재 대상으로 등록
                for index, (material, source, dep) in enumerate(
                    wb_task['material_sources']
                ):
                    source_key = (id(wb_task), index)

                    if dep is None and source_key not in loaded_sources:
                        if len(slot_material) >= MAX_RAW_CAPACITY:
                            raise RuntimeError(
                                f'AMR 원재료 적재 용량 초과: '
                                f'PRODUCE {wb_task["product_id"]}'
                            )

                        load_by_station.setdefault(source, []).append(material)
                        slot_material.append(material)
                        loaded_sources.add(source_key)

                # 2) 현재 주문 때문에 이미 방문할 station을 우선해서
                #    미래 PRODUCE 재료를 미리 적재
                preferred_sources = set(load_by_station.keys())

                preload_by_station = self._select_preload_materials(
                    wb_sequence=wb_sequence,
                    loaded_sources=loaded_sources,
                    slot_material=slot_material,
                    preferred_sources=preferred_sources
                )

                # 3) 같은 station의 현재 재료와 미래 재료를 하나의 LOAD로 병합
                load_by_station = self._merge_load_maps(
                    load_by_station,
                    preload_by_station
                )

                sorted_sources = self._sort_load_sources(
                    load_by_station,
                    current_amr_station,
                    station_coords
                )

                for source in sorted_sources:
                    materials = load_by_station[source]

                    if not materials:
                        continue

                    steps.append(self._make_step(
                        step_id,
                        Step.AMR,
                        Step.LOAD,
                        materials,
                        source,
                        []
                    ))

                    pending_loads.append(step_id)
                    step_id += 1
                    current_amr_station = source

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
                    step_id,
                    Step.AMR,
                    Step.UNLOAD,
                    all_objects,
                    wb_id,
                    unload_depends
                ))

                step_id += 1
                current_amr_station = wb_id

            else:
                unload_step_id = None

            # ------------------------------------------------
            # WB 작업 PRODUCE or RECYCLE
            # ------------------------------------------------
            wb_action = Step.PRODUCE \
                if wb_task['order_type'] == Order.OT_PRODUCE else Step.RECYCLE

            wb_objects = [wb_task['product_id']]

            wb_depends = []

            if unload_step_id is not None:
                wb_depends.append(unload_step_id)

            if last_wb_step_id is not None:
                wb_depends.append(last_wb_step_id)

            if wb_task['order_type'] == Order.OT_PRODUCE:
                for _, _, dep_recycle in wb_task['material_sources']:
                    if dep_recycle is not None:
                        recycle_sid = self._find_wb_recycle_step_id(
                            steps,
                            dep_recycle['product_id']
                        )

                        if recycle_sid is not None and recycle_sid not in wb_depends:
                            wb_depends.append(recycle_sid)

            steps.append(self._make_step(
                step_id,
                Step.WB,
                wb_action,
                wb_objects,
                wb_id,
                wb_depends
            ))

            last_wb_step_id = step_id
            step_id += 1

            # ------------------------------------------------
            # PRODUCE 완료 후: 완성품 납품
            # ------------------------------------------------
            if wb_task['order_type'] == Order.OT_PRODUCE:
                if wb_task.get('has_following_recycle', False):
                    self.get_logger().info(
                        f'PRODUCE {wb_task["product_id"]} 결과물은 '
                        'lifecycle RECYCLE을 위해 WB에 유지'
                    )
                else:
                    load_sid = step_id

                    steps.append(self._make_step(
                        step_id,
                        Step.AMR,
                        Step.LOAD,
                        [wb_task['product_id']],
                        wb_id,
                        [last_wb_step_id]
                    ))

                    step_id += 1
                    current_amr_station = wb_id

                    steps.append(self._make_step(
                        step_id,
                        Step.AMR,
                        Step.UNLOAD,
                        [wb_task['product_id']],
                        customer_id,
                        [load_sid]
                    ))

                    step_id += 1
                    current_amr_station = customer_id

            # ------------------------------------------------
            # RECYCLE 완료 후: 불필요 원재료 Storage 반납
            # ------------------------------------------------
            if (
                wb_task['order_type'] == Order.OT_RECYCLE
                and wb_task['waste_materials']
            ):
                if len(wb_task['waste_materials']) > MAX_RAW_CAPACITY:
                    raise RuntimeError(
                        f'AMR 원재료 적재 용량 초과: '
                        f'RECYCLE waste={wb_task["waste_materials"]}'
                    )

                load_sid = step_id

                steps.append(self._make_step(
                    step_id,
                    Step.AMR,
                    Step.LOAD,
                    wb_task['waste_materials'],
                    wb_id,
                    [last_wb_step_id]
                ))

                step_id += 1
                current_amr_station = wb_id

                steps.append(self._make_step(
                    step_id,
                    Step.AMR,
                    Step.UNLOAD,
                    wb_task['waste_materials'],
                    storage_id,
                    [load_sid]
                ))

                step_id += 1
                current_amr_station = storage_id

            slot_1 = None
            slot_material = []
            pending_loads = []

        # ------------------------------------------------
        # 모든 작업 완료 후 START/GOAL 복귀
        # ------------------------------------------------
        if step_id > 0:
            last_step_id = step_id - 1

            steps.append(self._make_step(
                step_id,
                Step.AMR,
                Step.GOAL,
                [],
                STATION_START_GOAL,
                [last_step_id]
            ))

            step_id += 1

        return steps

    def _select_preload_materials(
        self,
        wb_sequence,
        loaded_sources,
        slot_material,
        preferred_sources=None
    ):
        """
        남는 원재료 적재 공간이 있으면 이후 PRODUCE에 필요한 원재료를 미리 적재한다.

        핵심:
        - 이미 현재 주문 때문에 방문할 station의 미래 재료를 우선 적재한다.
        - 예: 현재 711 때문에 station 1을 방문한다면,
          미래 462의 재료 6이 station 1에 있을 때 같이 싣는다.

        적재 제약:
        - 원재료 슬롯: 최대 5개
        - 완성품 슬롯은 별도이므로 여기서 계산하지 않는다.
        """

        preferred_sources = set(preferred_sources or [])
        preload_by_station = {}

        occupied_raw = len(slot_material)
        available_raw = MAX_RAW_CAPACITY - occupied_raw

        if available_raw <= 0:
            return preload_by_station

        candidates = []

        for future_task in wb_sequence:
            if future_task['order_type'] != Order.OT_PRODUCE:
                continue

            for index, (material, source, dep) in enumerate(
                future_task['material_sources']
            ):
                source_key = (id(future_task), index)

                if (
                    dep is None
                    and isinstance(source, int)
                    and source_key not in loaded_sources
                ):
                    preferred_score = 0 if source in preferred_sources else 1

                    candidates.append({
                        'preferred_score': preferred_score,
                        'source': source,
                        'material': material,
                        'source_key': source_key,
                    })

        candidates.sort(
            key=lambda c: (
                c['preferred_score'],
                c['source']
            )
        )

        for c in candidates:
            if available_raw <= 0:
                break

            source = c['source']
            material = c['material']
            source_key = c['source_key']

            preload_by_station.setdefault(source, []).append(material)
            slot_material.append(material)
            loaded_sources.add(source_key)
            available_raw -= 1

        return preload_by_station

    def _merge_load_maps(self, base_map, extra_map):
        """
        같은 station에서 가져올 현재 주문 재료와 미래 주문 재료를 병합한다.
        """
        merged = {}

        for source, materials in base_map.items():
            merged.setdefault(source, []).extend(materials)

        for source, materials in extra_map.items():
            merged.setdefault(source, []).extend(materials)

        return merged

    def _sort_load_sources(self, load_by_station, current_station, station_coords):
        sources = list(load_by_station.keys())

        return sorted(
            sources,
            key=lambda sid: (
                self._move_time_between(current_station, sid, station_coords),
                -len(load_by_station[sid])
            )
        )

    # --------------------------------------------------------
    # 헬퍼 함수
    # --------------------------------------------------------

    def _make_step(self, step_id, type_, action, object_ids, station_id, depends_on):
        step = Step()

        step.step_id = step_id
        step.type = type_
        step.action = action
        step.object_ids = list(object_ids)
        step.station_id = station_id if station_id is not None else -1
        step.depends_on = list(depends_on)

        return step

    def _flush_unload(
        self,
        steps,
        step_id,
        pending_loads,
        slot_1,
        slot_material,
        wb_id,
        last_wb_step_id
    ):
        unload_depends = list(pending_loads)

        if last_wb_step_id is not None:
            unload_depends.append(last_wb_step_id)

        all_objects = (
            ([slot_1] if slot_1 is not None else []) + slot_material
        )

        if all_objects:
            steps.append(self._make_step(
                step_id,
                Step.AMR,
                Step.UNLOAD,
                all_objects,
                wb_id,
                unload_depends
            ))

            step_id += 1

        return step_id, last_wb_step_id

    def _find_wb_recycle_step_id(self, steps, product_id):
        for step in steps:
            if step.type == Step.WB and step.action == Step.RECYCLE:
                if product_id in step.object_ids:
                    return step.step_id

        return None

    # --------------------------------------------------------
    # 로그
    # --------------------------------------------------------

    def _log_cost_summary(
        self,
        wb_sequence,
        station_materials,
        station_coords,
        wb_id,
        customer_id,
        storage_id
    ):
        self.get_logger().info('===== 시간 비용 기반 WB 작업 순서 =====')

        current_station = STATION_START_GOAL

        for index, task in enumerate(wb_sequence):
            cost = self._estimate_task_cost(
                task,
                current_station,
                station_materials,
                station_coords,
                wb_id,
                customer_id,
                storage_id
            )

            self.get_logger().info(
                f'{index + 1}. {self._task_label(task)} | '
                f'estimated_cost={cost:.2f}s'
            )

            current_station = self._estimate_end_station(
                task,
                wb_id,
                customer_id,
                storage_id
            )

        self.get_logger().info('===================================')

    def _log_plan_summary(self, produce_orders, recycle_orders):
        def name(pid):
            return PRODUCT_NAMES.get(pid, str(pid))

        self.get_logger().info('===== 실행 계획 요약 =====')

        self.get_logger().info(
            f'AMR 적재 제약: 원재료 최대 {MAX_RAW_CAPACITY}개, '
            f'완성품 최대 {MAX_PRODUCT_CAPACITY}개'
        )

        for order in recycle_orders:
            pid = order['product_id']

            self.get_logger().info(f'[RECYCLE] {pid} ({name(pid)})')
            self.get_logger().info(f'  -> 분해 후: {order["materials"]}')

            if order.get('source_after_produce', False):
                self.get_logger().info(
                    '  -> source : PRODUCE 결과물을 WB에서 바로 RECYCLE'
                )

            if order['reuse_materials']:
                reuse_info = []
                reuse_remaining = list(order['reuse_materials'])

                for po in produce_orders:
                    for material, source, dep_recycle in po['material_sources']:
                        if dep_recycle is order and material in reuse_remaining:
                            reuse_remaining.remove(material)
                            reuse_info.append(
                                f'{material} -> PRODUCE {po["product_id"]}'
                            )

                self.get_logger().info(
                    f'  -> reuse : {order["reuse_materials"]}  '
                    f'({" / ".join(reuse_info)})'
                )

            if order['waste_materials']:
                self.get_logger().info(
                    f'  -> waste : {order["waste_materials"]}  -> Storage 반납'
                )

        for order in produce_orders:
            pid = order['product_id']

            self.get_logger().info(f'[PRODUCE] {pid} ({name(pid)})')
            self.get_logger().info(f'  -> 재료: {order["materials"]}')

            if order.get('has_following_recycle', False):
                self.get_logger().info(
                    '  -> lifecycle : 생산 후 고객 납품 없이 WB에서 RECYCLE 예정'
                )

            for material, source, dep_recycle in order['material_sources']:
                if dep_recycle is not None:
                    self.get_logger().info(
                        f'  -> {material} : RECYCLE '
                        f'{dep_recycle["product_id"]} 후 WB에서 재사용'
                    )
                else:
                    self.get_logger().info(
                        f'  -> {material} : station={source} 에서 Load'
                    )

        self.get_logger().info('==========================')

    def _log_steps(self, steps):
        type_map = {
            Step.AMR: 'AMR',
            Step.WB: 'WB ',
        }

        action_map = {
            Step.LOAD: 'LOAD   ',
            Step.UNLOAD: 'UNLOAD ',
            Step.PRODUCE: 'PRODUCE',
            Step.RECYCLE: 'RECYCLE',
            Step.GOAL: 'GOAL   ',
        }

        self.get_logger().info('===== 스텝 시퀀스 =====')

        for s in steps:
            self.get_logger().info(
                f'[{s.step_id:2d}] {type_map.get(s.type, str(s.type))} | '
                f'{action_map.get(s.action, str(s.action))} | '
                f'objects={list(s.object_ids)} | '
                f'station={s.station_id} | '
                f'depends_on={list(s.depends_on)}'
            )

        self.get_logger().info('======================')


def main(args=None):
    rclpy.init(args=args)

    node = PlanningNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()