"""
planning_node.py
Task.msg를 수신해서 depends_on 기반 스텝 시퀀스를 생성하고
ManagerNode 요청 시 전달하는 노드.

반영된 규칙
1. batch ID 해석
   - 10,20,...,80은 각각 raw 1,2,...,8을 BATCH_SIZE개 담은 batch로 취급한다.
   - planner 내부 재고 계산에는 batch capacity를 사용한다.
   - AMR LOAD/UNLOAD object_ids에는 batch ID가 아니라 raw ID로 분해해서 넣는다.

2. RECYCLE source 규칙
   - Order.OT_RECYCLE product_id는 무조건 CUSTOMER station에 있다고 가정한다.
   - customer.material_ids에 product_id가 없어도 CUSTOMER에서 LOAD하도록 계획한다.

3. Lifecycle 역추론 규칙
   P = PRODUCE 필요 원재료
   R = RECYCLE 분해 결과 원재료
   C = P와 R의 공통 원재료

   arena_layout의 storage/hybrid material_ids는
   (P - C) + (R - C)로 생성되었다고 보고 역추론한다.

   P - C : PRODUCE에 바로 사용할 초기 재료
   C     : RECYCLE 후 WB에서 PRODUCE에 바로 재사용할 재료
   R - C : RECYCLE 후 station에 반납해야 할 leftover target
"""

import copy
import json
import math
import os
from collections import Counter

import rclpy
from rclpy.node import Node

from sml_msgs.msg import Task, Order, Station, Step
from sml_msgs.srv import GetPlan


PRODUCT_NAMES = {
    34: 'Battery', 13: 'Magnet', 81: 'E-Stop',
    442: 'Carrot', 241: 'Traffic Light', 462: 'Small Tree',
    711: 'Hammer', 4482: 'Big Carrot', 8518: 'Burger',
    48132: 'Ice Cream', 46262: 'Big Tree',
}

RAW_TO_BATCH = {
    1: 10,
    2: 20,
    3: 30,
    4: 40,
    5: 50,
    6: 60,
    7: 70,
    8: 80,
}
BATCH_TO_RAW = {batch: raw for raw, batch in RAW_TO_BATCH.items()}
BATCH_SIZE = 5

# --------------------------------------------------------
# 경기 / 시스템 시간 가정값
# --------------------------------------------------------

AMR_SPEED = 13.0              # [m/s]
VISION_TIME_AVG = 2.0         # [s] 비전 인식 평균값
LOAD_TIME = 10.0              # [s/item]
UNLOAD_TIME = 10.0            # [s/item]
ASSEMBLY_PAIR_TIME = 12.5     # [s/connection]
DISASSEMBLY_PAIR_TIME = 12.5  # [s/disconnection]

# --------------------------------------------------------
# AMR 적재 제약
# --------------------------------------------------------

MAX_RAW_CAPACITY = 5
MAX_PRODUCT_CAPACITY = 1

# 대회 arena_layout에는 포함되지 않는 AMR의 시작/복귀 지점.
STATION_START_GOAL = 0

# 실제 시스템에서 작업로봇이 연결된 WORKBENCH station.
# arena_layout에 여러 WORKBENCH가 들어와도 플래너는 이 station만 사용한다.
FIXED_WORKBENCH_STATION_ID = 7

STATION_COORD_JSON_PARAM = 'station_coord_json_path'
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

        self.declare_parameter('task_topic', '/sml/task')
        self.declare_parameter('use_time_cost', True)
        self.declare_parameter('amr_speed_mps', AMR_SPEED)
        self.declare_parameter(
            STATION_COORD_JSON_PARAM,
            DEFAULT_STATION_COORD_JSON_PATH
        )

        task_topic = self.get_parameter('task_topic').value
        self.use_time_cost = bool(self.get_parameter('use_time_cost').value)
        self.amr_speed_mps = float(self.get_parameter('amr_speed_mps').value)
        self.station_coords = self._load_station_coord_json()

        self.task_sub = self.create_subscription(
            Task, task_topic, self.task_callback, 10
        )
        self.plan_srv = self.create_service(
            GetPlan, '/sml/get_plan', self.get_plan_callback
        )

        self.get_logger().info(f'PlanningNode 시작 | task_topic={task_topic} | use_time_cost={self.use_time_cost} | coords={len(self.station_coords)}')

    # --------------------------------------------------------
    # 콜백
    # --------------------------------------------------------

    def task_callback(self, task):
        if self.plan_generated:
            return
        self.plan_generated = True
        self.get_logger().info('Task 수신 → 계획 생성 시작')
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
            produce_orders, recycle_orders = \
                self._parse_orders(task.order_list)

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

            self.steps = self._generate_steps(
                wb_sequence, station_items,
                wb_id, customer_id, storage_id
            )

            self.get_logger().info(f'계획 생성 완료: {len(self.steps)}개 스텝')
            self._log_cost_summary(wb_sequence, wb_id, customer_id)
            self._log_material_model(material_model)
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
                'station_coord_json_path가 비어 있습니다. 이동 시간은 fallback 좌표로 계산됩니다.'
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
                f'station 좌표 JSON 로드 완료: {len(coords)}개 station, path={path}'
            )
            return coords

        except Exception as e:
            self.get_logger().error(f'station 좌표 JSON 로드 실패: {e}')
            return {}

    # --------------------------------------------------------
    # Step 2: 입력 파싱
    # --------------------------------------------------------

    def _parse_orders(self, order_list):
        produce_orders = []
        recycle_orders = []

        for order in order_list:
            parsed = {
                'order_type':       order.order_type,
                'product_id':       order.product_id,
                'materials':        self._parse_product_id(order.product_id),
                # material_sources: [(material, source, dep_recycle, object_id, token_ref), ...]
                # source가 station_id이면 AMR이 object_id를 source에서 LOAD한다.
                # source가 'WB'이면 해당 material은 RECYCLE 후 WB에서 바로 재사용한다.
                'material_sources': [],
                'waste_materials':  [],
                # waste_items: [{'raw': m, 'object_id': x, 'station_id': y, 'token_ref': z}, ...]
                'waste_items':      [],
                'reuse_materials':  [],
                'source_after_produce': False,
            }
            if order.order_type == Order.OT_PRODUCE:
                produce_orders.append(parsed)
            elif order.order_type == Order.OT_RECYCLE:
                recycle_orders.append(parsed)
            elif order.order_type == ORDER_TYPE_LIFECYCLE:
                # 단일 lifecycle order가 들어오는 경우만 produce→recycle 내부 연결로 해석한다.
                # 별도 OT_RECYCLE order는 항상 CUSTOMER에 있다고 가정한다.
                produce_part = copy.deepcopy(parsed)
                recycle_part = copy.deepcopy(parsed)
                produce_part['order_type'] = Order.OT_PRODUCE
                recycle_part['order_type'] = Order.OT_RECYCLE
                recycle_part['source_after_produce'] = True
                produce_orders.append(produce_part)
                recycle_orders.append(recycle_part)

        # 재료 개수가 적은 PRODUCE부터 처리하면 재사용/재고 배정이 단순해진다.
        produce_orders.sort(key=lambda o: len(o['materials']))
        return produce_orders, recycle_orders

    def _parse_product_id(self, product_id):
        return [int(d) for d in str(product_id)]

    def _multiset_common_preserve_order(self, left, right):
        right_count = Counter(right)
        common = []
        for item in left:
            if right_count[item] > 0:
                common.append(item)
                right_count[item] -= 1
        return common

    def _subtract_preserve_order(self, base, remove):
        remove_count = Counter(remove)
        result = []
        for item in base:
            if remove_count[item] > 0:
                remove_count[item] -= 1
            else:
                result.append(item)
        return result

    def _build_material_model(self, produce_orders, recycle_orders):
        produce_materials = []
        recycle_materials = []

        for order in produce_orders:
            produce_materials.extend(order['materials'])
        for order in recycle_orders:
            recycle_materials.extend(order['materials'])

        common = self._multiset_common_preserve_order(
            produce_materials, recycle_materials
        )
        produce_initial = self._subtract_preserve_order(produce_materials, common)
        recycle_leftover = self._subtract_preserve_order(recycle_materials, common)

        return {
            'produce_materials': produce_materials,
            'recycle_materials': recycle_materials,
            'common_reuse': common,
            'produce_initial': produce_initial,
            'recycle_leftover': recycle_leftover,
            'produce_initial_counts': Counter(produce_initial),
            'recycle_leftover_counts': Counter(recycle_leftover),
        }

    def _parse_arena(self, arena_layout, material_model):
        station_items = {}
        workbench_ids = []
        customer_id = None
        storage_id = None

        stock_tokens = []
        waste_target_tokens = []
        token_ref = 0

        initial_counts = Counter(material_model['produce_initial_counts'])
        waste_counts = Counter(material_model['recycle_leftover_counts'])

        for station in arena_layout:
            station_items[station.station_id] = list(station.material_ids)

            if station.station_type == Station.ST_WORKBENCH:
                workbench_ids.append(station.station_id)
            elif station.station_type == Station.ST_CUSTOMER:
                customer_id = station.station_id
            elif station.station_type == Station.ST_STORAGE and storage_id is None:
                storage_id = station.station_id

            # CUSTOMER의 material_ids는 recycle product 표시용이므로 raw stock으로 보지 않는다.
            if station.station_type not in (Station.ST_STORAGE, Station.ST_HYBRID):
                continue

            for object_id in station.material_ids:
                raw, raw_count = self._decode_station_object(object_id)
                if raw is None:
                    continue

                token = {
                    'ref': token_ref,
                    'station_id': station.station_id,
                    'object_id': object_id,
                    'raw': raw,
                    'capacity': raw_count,
                    'remaining': raw_count,
                }
                token_ref += 1

                # lifecycle 생성 규칙을 역추론한다.
                # P-C에 해당하는 raw는 초기 생산 재료(stock), R-C는 recycle leftover target.
                if initial_counts[raw] > 0:
                    stock_tokens.append(token)
                    initial_counts[raw] -= raw_count
                elif waste_counts[raw] > 0:
                    waste_target_tokens.append(token)
                    waste_counts[raw] -= raw_count
                else:
                    # 공식 예시나 수동 task에서 모델과 맞지 않는 재료가 들어온 경우,
                    # 생산 재료로 사용할 수 있도록 stock으로 둔다.
                    stock_tokens.append(token)

        if not workbench_ids:
            raise RuntimeError('arena_layout에 WORKBENCH station이 없습니다')

        # 작업로봇은 실제 시스템에서 station 7번에 고정되어 있으므로,
        # 비용 기반으로 WORKBENCH를 선택하지 않는다.
        if FIXED_WORKBENCH_STATION_ID not in workbench_ids:
            raise RuntimeError(
                f'고정 작업로봇 station {FIXED_WORKBENCH_STATION_ID}이 '
                f'arena_layout의 WORKBENCH 목록에 없습니다. '
                f'현재 WORKBENCH 후보={workbench_ids}'
            )
        wb_id = FIXED_WORKBENCH_STATION_ID
        self.get_logger().info(f'[WB] fixed workbench={wb_id} 사용')

        if customer_id is None:
            raise RuntimeError('arena_layout에 CUSTOMER station이 없습니다')
        if storage_id is None:
            raise RuntimeError('arena_layout에 STORAGE station이 없습니다')

        return station_items, stock_tokens, waste_target_tokens, wb_id, customer_id, storage_id

    def _decode_station_object(self, object_id):
        """station material_id를 raw 단위로 해석한다."""
        if object_id in BATCH_TO_RAW:
            return BATCH_TO_RAW[object_id], BATCH_SIZE
        if 1 <= object_id <= 8:
            return object_id, 1
        # product_id 등은 raw stock으로 보지 않음
        return None, 0

    # --------------------------------------------------------
    # Step 2: 재료 출처 분류 + 가상 차감
    # --------------------------------------------------------

    def _register_recycle_releases(self, recycle_orders):
        """RECYCLE 후 나올 재료 등록 → {material: [recycle_order, ...]} (개수 단위)"""
        recycle_available = {}
        for order in recycle_orders:
            if order.get('source_after_produce', False):
                continue
            for material in order['materials']:
                recycle_available.setdefault(material, []).append(order)
        return recycle_available

    def _assign_material_sources(self, produce_orders, recycle_available, stock_tokens, wb_id):
        """
        재료 출처 결정 + 가상 차감
        우선순위: RECYCLE 후 재료 → 초기 재고
        material_sources = [(material, source, dep_recycle, object_id, token_ref), ...]
        """
        for order in produce_orders:
            for material in order['materials']:

                # 1순위: RECYCLE 후 WB에 생기는 재료
                if material in recycle_available and recycle_available[material]:
                    recycle_order = recycle_available[material].pop(0)
                    order['material_sources'].append(
                        (material, 'WB', recycle_order, material, None)
                    )

                # 2순위: 초기 재고에서 가져오기
                else:
                    found = self._find_in_stock(material, stock_tokens, wb_id, order)
                    if found is not None:
                        station_id, object_id, token_ref = found
                        order['material_sources'].append(
                            (material, station_id, None, object_id, token_ref)
                        )
                    else:
                        raise RuntimeError(f'재료 {material}를 구할 수 없음')

    def _find_in_stock(self, material, stock_tokens, wb_id=None, order=None):
        candidates = [
            token for token in stock_tokens
            if token['raw'] == material and token['remaining'] > 0
        ]
        if not candidates:
            return None

        if self.use_time_cost:
            used_station_counts = Counter()
            if order is not None:
                for (_, source, dep, _, _) in order['material_sources']:
                    if dep is None and isinstance(source, int):
                        used_station_counts[source] += 1

            def cost(token):
                station_id = token['station_id']
                group_bonus = 1.5 * used_station_counts[station_id]
                return self._travel_time(station_id, wb_id) - group_bonus

            chosen = min(candidates, key=cost)
        else:
            chosen = candidates[0]

        # AMR/ARM에는 batch ID(10,20,...,80)를 그대로 보내지 않는다.
        # batch token 하나가 여러 raw를 담고 있으면, 필요한 개수만큼 raw ID로 분해해서 보낸다.
        # 예: object_id=40, BATCH_SIZE=5인 token에서 raw 4를 두 번 쓰면
        #     AMR step object_ids에는 [4, 4]가 들어간다.
        use_index = chosen['capacity'] - chosen['remaining']
        chosen['remaining'] -= 1

        amr_object_id = material
        raw_token_ref = (chosen['ref'], use_index)
        return chosen['station_id'], amr_object_id, raw_token_ref

    def _assign_waste_materials(
        self, recycle_orders, produce_orders, waste_target_tokens,
        fallback_storage_id, wb_id
    ):
        """RECYCLE 후 재료 → reuse / waste 분류 및 waste 반납 target 결정."""
        reused_by_recycle = {id(order): [] for order in recycle_orders}

        for po in produce_orders:
            for (material, source, dep_recycle, object_id, token_ref) in po['material_sources']:
                if dep_recycle is not None:
                    reused_by_recycle[id(dep_recycle)].append(material)

        for order in recycle_orders:
            remaining = list(order['materials'])
            for material in reused_by_recycle[id(order)]:
                if material in remaining:
                    remaining.remove(material)
                    order['reuse_materials'].append(material)

            order['waste_materials'] = remaining
            order['waste_items'] = self._assign_waste_targets(
                remaining, waste_target_tokens, fallback_storage_id, wb_id
            )

    def _assign_waste_targets(self, waste_materials, waste_target_tokens, fallback_storage_id, wb_id):
        assignments = []
        fallback_ref_base = 100000

        for i, material in enumerate(waste_materials):
            target = self._find_waste_target(material, waste_target_tokens, wb_id)
            if target is not None:
                station_id, object_id, token_ref = target
                assignments.append({
                    'raw': material,
                    'station_id': station_id,
                    'object_id': object_id,
                    'token_ref': token_ref,
                })
            else:
                # target station을 찾지 못하면 첫 storage에 raw ID 그대로 반납한다.
                assignments.append({
                    'raw': material,
                    'station_id': fallback_storage_id,
                    'object_id': material,
                    'token_ref': fallback_ref_base + i,
                })

        return assignments

    def _find_waste_target(self, material, waste_target_tokens, wb_id=None):
        candidates = [
            token for token in waste_target_tokens
            if token['raw'] == material and token['remaining'] > 0
        ]
        if not candidates:
            return None

        if self.use_time_cost:
            chosen = min(
                candidates,
                key=lambda token: self._travel_time(wb_id, token['station_id'])
            )
        else:
            chosen = candidates[0]

        # Waste target도 batch ID가 아니라 실제 raw ID로 반납하도록 step을 만든다.
        use_index = chosen['capacity'] - chosen['remaining']
        chosen['remaining'] -= 1

        amr_object_id = material
        raw_token_ref = (chosen['ref'], use_index)
        return chosen['station_id'], amr_object_id, raw_token_ref

    # --------------------------------------------------------
    # Step 3: WB 시퀀스 결정
    # --------------------------------------------------------

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
    # --------------------------------------------------------

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
        return 0.0

    def _make_step(self, step_id, type_, action, object_ids, station_id, depends_on):
        step = Step()
        step.step_id    = step_id
        step.type       = type_
        step.action     = action
        step.object_ids = list(object_ids)
        step.station_id = station_id if station_id is not None else -1
        step.depends_on = list(depends_on)
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
    # --------------------------------------------------------

    def _task_label(self, task):
        if task['order_type'] == Order.OT_PRODUCE:
            return f'PRODUCE {task["product_id"]}'
        if task['order_type'] == Order.OT_RECYCLE:
            return f'RECYCLE {task["product_id"]}'
        return f'UNKNOWN {task["product_id"]}'

    def _log_cost_summary(self, wb_sequence, wb_id, customer_id):
        self.get_logger().info('===== 시간 비용 기반 WB 작업 순서 =====')

        for index, task in enumerate(wb_sequence):
            cost = self._estimate_task_cost(task, wb_id, customer_id)
            self.get_logger().info(
                f'{index + 1}. {self._task_label(task)} | '
                f'estimated_cost={cost:.2f}s'
            )

        self.get_logger().info('===================================')

    def _log_material_model(self, model):
        self.get_logger().info('===== lifecycle material model =====')
        self.get_logger().info(f'P produce_materials : {model["produce_materials"]}')
        self.get_logger().info(f'R recycle_materials : {model["recycle_materials"]}')
        self.get_logger().info(f'C common_reuse      : {model["common_reuse"]}')
        self.get_logger().info(f'P-C initial         : {model["produce_initial"]}')
        self.get_logger().info(f'R-C leftover        : {model["recycle_leftover"]}')
        self.get_logger().info('====================================')

    def _log_plan_summary(self, produce_orders, recycle_orders):
        def name(pid):
            return PRODUCT_NAMES.get(pid, str(pid))

        self.get_logger().info('===== 실행 계획 요약 =====')

        for order in recycle_orders:
            pid = order['product_id']
            self.get_logger().info(f'[RECYCLE] {pid} ({name(pid)})')
            self.get_logger().info(f'  -> CUSTOMER에서 완성품 LOAD 가정')
            self.get_logger().info(f'  -> 분해 후: {order["materials"]}')
            if order['reuse_materials']:
                reuse_info = []
                reuse_remaining = list(order['reuse_materials'])
                for po in produce_orders:
                    for (material, source, dep_recycle, object_id, token_ref) in po['material_sources']:
                        if dep_recycle is order and material in reuse_remaining:
                            reuse_remaining.remove(material)
                            reuse_info.append(
                                f'{material} -> PRODUCE {po["product_id"]}'
                            )
                self.get_logger().info(
                    f'  -> reuse : {order["reuse_materials"]}  ({" / ".join(reuse_info)})'
                )
            if order['waste_materials']:
                waste_targets = [
                    f'{item["object_id"]}->station {item["station_id"]}'
                    for item in order['waste_items']
                ]
                self.get_logger().info(
                    f'  -> waste : {order["waste_materials"]}  ({" / ".join(waste_targets)})'
                )

        for order in produce_orders:
            pid = order['product_id']
            self.get_logger().info(f'[PRODUCE] {pid} ({name(pid)})')
            self.get_logger().info(f'  -> 재료: {order["materials"]}')
            for (material, source, dep_recycle, object_id, token_ref) in order['material_sources']:
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
        type_map   = {Step.AMR: 'AMR', Step.WB: 'WB '}
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
                f'[{s.step_id:2d}] {type_map[s.type]} | '
                f'{action_map[s.action]} | '
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