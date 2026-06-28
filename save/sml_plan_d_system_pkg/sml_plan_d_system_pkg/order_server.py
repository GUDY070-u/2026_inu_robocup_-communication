#!/usr/bin/env python3

import random
from collections import Counter

import rclpy
from rclpy.node import Node
from sml_plan_d_msgs.msg import Task, Order, Station


# Station Type
ST_STORAGE   = Station.ST_STORAGE
ST_WORKBENCH = Station.ST_WORKBENCH
ST_CUSTOMER  = Station.ST_CUSTOMER
ST_HYBRID    = Station.ST_HYBRID

# Order Type
OT_PRODUCE = Order.OT_PRODUCE
OT_RECYCLE = Order.OT_RECYCLE


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

MIXED_BATCH = 90


PRODUCT_DB = {
    34:    ("Battery",       [3, 4]),
    13:    ("Magnet",        [1, 3]),
    81:    ("E-Stop",        [8, 1]),
    442:   ("Carrot",        [4, 4, 2]),
    241:   ("Traffic Light", [2, 4, 1]),
    462:   ("Small Tree",    [4, 6, 2]),
    711:   ("Hammer",        [7, 1, 1]),
    4482:  ("Big Carrot",    [4, 4, 8, 2]),
    8518:  ("Burger",        [8, 5, 1, 8]),
    48132: ("Ice Cream",     [4, 8, 1, 3, 2]),
    46262: ("Big Tree",      [4, 6, 2, 6, 2]),
}


TIER_STAGE_CONFIG = {
    ('entry',    'production'): {'time':  5, 'orders':  1, 'returns':  0, 'raw_mat': ( 2,  1), 'products': ( 1,  0), 'fleet': ( 1,  3)},
    ('entry',    'lifecycle'):  {'time': 10, 'orders':  2, 'returns':  1, 'raw_mat': ( 7,  1), 'products': ( 3,  1), 'fleet': ( 1,  3)},
    ('beginner', 'production'): {'time':  5, 'orders':  2, 'returns':  0, 'raw_mat': ( 5,  1), 'products': ( 2,  0), 'fleet': ( 1,  3)},
    ('beginner', 'recycling'):  {'time':  5, 'orders':  0, 'returns':  2, 'raw_mat': ( 5,  1), 'products': ( 2,  0), 'fleet': ( 1,  3)},
    ('beginner', 'lifecycle'):  {'time': 10, 'orders':  3, 'returns':  2, 'raw_mat': (10,  3), 'products': ( 5,  1), 'fleet': ( 1,  3)},
    ('advanced', 'production'): {'time': 10, 'orders':  5, 'returns':  0, 'raw_mat': (10,  3), 'products': ( 5,  1), 'fleet': ( 1,  6)},
    ('advanced', 'recycling'):  {'time': 10, 'orders':  0, 'returns':  5, 'raw_mat': (10,  3), 'products': ( 5,  1), 'fleet': ( 1,  6)},
    ('advanced', 'lifecycle'):  {'time': 15, 'orders':  5, 'returns':  5, 'raw_mat': (20,  8), 'products': (10,  2), 'fleet': ( 1,  6)},
    ('expert',   'production'): {'time': 20, 'orders': 20, 'returns':  0, 'raw_mat': (40, 15), 'products': (20,  5), 'fleet': ( 3, 12)},
    ('expert',   'recycling'):  {'time': 20, 'orders':  0, 'returns': 20, 'raw_mat': (40, 15), 'products': (20,  5), 'fleet': ( 3, 12)},
    ('expert',   'lifecycle'):  {'time': 30, 'orders': 30, 'returns': 20, 'raw_mat': (100,30), 'products': (50, 10), 'fleet': ( 3, 12)},
}

TIER_NAMES  = {1: 'entry', 2: 'beginner', 3: 'advanced', 4: 'expert'}
STAGE_NAMES = {1: 'production', 2: 'recycling', 3: 'lifecycle'}

SIDE_STATION_OFFSET = {'a': 0, 'b': 8}


# ─────────────────────────────────────────────────────────────
# 고정 아레나 배치 (German Open 2026 규정 기준)
#   station_id = 0 (START/GOAL)은 arena_layout에 포함하지 않음
#   → AMR/Manager 쪽에서 별도 좌표로 관리
# ─────────────────────────────────────────────────────────────
STATION_LAYOUT = {
    1: ST_STORAGE,
    2: ST_STORAGE,
    3: ST_WORKBENCH,
    4: ST_STORAGE,
    5: ST_STORAGE,
    6: ST_WORKBENCH,   # 로봇팔 2대, 메인 작업공간 (코드상 구분 없음, 참고용)
    7: ST_WORKBENCH,
    8: ST_CUSTOMER,
}

STATION_COUNT = len(STATION_LAYOUT)  # 8, 항상 고정

STORAGE_STATION_IDS = [sid for sid, t in STATION_LAYOUT.items() if t == ST_STORAGE]
WORKBENCH_STATION_IDS = [sid for sid, t in STATION_LAYOUT.items() if t == ST_WORKBENCH]
CUSTOMER_STATION_IDS = [sid for sid, t in STATION_LAYOUT.items() if t == ST_CUSTOMER]


class OrderServer(Node):
    def __init__(self):
        super().__init__('order_server')

        self.task_pub  = self.create_publisher(Task, '/sml/task', 10)
        self.published = False

        # ── 입력: 시작 위치 / 경기장 side ─────────────────────
        self.side = self.get_input_side('Start 위치 선택 (1: A, 2: B): ')
        self.station_offset = SIDE_STATION_OFFSET[self.side]
        print(
            f'✓ side={self.side.upper()} 선택: '
            f'station_id offset={self.station_offset}'
        )

        # ── 입력: Tier ─────────────────────────────────────────
        tier_num = self.get_input_int(
            'Tier 선택 (1: Entry, 2: Beginner, 3: Advanced, 4: Expert): ',
            valid_values=[1, 2, 3, 4],
        )
        self.tier = TIER_NAMES[tier_num]

        # ── 입력: Stage (Entry는 Recycling 없음) ───────────────
        if self.tier == 'entry':
            stage_num = self.get_input_int(
                'Stage 선택 (1: Production, 3: Lifecycle): ',
                valid_values=[1, 3],
            )
        else:
            stage_num = self.get_input_int(
                'Stage 선택 (1: Production, 2: Recycling, 3: Lifecycle): ',
                valid_values=[1, 2, 3],
            )
        self.stage  = STAGE_NAMES[stage_num]
        self.config = TIER_STAGE_CONFIG[(self.tier, self.stage)]

        # ── 자동 설정 ──────────────────────────────────────────
        self.produce_count = self.config['orders']
        self.recycle_count = self.config['returns']
        self.station_count = STATION_COUNT

        # 항상 랜덤 선택
        self.random_order = True

        # 오더 수가 제품 종류(11개)를 초과하면 자동으로 중복 허용
        total_count = self.produce_count + self.recycle_count
        self.allow_duplicate = total_count > len(PRODUCT_DB)

        # 출력/검증용 메타 정보
        self.lifecycle_common_materials = []
        self.produce_initial_materials = []
        self.recycle_leftover_materials = []

        # ── Task 생성 + 원자재 수 검증 ─────────────────────────
        self.task, self.arena_layout = self._generate_with_validation()
        self.print_official_style(self.task, self.arena_layout)

        # 엔터를 치면 그 시점에 task를 publish (확인 후 진행)
        input('엔터를 누르면 task를 publish합니다 (플래너로 전달): ')
        self.publish_task()

    # ──────────────────────────────────────────────────────────
    # 원자재 수 검증 (batch 변환 전 개수 기준)
    # ──────────────────────────────────────────────────────────

    def _validate_raw_materials(self, task):
        raw_target, raw_variance = self.config['raw_mat']

        total_raw = 0
        for order in task.order_list:
            _, material_ids = PRODUCT_DB[order.product_id]
            # raw_mat는 생산 필요량 + 재활용 분해 발생량 기준으로 계산한다.
            total_raw += len(material_ids)

        low  = raw_target - raw_variance
        high = raw_target + raw_variance
        return (low <= total_raw <= high), total_raw

    # ──────────────────────────────────────────────────────────
    # 검증 포함 Task 생성 (최대 10회 재시도)
    # ──────────────────────────────────────────────────────────

    def _generate_with_validation(self):
        MAX_RETRY = 10
        task = arena_layout = None

        for attempt in range(1, MAX_RETRY + 1):
            task, arena_layout = self.generate_task()
            valid, total_raw   = self._validate_raw_materials(task)

            if valid:
                if attempt > 1:
                    print(f'✓ {attempt}회 시도 만에 검증 통과 (원자재: {total_raw}개)')
                return task, arena_layout

            raw_target, raw_variance = self.config['raw_mat']
            print(
                f'  시도 {attempt}/{MAX_RETRY}: '
                f'원자재 {total_raw}개 → '
                f'목표 범위 [{raw_target - raw_variance}, {raw_target + raw_variance}] 벗어남, 재생성 중...'
            )

        print(f'⚠ 경고: {MAX_RETRY}회 재시도 후에도 원자재 범위를 만족하지 못했습니다. 마지막 결과를 사용합니다.')
        return task, arena_layout

    # ──────────────────────────────────────────────────────────
    # 입력 헬퍼
    # ──────────────────────────────────────────────────────────

    def get_input_int(self, msg, valid_values=None, min_value=None):
        while True:
            try:
                value = int(input(msg))
                if valid_values is not None and value not in valid_values:
                    print(f'입력 가능 값: {valid_values}')
                    continue
                if min_value is not None and value < min_value:
                    print(f'{min_value} 이상의 값을 입력하세요.')
                    continue
                return value
            except ValueError:
                print('정수를 입력하세요.')

    def get_input_side(self, msg):
        while True:
            raw = input(msg).strip().lower()
            if raw in ('1', 'a'):
                return 'a'
            if raw in ('2', 'b'):
                return 'b'
            print('입력 가능 값: 1/A 또는 2/B')

    def _to_side_station_id(self, base_station_id):
        return int(base_station_id) + int(self.station_offset)

    # ──────────────────────────────────────────────────────────
    # 제품 선택 (produce / recycle 분리)
    # ──────────────────────────────────────────────────────────

    def select_products(self):
        product_ids = list(PRODUCT_DB.keys())
        total_count = self.produce_count + self.recycle_count

        if self.allow_duplicate:
            selected = [random.choice(product_ids) for _ in range(total_count)]
        else:
            selected = random.sample(product_ids, total_count)

        produce_ids = selected[:self.produce_count]
        recycle_ids = selected[self.produce_count:]
        return produce_ids, recycle_ids

    # ──────────────────────────────────────────────────────────
    # 재료 계산 / batch 처리
    # ──────────────────────────────────────────────────────────

    def _materials_from_products(self, product_ids):
        materials = []
        for pid in product_ids:
            _, material_ids = PRODUCT_DB[pid]
            materials.extend(material_ids)
        return materials

    def _multiset_common_preserve_order(self, left, right):
        """중복 개수를 보존한 교집합. 순서는 left 기준."""
        right_count = Counter(right)
        common = []
        for item in left:
            if right_count[item] > 0:
                common.append(item)
                right_count[item] -= 1
        return common

    def _subtract_preserve_order(self, base, remove):
        """base에서 remove를 중복 개수만큼 제거. 순서는 base 기준."""
        remove_count = Counter(remove)
        result = []
        for item in base:
            if remove_count[item] > 0:
                remove_count[item] -= 1
            else:
                result.append(item)
        return result

    def _build_storage_material_list(self, produce_ids, recycle_ids):
        """
        우리가 정한 lifecycle 생성 규칙.

        P = PRODUCE 필요 원재료
        R = RECYCLE 분해 결과 원재료
        C = P와 R의 공통 원재료(중복 개수 보존)

        station 배치 목록 = (P - C) + (R - C)
        CUSTOMER 배치 목록 = RECYCLE product_id

        production-only: C 없음 → P 배치
        recycling-only : C 없음 → R 배치(재활용 후 배치 대상)
        lifecycle      : 공통 C는 재활용 후 생산에 바로 재사용할 예약 재료라서 station에 배치하지 않음
        """
        produce_materials = self._materials_from_products(produce_ids)
        recycle_materials = self._materials_from_products(recycle_ids)

        common = self._multiset_common_preserve_order(
            produce_materials, recycle_materials
        )
        produce_initial = self._subtract_preserve_order(produce_materials, common)
        recycle_leftover = self._subtract_preserve_order(recycle_materials, common)

        self.lifecycle_common_materials = common
        self.produce_initial_materials = produce_initial
        self.recycle_leftover_materials = recycle_leftover

        return produce_initial + recycle_leftover

    def split_materials(self, materials, storage_count):
        buckets = [[] for _ in range(storage_count)]
        if storage_count <= 0:
            return buckets
        for i, material in enumerate(materials):
            buckets[i % storage_count].append(material)
        return buckets

    def _apply_station_batch_ids(self, materials):
        """
        station 내부에서 같은 raw material이 2개 이상이면 batch ID로 압축한다.

        예: [8, 1, 8] -> [80, 1]
            [4, 4, 4] -> [40, 4]
            [4, 4, 4, 4] -> [40, 40]

        product 분해 단계에서 batch화하지 않고, station 배치가 끝난 뒤에만 적용한다.
        """
        counts = Counter(materials)
        used = set()
        result = []

        for material in materials:
            if material in used:
                continue
            used.add(material)

            count = counts[material]
            if material in RAW_TO_BATCH and count >= 2:
                pair_count = count // 2
                result.extend([RAW_TO_BATCH[material]] * pair_count)
                if count % 2:
                    result.append(material)
            else:
                result.extend([material] * count)

        return result

    # ──────────────────────────────────────────────────────────
    # Task 생성
    # ──────────────────────────────────────────────────────────

    def generate_task(self):
        task = Task()
        produce_ids, recycle_ids = self.select_products()

        for pid in produce_ids:
            order            = Order()
            order.order_type = OT_PRODUCE
            order.product_id = pid
            task.order_list.append(order)

        for pid in recycle_ids:
            order            = Order()
            order.order_type = OT_RECYCLE
            order.product_id = pid
            task.order_list.append(order)

        storage_materials = self._build_storage_material_list(produce_ids, recycle_ids)
        customer_products = list(recycle_ids)

        # 고정 아레나 배치: STATION_LAYOUT 기준으로 station_id/타입을 그대로 사용
        storage_buckets = self.split_materials(
            storage_materials, len(STORAGE_STATION_IDS)
        )

        arena_layout   = []
        storage_index  = 0

        for base_station_id in sorted(STATION_LAYOUT.keys()):
            station_type = STATION_LAYOUT[base_station_id]
            station_id = self._to_side_station_id(base_station_id)

            if station_type == ST_STORAGE:
                material_ids = self._apply_station_batch_ids(
                    storage_buckets[storage_index]
                )
                storage_index += 1
            elif station_type == ST_CUSTOMER:
                material_ids = customer_products
            else:  # ST_WORKBENCH / ST_HYBRID
                material_ids = []

            arena_layout.append({
                'station_type': station_type,
                'station_id':   station_id,
                'material_ids': material_ids,
            })

            station_msg              = Station()
            station_msg.station_name = f'station_{station_id}'
            station_msg.station_type = station_type
            station_msg.station_id   = station_id
            station_msg.material_ids = material_ids
            task.arena_layout.append(station_msg)

        return task, arena_layout

    # ──────────────────────────────────────────────────────────
    # 출력
    # ──────────────────────────────────────────────────────────

    def get_order_comment(self, order):
        return PRODUCT_DB.get(order.product_id, ('unknown', []))[0]

    def print_official_style(self, task, arena_layout):
        _, total_raw             = self._validate_raw_materials(task)
        raw_target, raw_variance = self.config['raw_mat']

        print(f'\n# {self.tier.capitalize()} – {self.stage.capitalize()}\n')
        print(f'# start_side      = {self.side.upper()}')
        print(f'# station_offset  = {self.station_offset}')
        print(f'# time_limit      = {self.config["time"]} min')
        print(f'# produce_count   = {self.produce_count}')
        print(f'# recycle_count   = {self.recycle_count}')
        print(f'# raw_materials   = {total_raw}  (목표: {raw_target}±{raw_variance})')
        print(f'# station_count   = {self.station_count} (고정)')
        if self.produce_count and self.recycle_count:
            print(f'# lifecycle_common(reuse)   = {self.lifecycle_common_materials}')
            print(f'# produce_initial_materials = {self.produce_initial_materials}')
            print(f'# recycle_leftover_materials= {self.recycle_leftover_materials}')
        print()

        print('order_list = ')
        print('{')
        for order in task.order_list:
            label   = 'P' if order.order_type == OT_PRODUCE else 'R'
            comment = self.get_order_comment(order)
            print(
                f'   order_type = {order.order_type} ; '
                f'product_id = {order.product_id:<18} '
                f'# {comment:<18} ({label})'
            )
        print('}\n')

        print('arena_layout = ')
        print('{')
        for station in arena_layout:
            material_text = ', '.join(str(x) for x in station['material_ids'])
            print(
                f"   station_type = {station['station_type']}; "
                f"station_id = {station['station_id']}; "
                f"material_ids = {{{material_text}}}"
            )
        print('}\n')

    # ──────────────────────────────────────────────────────────
    # 발행
    # ──────────────────────────────────────────────────────────

    def publish_task(self):
        if self.published:
            return
        self.task_pub.publish(self.task)
        self.get_logger().info(
            f'Task published to /sml/task | side={self.side} | '
            f'station_offset={self.station_offset}'
        )
        self.published = True


def main(args=None):
    rclpy.init(args=args)
    node = OrderServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()