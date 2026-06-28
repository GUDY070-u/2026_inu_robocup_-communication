#!/usr/bin/env python3
"""SML order_server.

공식 예시 task builder 형식을 유지하면서 기존 sml_msgs / sml_system_pkg 구조에 맞춰
/sml/task 로 Task를 발행하는 노드.

주요 정책
- station_name은 공식 이름(side_a_storage_1 등)을 유지한다.
- 선택한 start_side에 따라 A면 station_id 1~8, B면 station_id 9~16으로 발행한다.
- 공식 Beginner preset의 batch ID(10, 30, 40 등)는 그대로 유지한다.
- 랜덤 모드에서도 같은 원재료 2개를 batch로 임의 압축하지 않는다.
- batch는 arena_layout에 들어갈 수 있고, planner가 raw 단위로 해석해야 한다.
"""

import random
from collections import Counter
from typing import Dict, Iterable, List, Sequence, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

TASK_QOS = QoSProfile(
    depth=1,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    reliability=QoSReliabilityPolicy.RELIABLE,
)

from sml_msgs.msg import Order, Station, Task


# ─────────────────────────────────────────────────────────────
# Message constants
# ─────────────────────────────────────────────────────────────
ST_STORAGE = Station.ST_STORAGE
ST_WORKBENCH = Station.ST_WORKBENCH
ST_CUSTOMER = Station.ST_CUSTOMER
ST_HYBRID = Station.ST_HYBRID

OT_PRODUCE = Order.OT_PRODUCE
OT_RECYCLE = Order.OT_RECYCLE


# ─────────────────────────────────────────────────────────────
# Object IDs
#   batch ID는 공식 정의대로 arena_layout에 유지한다.
#   planner/manager/arm 명령 단계에서는 raw ID로 풀어서 사용해야 한다.
# ─────────────────────────────────────────────────────────────
BATCH_TO_RAW = {
    10: 1,
    20: 2,
    30: 3,
    40: 4,
    50: 5,
    60: 6,
    70: 7,
    80: 8,
}
MIXED_BATCH = 90

PRODUCT_DB = {
    34: ("Battery", [3, 4]),
    13: ("Magnet", [1, 3]),
    81: ("E-Stop", [8, 1]),
    442: ("Carrot", [4, 4, 2]),
    241: ("Traffic Light", [2, 4, 1]),
    462: ("Small Tree", [4, 6, 2]),
    # Plan D 조립 순서 기준: Hammer는 product_id는 711 유지, 재료 순서는 [1, 1, 7]
    711: ("Hammer", [1, 1, 7]),
    4482: ("Big Carrot", [4, 4, 8, 2]),
    8518: ("Burger", [8, 5, 1, 8]),
    48132: ("Ice Cream", [4, 8, 1, 3, 2]),
    46262: ("Big Tree", [4, 6, 2, 6, 2]),
}

BEGINNER_PRODUCT_IDS = [34, 13, 81, 442, 241, 462, 711]
ADVANCED_PRODUCT_IDS = [4482, 8518, 48132, 46262]

TIER_PRODUCT_CANDIDATES = {
    "entry": BEGINNER_PRODUCT_IDS,
    "beginner": BEGINNER_PRODUCT_IDS,
    "advanced": BEGINNER_PRODUCT_IDS + ADVANCED_PRODUCT_IDS,
    "expert": BEGINNER_PRODUCT_IDS + ADVANCED_PRODUCT_IDS,
}


# ─────────────────────────────────────────────────────────────
# Task complexity table
# ─────────────────────────────────────────────────────────────
TIER_STAGE_CONFIG = {
    ("entry", "production"): {"time": 5, "orders": 1, "returns": 0, "raw_mat": (2, 1), "products": (1, 0), "fleet": (1, 3)},
    ("entry", "lifecycle"): {"time": 10, "orders": 2, "returns": 1, "raw_mat": (7, 1), "products": (3, 1), "fleet": (1, 3)},
    ("beginner", "production"): {"time": 5, "orders": 2, "returns": 0, "raw_mat": (5, 1), "products": (2, 0), "fleet": (1, 3)},
    ("beginner", "recycling"): {"time": 5, "orders": 0, "returns": 2, "raw_mat": (5, 1), "products": (2, 0), "fleet": (1, 3)},
    ("beginner", "lifecycle"): {"time": 10, "orders": 3, "returns": 2, "raw_mat": (10, 3), "products": (5, 1), "fleet": (1, 3)},
    ("advanced", "production"): {"time": 10, "orders": 5, "returns": 0, "raw_mat": (10, 3), "products": (5, 1), "fleet": (1, 6)},
    ("advanced", "recycling"): {"time": 10, "orders": 0, "returns": 5, "raw_mat": (10, 3), "products": (5, 1), "fleet": (1, 6)},
    ("advanced", "lifecycle"): {"time": 15, "orders": 5, "returns": 5, "raw_mat": (20, 8), "products": (10, 2), "fleet": (1, 6)},
    ("expert", "production"): {"time": 20, "orders": 20, "returns": 0, "raw_mat": (40, 15), "products": (20, 5), "fleet": (3, 12)},
    ("expert", "recycling"): {"time": 20, "orders": 0, "returns": 20, "raw_mat": (40, 15), "products": (20, 5), "fleet": (3, 12)},
    ("expert", "lifecycle"): {"time": 30, "orders": 30, "returns": 20, "raw_mat": (100, 30), "products": (50, 10), "fleet": (3, 12)},
}

TIER_NAMES = {1: "entry", 2: "beginner", 3: "advanced", 4: "expert"}
STAGE_NAMES = {1: "production", 2: "recycling", 3: "lifecycle"}


# ─────────────────────────────────────────────────────────────
# Station naming / ID mapping
#   local station: 1~8
#   A actual ID: 1~8
#   B actual ID: 9~16
# ─────────────────────────────────────────────────────────────
STATION_DEFS = [
    ("storage_1", 1, ST_STORAGE),
    ("storage_2", 2, ST_STORAGE),
    ("workbench_1", 3, ST_WORKBENCH),
    ("storage_3", 4, ST_STORAGE),
    ("hybrid_1", 5, ST_HYBRID),
    ("workbench_2", 6, ST_WORKBENCH),
    ("workbench_3", 7, ST_WORKBENCH),
    ("customer_1", 8, ST_CUSTOMER),
]

STATION_COUNT = 8


def normalize_side(side: str) -> str:
    s = str(side).strip().lower()
    if s in ("a", "side_a", "1"):
        return "a"
    if s in ("b", "side_b", "2"):
        return "b"
    raise ValueError(f"invalid side: {side}")


def side_prefix(side: str) -> str:
    return "side_a" if normalize_side(side) == "a" else "side_b"


def station_offset(side: str) -> int:
    return 0 if normalize_side(side) == "a" else 8


def make_order(order_type: int, _name: str, product_id: int) -> Order:
    """Order.msg에는 name 필드가 없으므로 _name은 출력/호환 목적 인자다."""
    order = Order()
    order.order_type = int(order_type)
    order.product_id = int(product_id)
    return order


def make_station(station_name: str, station_type: int, station_id: int, material_ids: Sequence[int]) -> Station:
    station = Station()
    station.station_name = station_name
    station.station_type = int(station_type)
    station.station_id = int(station_id)
    station.material_ids = [int(x) for x in material_ids]
    return station


def fill_task(orders: Sequence[Order], material_map: Dict[str, Sequence[int]], start_side: str) -> Task:
    """공식 builder 스타일의 material_map에서 선택한 side만 Task로 변환한다."""
    side = normalize_side(start_side)
    prefix = side_prefix(side)
    offset = station_offset(side)

    task = Task()
    task.order_list = list(orders)

    for suffix, local_id, station_type in STATION_DEFS:
        name = f"{prefix}_{suffix}"
        materials = list(material_map.get(name, []))
        actual_station_id = local_id + offset
        task.arena_layout.append(
            make_station(
                station_name=name,
                station_type=station_type,
                station_id=actual_station_id,
                material_ids=materials,
            )
        )
    return task


# ─────────────────────────────────────────────────────────────
# Official preset builders
# ─────────────────────────────────────────────────────────────
def build_production_beginner_task(start_side: str) -> Task:
    orders = [
        make_order(OT_PRODUCE, "produce_estop", 81),
        make_order(OT_PRODUCE, "produce_carrot", 442),
    ]
    return fill_task(
        orders,
        {
            "side_a_storage_1": [2, 1],
            "side_a_storage_2": [8],
            "side_a_storage_3": [40],
            "side_b_storage_1": [2, 1],
            "side_b_storage_2": [8],
            "side_b_storage_3": [40],
        },
        start_side,
    )


def build_recycling_beginner_task(start_side: str) -> Task:
    orders = [
        make_order(OT_RECYCLE, "recycle_magnet", 13),
        make_order(OT_RECYCLE, "recycle_traffic_light", 241),
    ]
    return fill_task(
        orders,
        {
            "side_a_storage_1": [1, 3],
            "side_a_storage_3": [4],
            "side_a_hybrid_1": [2],
            "side_b_storage_1": [1, 3],
            "side_b_storage_3": [4],
            "side_b_hybrid_1": [2],
        },
        start_side,
    )


def build_lifecycle_beginner_task(start_side: str) -> Task:
    orders = [
        make_order(OT_PRODUCE, "produce_magnet", 13),
        make_order(OT_PRODUCE, "produce_small_tree", 462),
        make_order(OT_PRODUCE, "produce_hammer", 711),
        make_order(OT_RECYCLE, "recycle_carrot", 442),
        make_order(OT_RECYCLE, "recycle_battery", 34),
    ]
    return fill_task(
        orders,
        {
            "side_a_storage_1": [10, 2],
            "side_a_storage_2": [4, 30],
            "side_a_storage_3": [6],
            "side_a_hybrid_1": [7],
            "side_a_customer_1": [34, 442],
            "side_b_storage_1": [10, 2],
            "side_b_storage_2": [4, 30],
            "side_b_storage_3": [6],
            "side_b_hybrid_1": [7],
            "side_b_customer_1": [34, 442],
        },
        start_side,
    )


OFFICIAL_PRESET_BUILDERS = {
    ("beginner", "production"): build_production_beginner_task,
    ("beginner", "recycling"): build_recycling_beginner_task,
    ("beginner", "lifecycle"): build_lifecycle_beginner_task,
}


# ─────────────────────────────────────────────────────────────
# Helper functions for random mode
# ─────────────────────────────────────────────────────────────
def product_materials(product_ids: Iterable[int]) -> List[int]:
    materials: List[int] = []
    for pid in product_ids:
        materials.extend(PRODUCT_DB[int(pid)][1])
    return materials


def multiset_common_preserve_order(left: Sequence[int], right: Sequence[int]) -> List[int]:
    right_count = Counter(right)
    common: List[int] = []
    for item in left:
        if right_count[item] > 0:
            common.append(item)
            right_count[item] -= 1
    return common


def subtract_preserve_order(base: Sequence[int], remove: Sequence[int]) -> List[int]:
    remove_count = Counter(remove)
    result: List[int] = []
    for item in base:
        if remove_count[item] > 0:
            remove_count[item] -= 1
        else:
            result.append(item)
    return result


def split_round_robin(items: Sequence[int], bucket_count: int) -> List[List[int]]:
    buckets = [[] for _ in range(bucket_count)]
    if bucket_count <= 0:
        return buckets
    for i, item in enumerate(items):
        buckets[i % bucket_count].append(int(item))
    return buckets


def expand_batch_for_count(material_ids: Sequence[int]) -> List[int]:
    """raw count 검증용. batch는 최소 5개로 계산한다. mixed batch는 0개로 둔다."""
    expanded: List[int] = []
    for mid in material_ids:
        if mid in BATCH_TO_RAW:
            expanded.extend([BATCH_TO_RAW[mid]] * 5)
        elif mid == MIXED_BATCH:
            continue
        else:
            expanded.append(mid)
    return expanded


class OrderServer(Node):
    def __init__(self):
        super().__init__("order_server")

        self.declare_parameter("task_topic", "/sml/task")
        self.declare_parameter("auto_publish", False)
        self.declare_parameter("start_side", "")
        self.declare_parameter("mode", "")  # preset | random | "" interactive
        self.declare_parameter("tier", "")
        self.declare_parameter("stage", "")
        self.declare_parameter("seed", -1)

        self.task_topic = self.get_parameter("task_topic").value
        self.task_pub = self.create_publisher(Task, self.task_topic, TASK_QOS)
        self.published = False

        seed = int(self.get_parameter("seed").value)
        if seed >= 0:
            random.seed(seed)
            self.get_logger().info(f"random seed={seed}")

        self.start_side = self._get_start_side()
        self.tier, self.stage = self._get_tier_stage()
        self.config = TIER_STAGE_CONFIG[(self.tier, self.stage)]

        self.mode = self._get_mode()
        self.lifecycle_common_materials: List[int] = []
        self.produce_initial_materials: List[int] = []
        self.recycle_leftover_materials: List[int] = []

        if self.mode == "preset":
            builder = OFFICIAL_PRESET_BUILDERS.get((self.tier, self.stage))
            if builder is None:
                print("선택한 Tier/Stage에는 공식 preset이 없어 random mode로 생성합니다.")
                self.mode = "random"
                self.task = self._generate_random_with_validation()
            else:
                self.task = builder(self.start_side)
                self._update_lifecycle_meta_from_task(self.task)
        else:
            self.task = self._generate_random_with_validation()

        self.print_official_style(self.task)

        auto_publish = bool(self.get_parameter("auto_publish").value)
        if auto_publish:
            self.publish_task()
        else:
            input("엔터를 누르면 task를 publish합니다 (플래너로 전달): ")
            self.publish_task()

    # ──────────────────────────────────────────────────────────
    # Interactive / parameter input
    # ──────────────────────────────────────────────────────────
    def _get_start_side(self) -> str:
        param_side = str(self.get_parameter("start_side").value).strip()
        if param_side:
            return normalize_side(param_side)
        side_num = self.get_input_int("Start 위치 선택 (1: A, 2: B): ", valid_values=[1, 2])
        return "a" if side_num == 1 else "b"

    def _get_tier_stage(self) -> Tuple[str, str]:
        param_tier = str(self.get_parameter("tier").value).strip().lower()
        param_stage = str(self.get_parameter("stage").value).strip().lower()

        if param_tier and param_stage:
            tier = param_tier
            stage = param_stage
            if (tier, stage) not in TIER_STAGE_CONFIG:
                raise ValueError(f"invalid tier/stage: {tier}/{stage}")
            return tier, stage

        tier_num = self.get_input_int(
            "Tier 선택 (1: Entry, 2: Beginner, 3: Advanced, 4: Expert): ",
            valid_values=[1, 2, 3, 4],
        )
        tier = TIER_NAMES[tier_num]

        if tier == "entry":
            stage_num = self.get_input_int(
                "Stage 선택 (1: Production, 3: Lifecycle): ",
                valid_values=[1, 3],
            )
        else:
            stage_num = self.get_input_int(
                "Stage 선택 (1: Production, 2: Recycling, 3: Lifecycle): ",
                valid_values=[1, 2, 3],
            )
        return tier, STAGE_NAMES[stage_num]

    def _get_mode(self) -> str:
        param_mode = str(self.get_parameter("mode").value).strip().lower()
        if param_mode in ("preset", "random"):
            return param_mode

        has_preset = (self.tier, self.stage) in OFFICIAL_PRESET_BUILDERS
        if has_preset:
            mode_num = self.get_input_int(
                "Task 생성 방식 선택 (1: 공식 preset, 2: random): ",
                valid_values=[1, 2],
            )
            return "preset" if mode_num == 1 else "random"
        return "random"

    @staticmethod
    def get_input_int(msg: str, valid_values=None, min_value=None) -> int:
        while True:
            try:
                value = int(input(msg))
                if valid_values is not None and value not in valid_values:
                    print(f"입력 가능 값: {valid_values}")
                    continue
                if min_value is not None and value < min_value:
                    print(f"{min_value} 이상의 값을 입력하세요.")
                    continue
                return value
            except ValueError:
                print("정수를 입력하세요.")

    # ──────────────────────────────────────────────────────────
    # Random generation
    # ──────────────────────────────────────────────────────────
    def _select_random_products(self) -> Tuple[List[int], List[int]]:
        produce_count = self.config["orders"]
        recycle_count = self.config["returns"]
        total_count = produce_count + recycle_count
        candidates = list(TIER_PRODUCT_CANDIDATES[self.tier])

        if total_count > len(candidates):
            selected = [random.choice(candidates) for _ in range(total_count)]
        else:
            selected = random.sample(candidates, total_count)

        return selected[:produce_count], selected[produce_count:]

    def _build_storage_materials_for_random(self, produce_ids: Sequence[int], recycle_ids: Sequence[int]) -> List[int]:
        produce_materials = product_materials(produce_ids)
        recycle_materials = product_materials(recycle_ids)

        if self.config["orders"] > 0 and self.config["returns"] > 0:
            common = multiset_common_preserve_order(produce_materials, recycle_materials)
        else:
            common = []

        produce_initial = subtract_preserve_order(produce_materials, common)
        recycle_leftover = subtract_preserve_order(recycle_materials, common)

        self.lifecycle_common_materials = common
        self.produce_initial_materials = produce_initial
        self.recycle_leftover_materials = recycle_leftover
        return produce_initial + recycle_leftover

    def _generate_random_task(self) -> Task:
        produce_ids, recycle_ids = self._select_random_products()

        orders: List[Order] = []
        for pid in produce_ids:
            orders.append(make_order(OT_PRODUCE, f"produce_{pid}", pid))
        for pid in recycle_ids:
            orders.append(make_order(OT_RECYCLE, f"recycle_{pid}", pid))

        storage_materials = self._build_storage_materials_for_random(produce_ids, recycle_ids)
        customer_products = list(recycle_ids)

        prefix = side_prefix(self.start_side)
        material_map: Dict[str, List[int]] = {}

        # 공식 예시처럼 storage_1, storage_2, storage_3, hybrid_1을 재료 공급 후보로 사용한다.
        supply_station_names = [
            f"{prefix}_storage_1",
            f"{prefix}_storage_2",
            f"{prefix}_storage_3",
            f"{prefix}_hybrid_1",
        ]
        buckets = split_round_robin(storage_materials, len(supply_station_names))
        for name, bucket in zip(supply_station_names, buckets):
            # 중요: random mode에서도 같은 raw 2개를 batch로 임의 압축하지 않는다.
            material_map[name] = bucket

        if customer_products:
            material_map[f"{prefix}_customer_1"] = customer_products

        return fill_task(orders, material_map, self.start_side)

    def _generate_random_with_validation(self) -> Task:
        max_retry = 30
        last_task = None
        last_total = 0
        raw_target, raw_variance = self.config["raw_mat"]
        low, high = raw_target - raw_variance, raw_target + raw_variance

        for attempt in range(1, max_retry + 1):
            task = self._generate_random_task()
            total = self._raw_material_count_from_orders(task.order_list)
            last_task, last_total = task, total
            if low <= total <= high:
                if attempt > 1:
                    print(f"✓ {attempt}회 시도 만에 검증 통과 (원자재: {total}개)")
                return task
            print(
                f"  시도 {attempt}/{max_retry}: 원자재 {total}개 → "
                f"목표 범위 [{low}, {high}] 벗어남, 재생성 중..."
            )

        print(f"⚠ 경고: {max_retry}회 재시도 후에도 원자재 범위를 만족하지 못했습니다. 마지막 결과({last_total}개)를 사용합니다.")
        return last_task

    # ──────────────────────────────────────────────────────────
    # Meta / validation
    # ──────────────────────────────────────────────────────────
    def _raw_material_count_from_orders(self, orders: Sequence[Order]) -> int:
        total = 0
        for order in orders:
            if order.product_id in PRODUCT_DB:
                total += len(PRODUCT_DB[order.product_id][1])
        return total

    def _update_lifecycle_meta_from_task(self, task: Task) -> None:
        produce_ids = [o.product_id for o in task.order_list if o.order_type == OT_PRODUCE]
        recycle_ids = [o.product_id for o in task.order_list if o.order_type == OT_RECYCLE]
        produce_materials = product_materials(produce_ids)
        recycle_materials = product_materials(recycle_ids)
        common = multiset_common_preserve_order(produce_materials, recycle_materials) if produce_ids and recycle_ids else []
        self.lifecycle_common_materials = common
        self.produce_initial_materials = subtract_preserve_order(produce_materials, common)
        self.recycle_leftover_materials = subtract_preserve_order(recycle_materials, common)

    def _raw_material_supply_count_from_arena(self, task: Task) -> int:
        total = 0
        for station in task.arena_layout:
            if station.station_type in (ST_STORAGE, ST_HYBRID):
                total += len(expand_batch_for_count(station.material_ids))
        return total

    # ──────────────────────────────────────────────────────────
    # Printing
    # ──────────────────────────────────────────────────────────
    @staticmethod
    def product_name(product_id: int) -> str:
        return PRODUCT_DB.get(int(product_id), ("unknown", []))[0]

    def print_official_style(self, task: Task) -> None:
        raw_target, raw_variance = self.config["raw_mat"]
        order_raw_total = self._raw_material_count_from_orders(task.order_list)
        supply_raw_total = self._raw_material_supply_count_from_arena(task)
        produce_count = sum(1 for o in task.order_list if o.order_type == OT_PRODUCE)
        recycle_count = sum(1 for o in task.order_list if o.order_type == OT_RECYCLE)

        print(f"\n# {self.tier.capitalize()} – {self.stage.capitalize()}\n")
        print(f"# start_side      = {self.start_side.upper()}")
        print(f"# station_offset  = {station_offset(self.start_side)}")
        print(f"# generation_mode = {self.mode}")
        print(f"# time_limit      = {self.config['time']} min")
        print(f"# produce_count   = {produce_count}")
        print(f"# recycle_count   = {recycle_count}")
        print(f"# raw_materials   = {order_raw_total}  (목표: {raw_target}±{raw_variance})")
        print(f"# supply_raw_est  = {supply_raw_total}  (batch는 최소 5개로 추정)")
        print(f"# station_count   = {STATION_COUNT} (고정)")
        if produce_count and recycle_count:
            print(f"# lifecycle_common(reuse)   = {self.lifecycle_common_materials}")
            print(f"# produce_initial_materials = {self.produce_initial_materials}")
            print(f"# recycle_leftover_materials= {self.recycle_leftover_materials}")
        print()

        print("order_list = ")
        print("{")
        for order in task.order_list:
            label = "P" if order.order_type == OT_PRODUCE else "R"
            print(
                f"   order_type = {order.order_type} ; "
                f"product_id = {order.product_id:<18} "
                f"# {self.product_name(order.product_id):<18} ({label})"
            )
        print("}\n")

        print("arena_layout = ")
        print("{")
        for station in task.arena_layout:
            material_text = ", ".join(str(x) for x in station.material_ids)
            print(
                f"   station_type = {station.station_type}; "
                f"station_id = {station.station_id}; "
                f"station_name = {station.station_name}; "
                f"material_ids = {{{material_text}}}"
            )
        print("}\n")

    # ──────────────────────────────────────────────────────────
    # Publish
    # ──────────────────────────────────────────────────────────
    def publish_task(self) -> None:
        if self.published:
            return
        self.task_pub.publish(self.task)
        self.get_logger().info(
            f"Task published to {self.task_topic} | side={self.start_side} | mode={self.mode} | "
            f"orders={len(self.task.order_list)}, stations={len(self.task.arena_layout)}"
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


if __name__ == "__main__":
    main()