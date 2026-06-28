"""Plan D planner constants and runtime configuration."""

from dataclasses import dataclass

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

# Product id stays 711, but Plan D assembly order is [1, 1, 7].
PRODUCT_MATERIALS = {
    34: [3, 4],
    13: [1, 3],
    81: [8, 1],
    442: [4, 4, 2],
    241: [2, 4, 1],
    462: [4, 6, 2],
    711: [1, 1, 7],
    4482: [4, 4, 8, 2],
    8518: [8, 5, 1, 8],
    48132: [4, 8, 1, 3, 2],
    46262: [4, 6, 2, 6, 2],
}

WB_ONLY_PRODUCTS = {8518, 48132, 46262}
AMR_CAPABLE_PRODUCTS = set(PRODUCT_MATERIALS) - WB_ONLY_PRODUCTS

RAW_TO_BATCH = {1: 10, 2: 20, 3: 30, 4: 40, 5: 50, 6: 60, 7: 70, 8: 80}
BATCH_TO_RAW = {batch: raw for raw, batch in RAW_TO_BATCH.items()}
BATCH_SIZE = 5

STATION_START_GOAL = 0
RAW_SLOT_INDICES = [0, 1, 2, 3, 4]
ASSEMBLY_SLOT_INDICES = [5, 6]
PRODUCT_SLOT_INDEX = 9
RAW_SLIDE_CAPACITY_UNITS = 3

# ─────────────────────────────────────────────────────────────
# 시간 모델 기본값
# ─────────────────────────────────────────────────────────────
# 실제 로봇 값이 정해지기 전까지는 ROS 파라미터로 쉽게 바꿀 수 있도록
# planning_node와 mock node 모두 같은 이름의 시간 파라미터를 사용한다.
AMR_SPEED = 1.50                         # [m/s] mock/nav 거리 기반 시간 계산용
AMR_LOAD_TIME_SEC_PER_ITEM = 2.0          # [s/item] AMR raw/product 적재
AMR_UNLOAD_TIME_SEC_PER_ITEM = 2.0        # [s/item] AMR raw/product 하역
AMR_ASSEMBLE_TIME_SEC_PER_CONNECTION = 4.0  # [s/connection] AMR 내부 조립
WB_PRODUCE_TIME_SEC_PER_CONNECTION = 10.0   # [s/connection] workbench 조립
WB_RECYCLE_TIME_SEC_PER_CONNECTION = 10.0   # [s/connection] workbench 분해
NAV_OVERHEAD_SEC = 0.0                    # [s] 이동 시작/정지 오버헤드

STATION_COORD_JSON_PARAM = 'station_coord_json_path'
DEFAULT_STATION_COORD_JSON_PATH = (
    '/home/user/ros2_ws/src/sml_system_pkg/config/station_coordinates_a_zone.json'
)


@dataclass
class PlannerConfig:
    use_time_cost: bool = True
    amr_speed_mps: float = AMR_SPEED
    station_coord_json_path: str = DEFAULT_STATION_COORD_JSON_PATH
    fixed_workbench_station_id: int = 6
    station_start_goal: int = STATION_START_GOAL

    # 시간 추정/로그용. 실제 실행 시간은 mock node의 파라미터도 함께 맞춰야 한다.
    amr_load_time_sec_per_item: float = AMR_LOAD_TIME_SEC_PER_ITEM
    amr_unload_time_sec_per_item: float = AMR_UNLOAD_TIME_SEC_PER_ITEM
    amr_assemble_time_sec_per_connection: float = AMR_ASSEMBLE_TIME_SEC_PER_CONNECTION
    wb_produce_time_sec_per_connection: float = WB_PRODUCE_TIME_SEC_PER_CONNECTION
    wb_recycle_time_sec_per_connection: float = WB_RECYCLE_TIME_SEC_PER_CONNECTION
    nav_overhead_sec: float = NAV_OVERHEAD_SEC
