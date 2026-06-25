"""Planner constants and runtime configuration."""

from dataclasses import dataclass


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

# 경기 / 시스템 시간 가정값
AMR_SPEED = 13.0              # [m/s]
VISION_TIME_AVG = 2.0         # [s] 비전 인식 평균값
LOAD_TIME = 10.0              # [s/item]
UNLOAD_TIME = 10.0            # [s/item]
ASSEMBLY_PAIR_TIME = 12.5     # [s/connection]
DISASSEMBLY_PAIR_TIME = 12.5  # [s/disconnection]

# AMR 적재 제약
MAX_RAW_CAPACITY = 5
MAX_PRODUCT_CAPACITY = 1

# 대회 arena_layout에는 포함되지 않는 AMR의 시작/복귀 지점.
STATION_START_GOAL = 0

# 실제 시스템에서 작업로봇이 연결된 WORKBENCH station.
FIXED_WORKBENCH_STATION_ID = 6

STATION_COORD_JSON_PARAM = 'station_coord_json_path'
DEFAULT_STATION_COORD_JSON_PATH = (
    '/home/user/ros2_ws/src/sml_system_pkg/config/station_coordinates_a_zone.json'
)


@dataclass
class PlannerConfig:
    """Runtime configuration passed from the ROS node into the pure planner."""

    use_time_cost: bool = True
    amr_speed_mps: float = AMR_SPEED
    station_coord_json_path: str = DEFAULT_STATION_COORD_JSON_PATH

    fixed_workbench_station_id: int = FIXED_WORKBENCH_STATION_ID
    station_start_goal: int = STATION_START_GOAL
    max_raw_capacity: int = MAX_RAW_CAPACITY
    max_product_capacity: int = MAX_PRODUCT_CAPACITY

    vision_time_avg: float = VISION_TIME_AVG
    load_time: float = LOAD_TIME
    unload_time: float = UNLOAD_TIME
    assembly_pair_time: float = ASSEMBLY_PAIR_TIME
    disassembly_pair_time: float = DISASSEMBLY_PAIR_TIME
