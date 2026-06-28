"""Plan D planner constants and runtime configuration.

Slot convention used by Step.slide_ids:
    slot 0   : product / recycle-product default slot
    slot 1-5 : raw material slides
    slot 6-7 : AMR assembly slots / recycle auxiliary preload slots
"""

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

# Arena / station constants.
STATION_START_GOAL = 0
FIXED_WORKBENCH_STATION_ID = 6

# Latest AMR internal slot convention.
PRODUCT_SLOT_INDEX = 0
RAW_SLOT_INDICES = [1, 2, 3, 4, 5]
ASSEMBLY_SLOT_INDICES = [6, 7]
RAW_SLIDE_CAPACITY_UNITS = 3

# Backward-compatible aliases.
PRODUCT_SLOT = PRODUCT_SLOT_INDEX
RAW_SLIDE_SLOTS = RAW_SLOT_INDICES
ASSEMBLY_SLOTS = ASSEMBLY_SLOT_INDICES

# Runtime cost defaults.
AMR_SPEED = 1.50  # [m/s]
AMR_LOAD_TIME_SEC_PER_ITEM = 2.0
AMR_UNLOAD_TIME_SEC_PER_ITEM = 2.0
AMR_ASSEMBLE_TIME_SEC_PER_CONNECTION = 4.0
WB_PRODUCE_TIME_SEC_PER_CONNECTION = 10.0
WB_RECYCLE_TIME_SEC_PER_CONNECTION = 10.0
NAV_OVERHEAD_SEC = 0.0

STATION_COORD_JSON_PARAM = 'station_coord_json_path'
DEFAULT_STATION_COORD_JSON_PATH = (
    '/home/user/ros2_ws/src/sml_system_pkg/config/station_coordinates_a_zone.json'
)
WAYPOINT_YAML_PARAM = 'waypoint_yaml_path'
DEFAULT_WAYPOINT_YAML_PATH = (
    '/home/user/ros2_ws/src/sml_system_pkg/config/robocup_waypoint.yaml'
)

# Optional fixed costs added to waypoint-based navigation estimates.
# Keep the defaults at zero until measured averages are supplied as ROS params.
NAV_ALIGN_TIME_PARAM = 'nav_align_time_avg'
NAV_ALIGN_TIME_AVG = 0.0
NAV_POST_TIME_PARAM = 'nav_post_time_avg'
NAV_POST_TIME_AVG = 0.0


@dataclass
class PlannerConfig:
    use_time_cost: bool = True
    amr_speed_mps: float = AMR_SPEED
    station_coord_json_path: str = DEFAULT_STATION_COORD_JSON_PATH
    waypoint_yaml_path: str = DEFAULT_WAYPOINT_YAML_PATH
    nav_align_time_avg: float = NAV_ALIGN_TIME_AVG
    nav_post_time_avg: float = NAV_POST_TIME_AVG

    fixed_workbench_station_id: int = FIXED_WORKBENCH_STATION_ID
    station_start_goal: int = STATION_START_GOAL

    # Time model used for planning logs / simple decisions.
    amr_load_time_sec_per_item: float = AMR_LOAD_TIME_SEC_PER_ITEM
    amr_unload_time_sec_per_item: float = AMR_UNLOAD_TIME_SEC_PER_ITEM
    amr_assemble_time_sec_per_connection: float = AMR_ASSEMBLE_TIME_SEC_PER_CONNECTION
    wb_produce_time_sec_per_connection: float = WB_PRODUCE_TIME_SEC_PER_CONNECTION
    wb_recycle_time_sec_per_connection: float = WB_RECYCLE_TIME_SEC_PER_CONNECTION
    nav_overhead_sec: float = NAV_OVERHEAD_SEC

    # Capacity hints.
    raw_slide_capacity_units: int = RAW_SLIDE_CAPACITY_UNITS
    max_amr_assembly_jobs: int = len(ASSEMBLY_SLOT_INDICES)
    max_recycle_preload_products: int = 3
