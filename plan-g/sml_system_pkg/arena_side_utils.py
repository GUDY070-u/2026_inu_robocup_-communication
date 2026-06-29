"""A/B arena station id conversion helpers for Plan D."""

SIDE_A_START_GOAL = 0
SIDE_B_START_GOAL = 9
SIDE_A_STATION_OFFSET = 0
SIDE_B_STATION_OFFSET = 9


def normalize_side(side):
    value = str(side or 'a').strip().lower()
    if value in ('a', 'side_a', 'left'):
        return 'a'
    if value in ('b', 'side_b', 'right'):
        return 'b'
    return 'a'


def side_to_fixed_workbench_station(side):
    """Return the real AMR station id of the fixed assembly robot/workbench."""
    side = normalize_side(side)
    return 16 if side == 'b' else 6


def side_to_start_goal_station(side):
    """Return the real start/goal station id for the selected arena side."""
    return SIDE_B_START_GOAL if normalize_side(side) == 'b' else SIDE_A_START_GOAL


def amr_station_to_planner_station(station_id, side):
    """Convert real AMR station id to planner-local station id.

    A side: 0~8  -> 0~8
    B side: 9~17 -> 0~8
    """
    station_id = int(station_id)
    side = normalize_side(side)
    if side == 'b':
        return station_id - SIDE_B_STATION_OFFSET
    return station_id


def planner_station_to_amr_station(station_id, side):
    """Convert planner-local station id to real AMR station id."""
    station_id = int(station_id)
    side = normalize_side(side)
    if side == 'b':
        return station_id + SIDE_B_STATION_OFFSET
    return station_id


def nav_target_for_station(station_id, side=None):
    """Return the numeric station id used by NavTask."""
    return int(station_id)
