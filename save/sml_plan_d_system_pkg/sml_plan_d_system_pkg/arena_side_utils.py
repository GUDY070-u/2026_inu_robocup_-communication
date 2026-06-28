"""A/B arena station id conversion helpers for Plan D."""


def normalize_side(side):
    value = str(side or 'a').strip().lower()
    if value in ('a', 'side_a', 'left'):
        return 'a'
    if value in ('b', 'side_b', 'right'):
        return 'b'
    return 'a'


def side_to_fixed_workbench_station(side):
    side = normalize_side(side)
    return 15 if side == 'b' else 6


def amr_station_to_planner_station(station_id, side):
    """Convert real AMR station id to planner-local A-side id."""
    station_id = int(station_id)
    side = normalize_side(side)
    if station_id == 0:
        return 0
    if side == 'b':
        return station_id - 8
    return station_id


def planner_station_to_amr_station(station_id, side):
    """Convert planner-local A-side station id to real AMR station id."""
    station_id = int(station_id)
    side = normalize_side(side)
    if station_id == 0:
        return 0
    if side == 'b':
        return station_id + 8
    return station_id


def nav_target_for_station(station_id, side):
    """Return numeric nav target. Home/start/goal is always station 0."""
    station_id = int(station_id)
    if station_id == 0:
        return 0
    return int(station_id)
