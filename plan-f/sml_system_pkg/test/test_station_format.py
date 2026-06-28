from sml_msgs.msg import Station

from sml_system_pkg.arena_side_utils import (
    amr_station_to_planner_station,
    planner_station_to_amr_station,
    side_to_fixed_workbench_station,
    side_to_start_goal_station,
)
from sml_system_pkg.order_server import STATION_DEFS, station_offset


def test_side_station_id_conversions():
    assert side_to_start_goal_station('a') == 0
    assert side_to_start_goal_station('b') == 9
    assert side_to_fixed_workbench_station('a') == 6
    assert side_to_fixed_workbench_station('b') == 16

    assert [planner_station_to_amr_station(i, 'a') for i in range(9)] == list(range(9))
    assert [planner_station_to_amr_station(i, 'b') for i in range(9)] == list(range(9, 18))
    assert [amr_station_to_planner_station(i, 'b') for i in range(9, 18)] == list(range(9))


def test_station_types_match_arena_format():
    local_types = {local_id: station_type for _, local_id, station_type in STATION_DEFS}

    assert station_offset('a') == 0
    assert station_offset('b') == 9
    assert {sid for sid, stype in local_types.items() if stype == Station.ST_STORAGE} == {
        1, 2, 4, 5,
    }
    assert {sid for sid, stype in local_types.items() if stype == Station.ST_WORKBENCH} == {
        3, 6, 7,
    }
    assert {sid for sid, stype in local_types.items() if stype == Station.ST_CUSTOMER} == {
        8,
    }
