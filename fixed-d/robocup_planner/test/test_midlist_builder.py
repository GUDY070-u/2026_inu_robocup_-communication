"""Unit tests for midlist_builder.py"""
import pytest
from collections import Counter
from unittest.mock import MagicMock
from robocup_planner.planning.midlist_builder import (
    build_storage_midlist,
    build_recycle_phase_entries,
    build_full_midlist,
    check_storage_satisfies,
    build_mid,
)


def make_calc(positions: dict):
    """Return a mock DistanceCalculator backed by the given {station_id: (x, y)} map."""
    import math
    calc = MagicMock()

    def get_pos(sid):
        return positions.get(sid)

    def point_to_station(x, y, sid):
        pos = positions.get(sid)
        if pos is None:
            return float('inf')
        return math.sqrt((x - pos[0]) ** 2 + (y - pos[1]) ** 2)

    calc.get_position.side_effect = get_pos
    calc.point_to_station.side_effect = point_to_station
    return calc


POSITIONS = {
    0: (0.0, 0.0),   # home
    1: (1.0, 0.0),   # station 1 — distance 1.0 from home
    2: (3.0, 0.0),   # station 2 — distance 3.0 from home
    3: (2.0, 0.0),   # station 3 — distance 2.0 from home
    5: (0.5, 0.0),   # workbench — distance 0.5 from home
}


class TestBuildStorageMidlist:
    def setup_method(self):
        self.calc = make_calc(POSITIONS)
        self.storage = [
            {'station_id': 2, 'material_ids': [5, 1]},
            {'station_id': 1, 'material_ids': [3]},
            {'station_id': 3, 'material_ids': [4, 2]},
        ]

    def test_sorted_by_distance_from_home(self):
        result = build_storage_midlist(self.storage, self.calc, ref_station_id=0)
        ids = [e['station_id'] for e in result]
        # dist(home→1)=1.0, dist(home→3)=2.0, dist(home→2)=3.0
        assert ids == [1, 3, 2]

    def test_entries_not_recycle_pickup(self):
        result = build_storage_midlist(self.storage, self.calc, ref_station_id=0)
        for e in result:
            assert e['is_recycle_pickup'] is False
            assert e['recycle_product_id'] is None

    def test_materials_preserved(self):
        result = build_storage_midlist(self.storage, self.calc, ref_station_id=0)
        by_id = {e['station_id']: e for e in result}
        assert by_id[2]['materials'] == [5, 1]
        assert by_id[1]['materials'] == [3]

    def test_ref_from_workbench(self):
        # From workbench (station 5 at x=0.5), station 1 (x=1.0) is closest
        result = build_storage_midlist(self.storage, self.calc, ref_station_id=5)
        ids = [e['station_id'] for e in result]
        assert ids[0] == 1


class TestBuildRecyclePhaseEntries:
    def setup_method(self):
        self.calc = make_calc(POSITIONS)

    def test_sorted_by_distance(self):
        orders = [
            {'station_id': 2, 'product_id': 81},
            {'station_id': 1, 'product_id': 34},
        ]
        result = build_recycle_phase_entries([], orders, self.calc, ref_station_id=0)
        assert result[0]['station_id'] == 1  # closer to home
        assert result[1]['station_id'] == 2

    def test_marked_as_recycle_pickup(self):
        orders = [{'station_id': 1, 'product_id': 81}]
        result = build_recycle_phase_entries([], orders, self.calc, ref_station_id=0)
        assert result[0]['is_recycle_pickup'] is True
        assert result[0]['recycle_product_id'] == 81
        assert result[0]['materials'] == []


class TestCheckStorageSatisfies:
    def test_satisfied(self):
        midlist = [
            {'materials': [8, 1], 'station_id': 1, 'distance': 1.0,
             'is_recycle_pickup': False, 'recycle_product_id': None},
        ]
        ok, missing = check_storage_satisfies(midlist, Counter({8: 1, 1: 1}))
        assert ok is True
        assert len(missing) == 0

    def test_missing_material(self):
        midlist = [
            {'materials': [8], 'station_id': 1, 'distance': 1.0,
             'is_recycle_pickup': False, 'recycle_product_id': None},
        ]
        ok, missing = check_storage_satisfies(midlist, Counter({8: 1, 1: 1}))
        assert ok is False
        assert missing[1] == 1

    def test_partially_satisfied_duplicates(self):
        midlist = [
            {'materials': [4], 'station_id': 1, 'distance': 1.0,
             'is_recycle_pickup': False, 'recycle_product_id': None},
        ]
        # Carrot needs {4:2, 2:1} — only one 4 available
        ok, missing = check_storage_satisfies(midlist, Counter({4: 2, 2: 1}))
        assert ok is False
        assert missing[4] == 1
        assert missing[2] == 1

    def test_empty_aidlist_always_satisfied(self):
        ok, missing = check_storage_satisfies([], Counter())
        assert ok is True


class TestBuildMid:
    def _make_entry(self, sid, materials, is_recycle=False, rpid=None):
        return {
            'station_id': sid,
            'materials': materials,
            'distance': 0.0,
            'is_recycle_pickup': is_recycle,
            'recycle_product_id': rpid,
        }

    def test_filters_to_needed_materials(self):
        midlist = [
            self._make_entry(1, [8, 3]),  # need 8, not 3
            self._make_entry(2, [1, 4]),  # need 1, not 4
        ]
        mid = build_mid(midlist, Counter({8: 1, 1: 1}))
        assert len(mid) == 2
        assert mid[0]['pickup_materials'] == [8]
        assert mid[1]['pickup_materials'] == [1]

    def test_skips_station_with_no_needed_materials(self):
        midlist = [
            self._make_entry(1, [3, 4]),  # none needed
            self._make_entry(2, [8, 1]),  # both needed
        ]
        mid = build_mid(midlist, Counter({8: 1, 1: 1}))
        assert len(mid) == 1
        assert mid[0]['station_id'] == 2

    def test_recycle_entries_pass_through_unchanged(self):
        midlist = [
            self._make_entry(10, [], is_recycle=True, rpid=81),
            self._make_entry(1, [8, 1]),
        ]
        mid = build_mid(midlist, Counter({8: 1, 1: 1}))
        assert mid[0]['is_recycle_pickup'] is True
        assert mid[0]['recycle_product_id'] == 81

    def test_stops_taking_once_satisfied(self):
        midlist = [
            self._make_entry(1, [8, 1]),  # satisfies everything
            self._make_entry(2, [8]),     # extra 8, not needed
        ]
        mid = build_mid(midlist, Counter({8: 1, 1: 1}))
        assert len(mid) == 1

    def test_handles_duplicate_material_need(self):
        # Carrot(442) needs {4:2, 2:1}; two stations each have one 4
        midlist = [
            self._make_entry(1, [4]),
            self._make_entry(2, [4, 2]),
        ]
        mid = build_mid(midlist, Counter({4: 2, 2: 1}))
        assert len(mid) == 2
        assert mid[0]['pickup_materials'] == [4]
        assert 4 in mid[1]['pickup_materials']
        assert 2 in mid[1]['pickup_materials']

    def test_empty_aidlist_returns_only_recycle_entries(self):
        midlist = [
            self._make_entry(10, [], is_recycle=True, rpid=81),
            self._make_entry(1, [8, 1]),
        ]
        mid = build_mid(midlist, Counter())
        # Only the recycle entry passes through; storage entry has nothing to pick
        assert len(mid) == 1
        assert mid[0]['is_recycle_pickup'] is True
