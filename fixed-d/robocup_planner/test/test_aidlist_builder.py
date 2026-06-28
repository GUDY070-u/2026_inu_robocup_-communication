"""Unit tests for aidlist_builder.py"""
from collections import Counter
from robocup_planner.planning.aidlist_builder import (
    build_aidlist,
    build_recycled_materials,
    compute_net_aidlist,
)


class TestBuildAidlist:
    def test_single_product(self):
        # E-Stop (81): needs 8×1, 1×1
        assert build_aidlist([81]) == Counter({8: 1, 1: 1})

    def test_two_products_no_overlap(self):
        # E-Stop(81) + Battery(34): {8:1, 1:1} + {3:1, 4:1}
        result = build_aidlist([81, 34])
        assert result == Counter({8: 1, 1: 1, 3: 1, 4: 1})

    def test_two_products_with_overlap(self):
        # E-Stop(81): {8:1, 1:1}  Magnet(13): {1:1, 3:1}  → 1 appears twice
        result = build_aidlist([81, 13])
        assert result[1] == 2
        assert result[8] == 1
        assert result[3] == 1

    def test_duplicate_product(self):
        # Same product ordered twice → counts double
        result = build_aidlist([81, 81])
        assert result == Counter({8: 2, 1: 2})

    def test_empty_produce_list(self):
        assert build_aidlist([]) == Counter()

    def test_workbench_product_included(self):
        # Burger(48132): {4:1, 8:1, 1:1, 3:1, 2:1}
        result = build_aidlist([48132])
        assert result[4] == 1 and result[8] == 1


class TestBuildRecycledMaterials:
    def test_single_recycle(self):
        # Recycling E-Stop(81) returns {8:1, 1:1}
        assert build_recycled_materials([81]) == Counter({8: 1, 1: 1})

    def test_two_recycles(self):
        result = build_recycled_materials([81, 34])
        assert result == Counter({8: 1, 1: 1, 3: 1, 4: 1})

    def test_empty(self):
        assert build_recycled_materials([]) == Counter()


class TestComputeNetAidlist:
    def test_no_recycling(self):
        aidlist, net, recycled = compute_net_aidlist([81], [])
        assert aidlist == Counter({8: 1, 1: 1})
        assert net == Counter({8: 1, 1: 1})
        assert recycled == Counter()

    def test_full_coverage_by_recycling(self):
        # Produce E-Stop, recycle E-Stop → net should be empty
        aidlist, net, recycled = compute_net_aidlist([81], [81])
        assert net == Counter()
        assert aidlist == Counter({8: 1, 1: 1})
        assert recycled == Counter({8: 1, 1: 1})

    def test_partial_coverage(self):
        # Produce Carrot(442): needs {4:2, 2:1}
        # Recycle E-Stop(81): provides {8:1, 1:1} — no overlap
        # net stays {4:2, 2:1}
        aidlist, net, recycled = compute_net_aidlist([442], [81])
        assert net == Counter({4: 2, 2: 1})

    def test_recycling_covers_partial_duplicate(self):
        # Produce Carrot(442): needs {4:2, 2:1}
        # Recycle Battery(34): provides {3:1, 4:1} → net[4] = 2-1 = 1
        _, net, _ = compute_net_aidlist([442], [34])
        assert net[4] == 1
        assert net[2] == 1
        assert 3 not in net  # 3 was not needed

    def test_net_never_goes_negative(self):
        # Produce Battery(34): needs {3:1, 4:1}
        # Recycle Carrot(442): provides {4:2, 2:1} — 4 exceeds need
        _, net, _ = compute_net_aidlist([34], [442])
        assert net.get(4, 0) == 0  # key should be removed, not negative
        assert net[3] == 1

    def test_complex_scenario(self):
        # Produce: E-Stop(81) + Hammer(711)
        # E-Stop: {8:1, 1:1}  Hammer: {1:2, 7:1} → aidlist {1:3, 8:1, 7:1}
        # Recycle: Battery(34) → {3:1, 4:1} — no overlap
        aidlist, net, _ = compute_net_aidlist([81, 711], [34])
        assert aidlist[1] == 3
        assert net[1] == 3  # recycled nothing for mat 1
        assert 3 not in net
