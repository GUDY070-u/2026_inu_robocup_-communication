"""Unit tests for product_catalog.py"""
import pytest
from collections import Counter
from robocup_planner.product_catalog import (
    get_material_count,
    is_intransit_eligible,
    get_build_order,
    get_base_block,
    get_all_layers,
    product_name,
    validate_product_id,
    PRODUCTS,
)


class TestGetMaterialCount:
    def test_estop_81(self):
        # E-Stop: [8, 1]
        assert get_material_count(81) == Counter({8: 1, 1: 1})

    def test_battery_34(self):
        # Battery: [3, 4]
        assert get_material_count(34) == Counter({3: 1, 4: 1})

    def test_carrot_442_has_duplicate(self):
        # Carrot: [4, 4, 2] — two 4s
        assert get_material_count(442) == Counter({4: 2, 2: 1})

    def test_big_carrot_4482(self):
        # Big Carrot: [4, 4, 8, 2]
        assert get_material_count(4482) == Counter({4: 2, 8: 1, 2: 1})

    def test_hammer_711(self):
        # Hammer: [1, 1, 7] — two 1s
        assert get_material_count(711) == Counter({1: 2, 7: 1})

    def test_big_tree_8518_workbench(self):
        # Big Tree layers: [[8], [5, 1], [8]] → {8:2, 5:1, 1:1}
        assert get_material_count(8518) == Counter({8: 2, 5: 1, 1: 1})

    def test_ice_cream_46262_workbench(self):
        # layers: [[4], [6, 2], [6], [2]] → {4:1, 6:2, 2:2}
        assert get_material_count(46262) == Counter({4: 1, 6: 2, 2: 2})

    def test_burger_48132_workbench(self):
        # layers: [[4], [8], [1, 3], [2]] → {4:1, 8:1, 1:1, 3:1, 2:1}
        assert get_material_count(48132) == Counter({4: 1, 8: 1, 1: 1, 3: 1, 2: 1})

    def test_all_products_defined(self):
        for pid in PRODUCTS:
            count = get_material_count(pid)
            assert sum(count.values()) > 0, f"Product {pid} has no materials"


class TestIntransitEligibility:
    def test_single_column_products_eligible(self):
        eligible = [81, 34, 13, 442, 241, 462, 4482, 711]
        for pid in eligible:
            assert is_intransit_eligible(pid), f"{pid} should be in-transit eligible"

    def test_workbench_only_not_eligible(self):
        for pid in [8518, 46262, 48132]:
            assert not is_intransit_eligible(pid), f"{pid} should NOT be in-transit eligible"


class TestBuildOrder:
    def test_estop_order(self):
        assert get_build_order(81) == [8, 1]

    def test_carrot_order(self):
        assert get_build_order(442) == [4, 4, 2]

    def test_big_carrot_order(self):
        assert get_build_order(4482) == [4, 4, 8, 2]

    def test_hammer_order(self):
        assert get_build_order(711) == [1, 1, 7]

    def test_workbench_raises(self):
        with pytest.raises(ValueError):
            get_build_order(8518)

    def test_traffic_light_order(self):
        assert get_build_order(241) == [2, 4, 1]


class TestGetBaseBlock:
    def test_estop_base(self):
        assert get_base_block(81) == 8

    def test_battery_base(self):
        assert get_base_block(34) == 3

    def test_carrot_base(self):
        assert get_base_block(442) == 4

    def test_workbench_raises(self):
        with pytest.raises(ValueError):
            get_base_block(46262)


class TestMisc:
    def test_validate_product_id_valid(self):
        for pid in PRODUCTS:
            assert validate_product_id(pid)

    def test_validate_product_id_invalid(self):
        assert not validate_product_id(9999)

    def test_product_name_known(self):
        assert product_name(81) == 'E-Stop'
        assert product_name(4482) == 'Big Carrot'

    def test_product_name_unknown(self):
        assert 'Unknown' in product_name(9999)

    def test_total_11_products(self):
        assert len(PRODUCTS) == 11
