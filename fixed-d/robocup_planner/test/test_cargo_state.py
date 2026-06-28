"""Unit tests for cargo_state.py"""
import pytest
from collections import Counter
from robocup_planner.execution.cargo_state import CargoManager, CargoSlot


class TestCargoSlotPlacement:
    def test_2x2_gets_valid_placement(self):
        slot = CargoSlot(cargo_id=2)
        # material_id=1 is 2x2 → placements {0,2,4}
        idx = slot.find_placement(1)
        assert idx in (0, 2, 4)

    def test_2x4_gets_valid_placement(self):
        slot = CargoSlot(cargo_id=2)
        # material_id=5 is 2x4 → placements {1,3}
        idx = slot.find_placement(5)
        assert idx in (1, 3)

    def test_2x4_blocks_2x2_overlapping_cols(self):
        slot = CargoSlot(cargo_id=2)
        slot.place(5, 1)  # 2x4 at placement 1 → occupies cols 0-3
        # 2x2 at placement 0 (cols 0-1) conflicts → not available
        idx = slot.find_placement(1)
        # Only placement 4 (cols 4-5) should be free
        assert idx == 4

    def test_2x2_blocks_2x4_overlapping_cols(self):
        slot = CargoSlot(cargo_id=2)
        slot.place(2, 2)  # 2x2 at placement 2 → occupies cols 2-3
        # 2x4 at placement 3 (cols 2-5) conflicts → try placement 1 (cols 0-3) also conflicts
        idx = slot.find_placement(5)
        assert idx is None  # both 2x4 placements blocked

    def test_place_returns_manipulator_value(self):
        slot = CargoSlot(cargo_id=3)
        val = slot.place(1, 4)
        assert val == 34  # 3 * 10 + 4

    def test_remove_frees_placement(self):
        slot = CargoSlot(cargo_id=2)
        slot.place(5, 3)  # 2x4 at placement 3
        slot.remove(3)
        # After removal, placement 3 should be free again
        assert slot.find_placement(5) == 3

    def test_empty_slot_is_not_full(self):
        slot = CargoSlot(cargo_id=2)
        assert not slot.is_full()
        assert slot.is_empty()

    def test_full_slot_detection(self):
        slot = CargoSlot(cargo_id=2)
        # Fill with: 2x4 at 1 (cols 0-3), 2x2 at 4 (cols 4-5)
        slot.place(5, 1)
        slot.place(1, 4)
        assert slot.is_full()

    def test_prefer_rightmost_2x2(self):
        # Preference order for 2x2 is [4, 2, 0] — rightmost first
        slot = CargoSlot(cargo_id=2)
        idx = slot.find_placement(1)
        assert idx == 4

    def test_prefer_rightmost_2x4(self):
        # Preference order for 2x4 is [3, 1] — rightmost first
        slot = CargoSlot(cargo_id=2)
        idx = slot.find_placement(5)
        assert idx == 3


class TestCargoManager:
    def test_place_material_returns_value(self):
        cm = CargoManager()
        val = cm.place_material(1)  # 2x2
        # Should go to cargo 2, placement 4 (rightmost) → 2*10+4 = 24
        assert val == 24

    def test_place_fills_first_available_cargo(self):
        cm = CargoManager()
        val = cm.place_material(5)  # 2x4 → cargo 2, placement 3 → 23
        assert val == 23

    def test_returns_none_when_full(self):
        cm = CargoManager()
        # Fill all 5 cargo slots (2-6) completely
        # Each slot fits: one 2x4@3 + one 2x4@1 is actually checked via is_full
        # Easier: fill each slot with materials until full
        for _ in range(5):
            # Fill a cargo slot: 2x4@3 (cols 2-5) + 2x4@1 (cols 0-3)? No, they overlap
            # Each cargo slot can hold: e.g. 2x4@1(cols0-3) + 2x2@4(cols4-5) = 2 items
            cm.place_material(5)  # 2x4 → placement 3
            cm.place_material(1)  # 2x2 → placement 4 (only free spot after 2x4@3)
        # All 5 cargo slots are full now
        result = cm.place_material(1)
        assert result is None

    def test_all_materials_lists_contents(self):
        cm = CargoManager()
        cm.place_material(1)  # mat 1 on cargo 2
        cm.place_material(2)  # mat 2 on cargo 2
        contents = cm.all_materials()
        mat_ids = [m for _, _, m in contents]
        assert 1 in mat_ids
        assert 2 in mat_ids

    def test_remove_material(self):
        cm = CargoManager()
        val = cm.place_material(5)  # 2x4 → cargo2, placement3, val=23
        cm.remove_material(2, 3)
        # After removal, placing 2x4 again should succeed on cargo 2
        val2 = cm.place_material(5)
        assert val2 == 23  # back to placement 3

    def test_find_materials_for_product_success(self):
        cm = CargoManager()
        # E-Stop needs {8:1, 1:1}
        cm.place_material(8)  # 2x4
        cm.place_material(1)  # 2x2
        result = cm.find_materials_for_product(81)
        assert result is not None
        mat_ids = {m for _, _, m in result}
        assert 8 in mat_ids
        assert 1 in mat_ids

    def test_find_materials_for_product_missing(self):
        cm = CargoManager()
        cm.place_material(8)  # only 8, missing 1
        result = cm.find_materials_for_product(81)
        assert result is None

    def test_find_materials_for_product_with_duplicate(self):
        # Hammer(711): needs {1:2, 7:1}
        cm = CargoManager()
        cm.place_material(1)  # first 1
        cm.place_material(1)  # second 1
        cm.place_material(7)  # 7 (2x4)
        result = cm.find_materials_for_product(711)
        assert result is not None
        from collections import Counter
        mat_count = Counter(m for _, _, m in result)
        assert mat_count[1] == 2
        assert mat_count[7] == 1

    def test_can_assemble_for_workbench(self):
        cm = CargoManager()
        cm.place_material(8)
        cm.place_material(1)
        result = cm.can_assemble_for_workbench([81])
        assert result == 81

    def test_can_assemble_returns_none_when_missing(self):
        cm = CargoManager()
        cm.place_material(8)
        result = cm.can_assemble_for_workbench([81])
        assert result is None

    def test_finished_product_tracking(self):
        cm = CargoManager()
        assert cm.finished_on_cargo1 == 0
        cm.add_finished_product()
        cm.add_finished_product()
        assert cm.finished_on_cargo1 == 2
        cm.consume_finished_product()
        assert cm.finished_on_cargo1 == 1
