"""Unit tests for cargo_allocator.py"""
import pytest
from robocup_planner.planning.cargo_allocator import CargoAllocator, INTRANSIT_CARGO_IDS


class TestAllocate:
    def test_single_eligible_product_gets_cargo7(self):
        alloc = CargoAllocator()
        result = alloc.allocate([81])  # E-Stop — eligible
        assert 81 in result
        assert result[81] == 7

    def test_two_eligible_products_use_both_slots(self):
        alloc = CargoAllocator()
        result = alloc.allocate([81, 34])
        assert set(result.values()) == {7, 8}

    def test_three_eligible_only_two_allocated(self):
        alloc = CargoAllocator()
        result = alloc.allocate([81, 34, 13])
        # Only 2 cargo slots available (7 and 8)
        assert len(result) == 2

    def test_workbench_only_not_allocated(self):
        alloc = CargoAllocator()
        result = alloc.allocate([8518])  # Big Tree — workbench only
        assert len(result) == 0

    def test_mixed_eligible_and_workbench(self):
        alloc = CargoAllocator()
        result = alloc.allocate([81, 8518])
        assert 81 in result
        assert 8518 not in result

    def test_priority_fewest_blocks_first(self):
        # E-Stop(81): 2 blocks, Carrot(442): 3 blocks
        # With only 1 slot available (we fill the other first),
        # E-Stop should get cargo 7 (fewest blocks → allocated first)
        alloc = CargoAllocator()
        result = alloc.allocate([442, 81])  # order shouldn't matter
        # E-Stop (2 blocks) should be cargo 7 as it's simpler
        assert result[81] == 7
        assert result[442] == 8

    def test_empty_list(self):
        alloc = CargoAllocator()
        result = alloc.allocate([])
        assert result == {}

    def test_all_workbench(self):
        alloc = CargoAllocator()
        result = alloc.allocate([8518, 46262, 48132])
        assert result == {}


class TestFindSlotForBlock:
    def test_finds_correct_slot_for_next_block(self):
        alloc = CargoAllocator()
        alloc.allocate([81])  # E-Stop: build order [8, 1]
        # First needed block is 8
        assert alloc.find_slot_for_block(8) == 7

    def test_does_not_find_wrong_block(self):
        alloc = CargoAllocator()
        alloc.allocate([81])  # needs 8 first, not 1
        assert alloc.find_slot_for_block(1) is None

    def test_advances_after_first_block_confirmed(self):
        alloc = CargoAllocator()
        alloc.allocate([81])  # build order [8, 1]
        alloc.confirm_placed(7, 8)  # place block 8
        # Now needs block 1
        assert alloc.find_slot_for_block(1) == 7
        assert alloc.find_slot_for_block(8) is None

    def test_two_slots_find_correct(self):
        alloc = CargoAllocator()
        # Both have 2 blocks; tie-break by product_id → Battery(34) < E-Stop(81)
        # Battery(34) → cargo7 [3,4], E-Stop(81) → cargo8 [8,1]
        alloc.allocate([81, 34])
        assert alloc.find_slot_for_block(3) == 7  # Battery needs 3 first on cargo 7
        assert alloc.find_slot_for_block(8) == 8  # E-Stop needs 8 first on cargo 8


class TestConfirmPlaced:
    def test_returns_false_until_complete(self):
        alloc = CargoAllocator()
        alloc.allocate([81])  # [8, 1]
        assert alloc.confirm_placed(7, 8) is False  # first block placed, not done

    def test_returns_true_when_complete(self):
        alloc = CargoAllocator()
        alloc.allocate([81])  # [8, 1]
        alloc.confirm_placed(7, 8)
        assert alloc.confirm_placed(7, 1) is True

    def test_three_block_product(self):
        alloc = CargoAllocator()
        alloc.allocate([442])  # Carrot [4, 4, 2]
        assert alloc.confirm_placed(7, 4) is False
        assert alloc.confirm_placed(7, 4) is False
        assert alloc.confirm_placed(7, 2) is True


class TestGetCompletedSlots:
    def test_no_completed_initially(self):
        alloc = CargoAllocator()
        alloc.allocate([81])
        assert alloc.get_completed_slots() == []

    def test_completed_after_all_blocks(self):
        alloc = CargoAllocator()
        alloc.allocate([81])  # [8, 1]
        alloc.confirm_placed(7, 8)
        alloc.confirm_placed(7, 1)
        slots = alloc.get_completed_slots()
        assert len(slots) == 1
        assert slots[0].cargo_id == 7
        assert slots[0].product_id == 81


class TestFreeSlot:
    def test_free_slot_clears_allocation(self):
        alloc = CargoAllocator()
        alloc.allocate([81])
        alloc.confirm_placed(7, 8)
        alloc.confirm_placed(7, 1)
        alloc.free_slot(7)
        assert alloc.get_completed_slots() == []
        assert not alloc.is_cargo_allocated(7)

    def test_free_slot_allows_reuse(self):
        alloc = CargoAllocator()
        alloc.allocate([81])
        alloc.confirm_placed(7, 8)
        alloc.confirm_placed(7, 1)
        alloc.free_slot(7)
        # Now cargo 7 is free; allocate another product
        result = alloc.allocate([34])
        assert 34 in result
        assert result[34] == 7
