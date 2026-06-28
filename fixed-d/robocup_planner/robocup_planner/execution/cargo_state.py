"""
Tracks the physical state of cargo slots 2-6 (material storage).

Cargo geometry (per slot, cargo IDs 2-6):
  The tray is 2×6 studs wide.  Five placement positions are defined by
  where the centre of a brick sits along the 6-stud axis:

  Placement index | Block size | Centre stud column | Studs occupied
  ----------------+-----------+--------------------+---------------
       0          |   2×2     |        1           |  0-1
       1          |   2×4     |        2           |  0-3
       2          |   2×2     |        3           |  2-3
       3          |   2×4     |        4           |  2-5
       4          |   2×2     |        5           |  4-5

  A 2×2 block uses placements {0, 2, 4}.
  A 2×4 block uses placements {1, 3}.

  Overlapping stud columns prevent combining certain placements:
    placement 1 (cols 0-3) conflicts with placements 0 (cols 0-1) and 2 (cols 2-3).
    placement 3 (cols 2-5) conflicts with placements 2 (cols 2-3) and 4 (cols 4-5).

  The manipulator team value for a placed block is:  cargo_id * 10 + placement_index.
  Preferred placements per manipulator team spec: prefer higher indices (3, 4) first.

Cargo 1  — finished products (counted only, no slot tracking needed).
Cargo 7/8 — managed by CargoAllocator (planning module).
"""

from typing import Dict, List, Optional, Tuple

from robocup_planner.product_catalog import MATERIAL_SIZE

# Stud columns occupied by each placement index
_PLACEMENT_COLS: Dict[int, Tuple[int, int]] = {
    0: (0, 1),
    1: (0, 3),
    2: (2, 3),
    3: (2, 5),
    4: (4, 5),
}

# Valid placement indices per block size
_SIZE_PLACEMENTS: Dict[str, List[int]] = {
    '2x2': [4, 2, 0],   # prefer rightmost (manipulator spec)
    '2x4': [3, 1],      # prefer rightmost
}


def _cols_for(placement: int) -> set:
    lo, hi = _PLACEMENT_COLS[placement]
    return set(range(lo, hi + 1))


class CargoSlot:
    """One cargo tray (ID 2-6)."""

    def __init__(self, cargo_id: int):
        self.cargo_id = cargo_id
        # placement_index -> material_id  (None = empty)
        self._contents: Dict[int, Optional[int]] = {i: None for i in range(5)}

    def _occupied_cols(self) -> set:
        cols: set = set()
        for idx, mat in self._contents.items():
            if mat is not None:
                cols |= _cols_for(idx)
        return cols

    def find_placement(self, material_id: int) -> Optional[int]:
        """
        Find the lowest (preferred) free placement index for this material.
        Returns None if no space is available.
        """
        size = MATERIAL_SIZE.get(material_id, '2x2')
        occupied = self._occupied_cols()
        for idx in _SIZE_PLACEMENTS[size]:
            if not _cols_for(idx).intersection(occupied):
                return idx
        return None

    def place(self, material_id: int, placement_idx: int) -> int:
        """
        Place material at placement_idx.
        Returns the manipulator value: cargo_id * 10 + placement_idx.
        """
        self._contents[placement_idx] = material_id
        return self.cargo_id * 10 + placement_idx

    def remove(self, placement_idx: int) -> None:
        self._contents[placement_idx] = None

    def contents(self) -> List[Tuple[int, int]]:
        """List of (placement_idx, material_id) for occupied placements."""
        return [(idx, mat) for idx, mat in self._contents.items() if mat is not None]

    def is_empty(self) -> bool:
        return all(v is None for v in self._contents.values())

    def is_full(self) -> bool:
        """True if neither a 2×2 nor 2×4 block can be placed."""
        return self.find_placement(1) is None and self.find_placement(5) is None


class CargoManager:
    """
    Manages material cargo slots 2-6 and the finished-product count on cargo 1.
    Cargo 7/8 (in-transit assembly) are tracked by CargoAllocator.
    """

    MATERIAL_CARGO_IDS = list(range(2, 7))  # [2, 3, 4, 5, 6]

    def __init__(self):
        self._slots: Dict[int, CargoSlot] = {
            i: CargoSlot(i) for i in self.MATERIAL_CARGO_IDS
        }
        self.finished_on_cargo1: int = 0  # products sitting on cargo 1

    # ------------------------------------------------------------------
    # Placement
    # ------------------------------------------------------------------

    def place_material(self, material_id: int) -> Optional[int]:
        """
        Find the first available cargo slot (2-6) and place the material.
        Returns the manipulator value (cargo_id*10 + placement) or None if full.
        """
        for cargo_id in self.MATERIAL_CARGO_IDS:
            slot = self._slots[cargo_id]
            placement = slot.find_placement(material_id)
            if placement is not None:
                return slot.place(material_id, placement)
        return None

    def all_full(self) -> bool:
        """True if every material cargo slot is full."""
        return all(s.is_full() for s in self._slots.values())

    def remove_material(self, cargo_id: int, placement_idx: int) -> None:
        if cargo_id in self._slots:
            self._slots[cargo_id].remove(placement_idx)

    # ------------------------------------------------------------------
    # Content queries
    # ------------------------------------------------------------------

    def all_materials(self) -> List[Tuple[int, int, int]]:
        """All (cargo_id, placement_idx, material_id) currently stored."""
        result = []
        for cargo_id, slot in self._slots.items():
            for idx, mat in slot.contents():
                result.append((cargo_id, idx, mat))
        return result

    def find_materials_for_product(
        self, product_id: int
    ) -> Optional[List[Tuple[int, int, int]]]:
        """
        Check if all materials for product_id are present in cargo 2-6.
        Returns list of (cargo_id, placement_idx, material_id) covering
        exactly the required materials, or None if not all are available.
        """
        from collections import Counter
        from robocup_planner.product_catalog import get_material_count

        needed = get_material_count(product_id)
        available: List[Tuple[int, int, int]] = []

        remaining = Counter(needed)
        for cargo_id, placement_idx, mat_id in self.all_materials():
            if remaining.get(mat_id, 0) > 0:
                available.append((cargo_id, placement_idx, mat_id))
                remaining[mat_id] -= 1
                if remaining[mat_id] == 0:
                    del remaining[mat_id]

        return available if not remaining else None

    def can_assemble_for_workbench(
        self, workbench_product_ids: List[int]
    ) -> Optional[int]:
        """
        Return the first product_id from workbench_product_ids whose
        materials are all available in cargo 2-6, or None.
        """
        for pid in workbench_product_ids:
            if self.find_materials_for_product(pid) is not None:
                return pid
        return None

    # ------------------------------------------------------------------
    # Finished product tracking (cargo 1)
    # ------------------------------------------------------------------

    def add_finished_product(self) -> None:
        self.finished_on_cargo1 += 1

    def consume_finished_product(self) -> None:
        if self.finished_on_cargo1 > 0:
            self.finished_on_cargo1 -= 1
