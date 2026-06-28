"""
Cargo slot manager for in-transit assembly (cargo IDs 7 and 8).

Only products with no side-by-side layers are eligible (is_intransit_eligible).
Priority rule when more than 2 products qualify:
  1. Fewest blocks first — simpler assemblies finish sooner, freeing the slot.
  2. Smallest product_id as deterministic tie-break.

Slot lifecycle:
  allocate()         → assign products to cargo 7/8 at plan time.
  find_slot_for_block() → query which cargo slot is waiting for a given block.
  confirm_placed()   → record that a block was successfully placed; returns
                        True when the assembly on that cargo is complete.
  free_slot()        → called after product is delivered; releases the slot.
"""

from typing import Dict, List, Optional

from robocup_planner.product_catalog import (
    get_build_order,
    is_intransit_eligible,
)

INTRANSIT_CARGO_IDS = (7, 8)


class IntransitSlot:
    def __init__(self, cargo_id: int, product_id: int):
        self.cargo_id = cargo_id
        self.product_id = product_id
        self.build_order: List[int] = get_build_order(product_id)
        self.placed: List[int] = []

    @property
    def next_needed(self) -> Optional[int]:
        idx = len(self.placed)
        if idx < len(self.build_order):
            return self.build_order[idx]
        return None

    @property
    def is_complete(self) -> bool:
        return self.placed == self.build_order

    def confirm_block(self, material_id: int) -> bool:
        """Record block as placed. Returns True if assembly is now complete."""
        self.placed.append(material_id)
        return self.is_complete


class CargoAllocator:
    def __init__(self):
        self._slots: Dict[int, Optional[IntransitSlot]] = {
            cargo_id: None for cargo_id in INTRANSIT_CARGO_IDS
        }

    def allocate(self, produce_product_ids: List[int]) -> Dict[int, int]:
        """
        Assign eligible products to cargo 7/8.
        Returns {product_id: cargo_id} for every allocated product.
        Products that do not get a slot go to the workbench path.
        """
        eligible = sorted(
            [pid for pid in produce_product_ids if is_intransit_eligible(pid)],
            key=lambda pid: (len(get_build_order(pid)), pid),
        )

        allocation: Dict[int, int] = {}
        for pid in eligible:
            for cargo_id in INTRANSIT_CARGO_IDS:
                if self._slots[cargo_id] is None:
                    self._slots[cargo_id] = IntransitSlot(cargo_id, pid)
                    allocation[pid] = cargo_id
                    break

        return allocation

    def find_slot_for_block(self, material_id: int) -> Optional[int]:
        """
        Return the cargo_id of a slot that is currently waiting for
        material_id as its next block, or None if no slot is waiting.
        """
        for cargo_id, slot in self._slots.items():
            if slot is not None and slot.next_needed == material_id:
                return cargo_id
        return None

    def confirm_placed(self, cargo_id: int, material_id: int) -> bool:
        """
        Record that material_id was placed on cargo_id.
        Returns True if the in-transit assembly is now complete.
        """
        slot = self._slots.get(cargo_id)
        if slot is None:
            return False
        return slot.confirm_block(material_id)

    def get_completed_slots(self) -> List[IntransitSlot]:
        """Return all slots whose assembly is complete (product ready to deliver)."""
        return [s for s in self._slots.values() if s is not None and s.is_complete]

    def free_slot(self, cargo_id: int) -> None:
        """Release a cargo slot after the product has been delivered."""
        self._slots[cargo_id] = None

    def allocated_products(self) -> List[int]:
        return [s.product_id for s in self._slots.values() if s is not None]

    def is_cargo_allocated(self, cargo_id: int) -> bool:
        return self._slots.get(cargo_id) is not None
