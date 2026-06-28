"""Plan D AMR internal inventory model.

This module is intentionally ROS-message independent.  The planner keeps rich
metadata here and converts it to Step.object_ids / Step.slide_ids only at the
last step-generation boundary.

Current physical slot convention:
    slot 0   : product / recycle-product default slot
    slot 1-5 : raw material slides
    slot 6-7 : AMR assembly slots / auxiliary recycle-product preload slots
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional, Tuple


# ---------------------------------------------------------------------------
# Slot constants
# ---------------------------------------------------------------------------

PRODUCT_SLOT = 0
RAW_SLIDE_SLOTS = (1, 2, 3, 4, 5)
ASSEMBLY_SLOTS = (6, 7)
RAW_SLIDE_CAPACITY_UNITS = 3


# ---------------------------------------------------------------------------
# Item roles
# ---------------------------------------------------------------------------

ROLE_PRODUCE_WB_RAW = "produce_wb_raw"
ROLE_PRODUCE_AMR_BASE = "produce_amr_base"
ROLE_PRODUCE_AMR_UPPER = "produce_amr_upper"
ROLE_PRODUCE_AMR_PRODUCT = "produce_amr_product"
ROLE_WB_PRODUCT = "wb_product"

ROLE_RECYCLE_PRODUCT = "recycle_product"
ROLE_RECYCLE_PRODUCT_PRELOAD = "recycle_product_preload"
ROLE_RECYCLED_RAW = "recycled_raw"
ROLE_RETURN_RAW = "return_raw"
ROLE_REUSE_RAW = "reuse_raw"


@dataclass
class InventoryItem:
    """One physical/logical object carried or handled by the AMR.

    `object_id` is the ID sent to the arm.
    `slide_id` is the compact ID sent over Step.slide_ids.

    The extra fields are planner-only metadata.  They let the planner reason
    about ownership, purpose, source/target station and physical slot without
    overloading Step.object_ids / Step.slide_ids.
    """

    object_id: int
    slot_index: int
    slide_id: int
    role: str

    uid: str = ""
    object_kind: str = "raw"  # raw, product, batch

    order_index: Optional[int] = None
    order_type: Optional[str] = None  # produce, recycle
    product_id: Optional[int] = None

    source_station: Optional[int] = None
    target_station: Optional[int] = None

    size: int = 1
    material_index: Optional[int] = None
    note: str = ""

    def __post_init__(self):
        self.object_id = int(self.object_id)
        self.slot_index = int(self.slot_index)
        self.slide_id = int(self.slide_id)
        self.size = int(self.size)
        if self.order_index is not None:
            self.order_index = int(self.order_index)
        if self.product_id is not None:
            self.product_id = int(self.product_id)
        if self.source_station is not None:
            self.source_station = int(self.source_station)
        if self.target_station is not None:
            self.target_station = int(self.target_station)
        if self.material_index is not None:
            self.material_index = int(self.material_index)
        if not self.uid:
            oi = "none" if self.order_index is None else str(self.order_index)
            pid = "none" if self.product_id is None else str(self.product_id)
            ss = "none" if self.source_station is None else str(self.source_station)
            ts = "none" if self.target_station is None else str(self.target_station)
            mi = "none" if self.material_index is None else str(self.material_index)
            self.uid = f"{self.role}:o{oi}:p{pid}:obj{self.object_id}:s{self.slot_index}:src{ss}:dst{ts}:m{mi}"

    @property
    def physical_slot(self) -> int:
        return int(self.slot_index)

    def short(self) -> str:
        order = "-" if self.order_index is None else str(self.order_index)
        product = "-" if self.product_id is None else str(self.product_id)
        src = "-" if self.source_station is None else str(self.source_station)
        dst = "-" if self.target_station is None else str(self.target_station)
        return (
            f"obj={self.object_id},slide={self.slide_id},slot={self.slot_index},"
            f"role={self.role},order={order},product={product},src={src},dst={dst}"
        )


@dataclass
class SlotState:
    slot_index: int
    slot_type: str
    capacity: int
    used_size: int = 0
    items: List[InventoryItem] = field(default_factory=list)

    def can_place(self, item: InventoryItem) -> bool:
        if int(item.slot_index) != int(self.slot_index):
            return False
        if self.slot_type == "raw_slide":
            return self.used_size + int(item.size) <= int(self.capacity)
        # product / assembly slots hold one product/base by default.
        return not self.items

    def place(self, item: InventoryItem):
        if not self.can_place(item):
            raise RuntimeError(
                f"slot {self.slot_index} cannot accept item {item.short()} | "
                f"used={self.used_size}/{self.capacity}, items={[x.short() for x in self.items]}"
            )
        self.items.append(item)
        self.used_size += int(item.size)

    def remove_uid(self, uid: str) -> Optional[InventoryItem]:
        for idx, item in enumerate(self.items):
            if item.uid == uid:
                removed = self.items.pop(idx)
                self.used_size = max(0, self.used_size - int(removed.size))
                return removed
        return None


class AmrInventory:
    """Small helper for planner-side AMR slot bookkeeping.

    It is deliberately permissive enough for planning-time use.  The Step message
    remains compact, but this object can be used to validate obvious physical slot
    conflicts and to print detailed debug information.
    """

    def __init__(self):
        self.slots: Dict[int, SlotState] = {
            PRODUCT_SLOT: SlotState(PRODUCT_SLOT, "product", 1),
            **{s: SlotState(s, "raw_slide", RAW_SLIDE_CAPACITY_UNITS) for s in RAW_SLIDE_SLOTS},
            **{s: SlotState(s, "assembly", 1) for s in ASSEMBLY_SLOTS},
        }

    def can_place(self, item: InventoryItem) -> bool:
        slot = self.slots.get(int(item.slot_index))
        return slot is not None and slot.can_place(item)

    def place(self, item: InventoryItem):
        slot = self.slots.get(int(item.slot_index))
        if slot is None:
            raise RuntimeError(f"unknown AMR slot {item.slot_index} for {item.short()}")
        slot.place(item)

    def remove(self, item: InventoryItem) -> Optional[InventoryItem]:
        slot = self.slots.get(int(item.slot_index))
        if slot is None:
            return None
        return slot.remove_uid(item.uid)

    def items_for_order(self, order_index: int) -> List[InventoryItem]:
        return [item for slot in self.slots.values() for item in slot.items if item.order_index == int(order_index)]

    def items_for_return_station(self, target_station: int) -> List[InventoryItem]:
        return [item for slot in self.slots.values() for item in slot.items if item.target_station == int(target_station)]

    def dump(self) -> List[str]:
        rows = []
        for slot_id in sorted(self.slots):
            slot = self.slots[slot_id]
            if not slot.items:
                continue
            rows.append(
                f"slot={slot_id} type={slot.slot_type} used={slot.used_size}/{slot.capacity} | "
                + "; ".join(item.short() for item in slot.items)
            )
        return rows


def command_from_items(items: Iterable[InventoryItem]) -> Tuple[List[int], List[int]]:
    item_list = list(items)
    return [int(item.object_id) for item in item_list], [int(item.slide_id) for item in item_list]


def describe_items(items: Iterable[InventoryItem]) -> str:
    return "[" + " | ".join(item.short() for item in items) + "]"
