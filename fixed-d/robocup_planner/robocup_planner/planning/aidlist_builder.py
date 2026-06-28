"""
Builds the aidlist (assembly id list) and net_aidlist.

aidlist     = all materials needed to produce every ordered product.
net_aidlist = aidlist minus materials that will be reclaimed by recycling.

Both are represented as Counter (frequency maps) so duplicate material
requirements are handled correctly.
"""

from collections import Counter
from typing import List

from robocup_planner.product_catalog import get_material_count


def build_aidlist(produce_product_ids: List[int]) -> Counter:
    """
    Sum up all material requirements across every product to be produced.
    Returns a Counter: {material_id: count_needed}.
    """
    aidlist: Counter = Counter()
    for pid in produce_product_ids:
        aidlist += get_material_count(pid)
    return aidlist


def build_recycled_materials(recycle_product_ids: List[int]) -> Counter:
    """
    Sum up all materials obtainable by disassembling every product to be recycled.
    Returns a Counter: {material_id: count_available_from_recycling}.
    """
    recycled: Counter = Counter()
    for pid in recycle_product_ids:
        recycled += get_material_count(pid)
    return recycled


def compute_net_aidlist(
    produce_product_ids: List[int],
    recycle_product_ids: List[int],
) -> tuple:
    """
    Compute both the full aidlist and the net_aidlist after subtracting
    materials recoverable through recycling.

    Returns (aidlist, net_aidlist, recycled_materials).
    net_aidlist contains only materials that must be fetched from storage.
    """
    aidlist = build_aidlist(produce_product_ids)
    recycled = build_recycled_materials(recycle_product_ids)

    net: Counter = Counter(aidlist)
    for mat_id, count in recycled.items():
        net[mat_id] = max(0, net[mat_id] - count)
        if net[mat_id] == 0:
            del net[mat_id]

    return aidlist, net, recycled
