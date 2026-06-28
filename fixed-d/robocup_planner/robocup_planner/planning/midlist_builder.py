"""
Builds the midlist and the final mid (movement id list).

midlist — list-in-list of storage stations sorted by distance.
          Each element represents one station and its available materials.
          When recycling is needed, customer-counter entries are prepended
          as Phase 1, and storage distances are recalculated from the
          workbench position for Phase 2.

mid     — midlist filtered to only the materials in net_aidlist,
          preserving distance order. This is the ordered pickup sequence
          the executor follows.
"""

from collections import Counter
from typing import Dict, List, Optional, Tuple

from robocup_planner.planning.distance_calculator import DistanceCalculator


# Each midlist entry (one station visit):
# {
#   'station_id':        int,
#   'materials':         [int, ...],   # all available at this station
#   'distance':          float,
#   'is_recycle_pickup': bool,         # True for customer-counter visits in Phase 1
#   'recycle_product_id': int | None,  # which product to pick up (Phase 1 only)
# }


def build_storage_midlist(
    storage_stations: List[Dict],
    calc: DistanceCalculator,
    ref_station_id: int,
) -> List[Dict]:
    """
    Sort storage stations by Euclidean distance from ref_station_id.
    ref_station_id is 0 (home) normally, or the workbench ID when recycling.
    """
    ref_pos = calc.get_position(ref_station_id) or (0.0, 0.0)
    entries = []
    for st in storage_stations:
        sid = st['station_id']
        dist = calc.point_to_station(ref_pos[0], ref_pos[1], sid)
        entries.append({
            'station_id': sid,
            'materials': list(st['material_ids']),
            'distance': dist,
            'is_recycle_pickup': False,
            'recycle_product_id': None,
        })
    entries.sort(key=lambda e: e['distance'])
    return entries


def build_recycle_phase_entries(
    customer_stations: List[Dict],
    recycle_orders: List[Dict],
    calc: DistanceCalculator,
    ref_station_id: int,
) -> List[Dict]:
    """
    Build Phase 1 entries: visits to customer counters to collect products
    that need to be recycled. Sorted by distance from ref_station_id (home).

    recycle_orders: list of {'station_id': int, 'product_id': int}
                    mapping each recycled product to its customer counter.
    """
    ref_pos = calc.get_position(ref_station_id) or (0.0, 0.0)
    entries = []
    for order in recycle_orders:
        sid = order['station_id']
        dist = calc.point_to_station(ref_pos[0], ref_pos[1], sid)
        entries.append({
            'station_id': sid,
            'materials': [],
            'distance': dist,
            'is_recycle_pickup': True,
            'recycle_product_id': order['product_id'],
        })
    entries.sort(key=lambda e: e['distance'])
    return entries


def build_full_midlist(
    storage_stations: List[Dict],
    customer_stations: List[Dict],
    recycle_orders: List[Dict],
    calc: DistanceCalculator,
    home_station_id: int,
    workbench_station_id: int,
    needs_recycling: bool,
) -> List[Dict]:
    """
    Assemble the complete midlist:
      Phase 1 (if recycling needed): customer counter visits, sorted from home.
      Phase 2: storage pickups, sorted from workbench if recycling, else from home.

    Returns the concatenated list.  Phase 1 entries are marked is_recycle_pickup=True.
    """
    if needs_recycling:
        phase1 = build_recycle_phase_entries(
            customer_stations, recycle_orders, calc, home_station_id
        )
        phase2_ref = workbench_station_id
    else:
        phase1 = []
        phase2_ref = home_station_id

    phase2 = build_storage_midlist(storage_stations, calc, phase2_ref)
    return phase1 + phase2


def check_storage_satisfies(
    storage_midlist: List[Dict],
    net_aidlist: Counter,
) -> Tuple[bool, Counter]:
    """
    Check whether the materials in storage can satisfy net_aidlist.
    Returns (satisfied, missing_materials).
    """
    available: Counter = Counter()
    for entry in storage_midlist:
        for mat in entry['materials']:
            available[mat] += 1

    missing: Counter = Counter()
    for mat_id, needed in net_aidlist.items():
        shortfall = needed - available.get(mat_id, 0)
        if shortfall > 0:
            missing[mat_id] = shortfall

    return (len(missing) == 0, missing)


def build_mid(
    midlist: List[Dict],
    net_aidlist: Counter,
) -> List[Dict]:
    """
    Filter midlist to only the materials needed by net_aidlist, preserving
    distance order.  Recycle-pickup entries (Phase 1) pass through unchanged.

    Each returned entry:
    {
      'station_id':         int,
      'pickup_materials':   [int, ...],
      'is_recycle_pickup':  bool,
      'recycle_product_id': int | None,
    }
    """
    remaining = Counter(net_aidlist)
    mid: List[Dict] = []

    for entry in midlist:
        if entry['is_recycle_pickup']:
            mid.append({
                'station_id': entry['station_id'],
                'pickup_materials': [],
                'is_recycle_pickup': True,
                'recycle_product_id': entry['recycle_product_id'],
            })
            continue

        pickup: List[int] = []
        for mat in entry['materials']:
            if remaining.get(mat, 0) > 0:
                pickup.append(mat)
                remaining[mat] -= 1
                if remaining[mat] == 0:
                    del remaining[mat]

        if pickup:
            mid.append({
                'station_id': entry['station_id'],
                'pickup_materials': pickup,
                'is_recycle_pickup': False,
                'recycle_product_id': None,
            })

    return mid
