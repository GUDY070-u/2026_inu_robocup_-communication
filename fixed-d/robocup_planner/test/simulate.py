"""
Standalone simulation for the Executor - no ROS2 or hardware required.

Replaces all PlannerNode blocking calls with a MockNode that logs
every action and instantly returns success. Run with:

    cd src/robocup_planner
    python -m test.simulate

Scenarios:
  1. Single in-transit product (E-Stop: cargo 7 assembly)
  2. Two in-transit products filling cargo 7 and 8
  3. Workbench-only product
  4. Mixed: 1 in-transit (E-Stop) + 1 workbench (Burger) — full storage
  5. Recycling fully covers workbench product (no storage pickup needed)
  6. Partial recycling for workbench product; rest from storage
  7. Cargo overflow: many products force mid-trip workbench visit
"""

import sys
import threading
import textwrap
from collections import Counter
from typing import List, Optional

# Make sure the package is importable from src/robocup_planner/
import importlib, pathlib
pkg_root = pathlib.Path(__file__).resolve().parents[1]
if str(pkg_root) not in sys.path:
    sys.path.insert(0, str(pkg_root))

from robocup_planner.planning.aidlist_builder import compute_net_aidlist
from robocup_planner.planning.cargo_allocator import CargoAllocator
from robocup_planner.planning.midlist_builder import (
    build_storage_midlist,
    build_full_midlist,
    build_mid,
    check_storage_satisfies,
)
from robocup_planner.execution.executor import Executor, Plan


# ---------------------------------------------------------------------------
# Mock infrastructure
# ---------------------------------------------------------------------------

class MockLogger:
    def __init__(self, prefix=''):
        self._prefix = prefix

    def info(self, msg):
        print(f'  [LOG] {msg}')

    def warning(self, msg):
        print(f'  [WARN] {msg}')

    def error(self, msg):
        print(f'  [ERR] {msg}')


class MockNode:
    """
    Replaces PlannerNode in tests. Records every call and returns success.
    An optional wb_trigger_after_nav can be used to simulate the workbench
    sending a ready signal after a specific navigate() call.
    """

    def __init__(self, executor_ref_holder: list, wb_auto_signal_on_nav: Optional[int] = None):
        self._logger = MockLogger()
        self._calls: List[str] = []
        self._executor_holder = executor_ref_holder   # list so we can inject after construction
        self._wb_auto_signal_on_nav = wb_auto_signal_on_nav
        self._nav_count = 0

    def get_logger(self):
        return self._logger

    def navigate(self, station_id: int) -> bool:
        self._nav_count += 1
        self._calls.append(f'navigate({station_id})')
        print(f'    >> navigate → station {station_id}')
        # Auto-fire workbench signal to simulate wb team publishing ready
        if self._wb_auto_signal_on_nav and self._nav_count == self._wb_auto_signal_on_nav:
            ex = self._executor_holder[0]
            if ex is not None:
                print(f'    ~~ [SIM] workbench ready signal fired after nav #{self._nav_count}')
                ex.wb_signal.set()
        return True

    def arm_pick_material(self, station_id: int, material_id: int, manipulator_slot: int) -> bool:
        self._calls.append(f'arm_pick_material(mat={material_id}, slot={manipulator_slot})')
        print(f'    >> arm_pick_material: mat={material_id} → slot={manipulator_slot}')
        return True

    def arm_pick_product(self, station_id: int, product_id: int) -> bool:
        self._calls.append(f'arm_pick_product(pid={product_id})')
        print(f'    >> arm_pick_product: pid={product_id}')
        return True

    def arm_unload_material(self, cargo_id: int, placement_idx: int) -> bool:
        self._calls.append(f'arm_unload_material(cargo={cargo_id}, idx={placement_idx})')
        print(f'    >> arm_unload_material: cargo {cargo_id}[{placement_idx}]')
        return True

    def arm_deliver(self, from_cargo_id: int) -> bool:
        self._calls.append(f'arm_deliver(from_cargo={from_cargo_id})')
        print(f'    >> arm_deliver: from cargo {from_cargo_id}')
        return True

    def wb_task(self, work_type: str, product_id: int) -> bool:
        self._calls.append(f'wb_task({work_type}, pid={product_id})')
        print(f'    >> wb_task: {work_type} product {product_id}')
        return True

    @property
    def calls(self) -> List[str]:
        return list(self._calls)


# ---------------------------------------------------------------------------
# Storage / arena fixtures
# ---------------------------------------------------------------------------

def make_storage(spec: dict) -> List[dict]:
    """spec = {station_id: [material_id, ...]}"""
    return [{'station_id': sid, 'material_ids': mats} for sid, mats in spec.items()]


def make_calc_mock(positions: dict):
    """Simple distance calculator backed by a static position map."""
    import math
    from unittest.mock import MagicMock

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


# Shared arena layout for most scenarios
POSITIONS = {
    0:  (0.0,  0.0),   # home
    10: (0.5,  0.0),   # workbench
    20: (5.0,  0.0),   # customer counter
    1:  (1.0,  0.0),   # storage A
    2:  (2.0,  0.0),   # storage B
    3:  (3.0,  0.0),   # storage C
    4:  (4.0,  0.0),   # storage D
}


def build_plan(produce_ids, recycle_ids, storage_spec,
               workbench_id=10, customer_id=20, home_id=0,
               wb_auto_signal_nav=None):
    calc = make_calc_mock(POSITIONS)
    storage = make_storage(storage_spec)

    aidlist, net_aidlist, recycled = compute_net_aidlist(produce_ids, recycle_ids)

    storage_mid = build_storage_midlist(storage, calc, home_id)
    needs_recycling = bool(recycle_ids)

    recycle_orders = [{'station_id': customer_id, 'product_id': pid} for pid in recycle_ids]

    full_midlist = build_full_midlist(
        storage_stations=storage,
        customer_stations=[],
        recycle_orders=recycle_orders,
        calc=calc,
        home_station_id=home_id,
        workbench_station_id=workbench_id,
        needs_recycling=needs_recycling,
    )
    mid = build_mid(full_midlist, net_aidlist)

    temp_alloc = CargoAllocator()
    intransit_allocated = temp_alloc.allocate(produce_ids)
    intransit_ids = list(intransit_allocated.keys())
    workbench_ids = [pid for pid in produce_ids if pid not in intransit_ids]

    plan = Plan(
        mid=mid,
        workbench_products=workbench_ids,
        intransit_products=intransit_ids,
        workbench_station_id=workbench_id,
        customer_station_id=customer_id,
        home_station_id=home_id,
    )
    return plan


def run_scenario(title: str, plan: Plan, wb_auto_signal_nav: Optional[int] = None):
    print()
    print('=' * 70)
    print(f'SCENARIO: {title}')
    print('=' * 70)
    print(f'  mid entries     : {len(plan.mid)}')
    print(f'  workbench prods : {plan.workbench_products}')
    print(f'  in-transit prods: {plan.intransit_products}')
    print()

    executor_holder = [None]
    node = MockNode(executor_holder, wb_auto_signal_on_nav=wb_auto_signal_nav)
    executor = Executor(plan, node)
    executor_holder[0] = executor

    executor.run()

    print()
    print(f'  Total actions: {len(node.calls)}')
    print('  Action sequence:')
    for i, c in enumerate(node.calls, 1):
        print(f'    {i:2d}. {c}')

    # Basic sanity checks
    navigates = [c for c in node.calls if c.startswith('navigate')]
    delivers  = [c for c in node.calls if c.startswith('arm_deliver')]
    picks     = [c for c in node.calls if c.startswith('arm_pick_material')]

    print()
    print(f'  navigate calls : {len(navigates)}')
    print(f'  arm_deliver    : {len(delivers)}')
    print(f'  arm_pick_mat   : {len(picks)}')

    expected_delivers = len(plan.workbench_products) + len(plan.intransit_products)
    status = 'PASS' if len(delivers) == expected_delivers else 'FAIL'
    print(f'  delivery check : {status} (expected {expected_delivers}, got {len(delivers)})')
    return status


# ---------------------------------------------------------------------------
# Scenarios
# ---------------------------------------------------------------------------

def scenario_1():
    """Single in-transit product (E-Stop). Assembled on cargo 7, delivered direct."""
    plan = build_plan(
        produce_ids=[81],           # E-Stop: [8, 1]
        recycle_ids=[],
        storage_spec={1: [8, 1]},   # both materials at station 1
    )
    return run_scenario('1 - Single in-transit (E-Stop)', plan)


def scenario_2():
    """Two in-transit products filling both cargo 7 and 8."""
    plan = build_plan(
        produce_ids=[81, 34],       # E-Stop + Battery
        recycle_ids=[],
        storage_spec={
            1: [8, 3],
            2: [1, 4],
        },
    )
    return run_scenario('2 - Two in-transit (E-Stop + Battery)', plan)


def scenario_3():
    """Workbench-only product (Big Tree). All materials go via workbench."""
    # wb_signal fired after 2nd navigate call to simulate WB publishing ready
    plan = build_plan(
        produce_ids=[8518],         # Big Tree: layers [[8],[5,1],[8]]
        recycle_ids=[],
        storage_spec={
            1: [8, 8],
            2: [5, 1],
        },
    )
    return run_scenario('3 - Workbench-only (Big Tree)', plan, wb_auto_signal_nav=3)


def scenario_4():
    """Mixed: 1 in-transit (E-Stop) + 1 workbench (Burger).

    E-Stop needs {8:1, 1:1}; Burger needs {4:1, 8:1, 1:1, 3:1, 2:1}.
    Combined: {8:2, 1:2, 4:1, 3:1, 2:1} — storage must supply both.
    """
    plan = build_plan(
        produce_ids=[81, 48132],    # E-Stop + Burger
        recycle_ids=[],
        storage_spec={
            1: [8, 8, 4],   # two 8s (one for E-Stop, one for Burger) + one 4
            2: [1, 1, 2],   # two 1s (one for E-Stop, one for Burger) + one 2
            3: [3],
        },
    )
    return run_scenario('4 - Mixed (E-Stop in-transit + Burger workbench)', plan)


def scenario_5():
    """Recycling fully covers a workbench-only product → no storage pickup.

    Big Tree needs {8:2, 5:1, 1:1}.  Recycling a Big Tree returns identical
    materials, so net_aidlist is empty.  The executor must still run Phase 1
    (disassembly) and then immediately assemble and deliver.
    """
    plan = build_plan(
        produce_ids=[8518],         # Big Tree (workbench-only)
        recycle_ids=[8518],         # recycle Big Tree → returns {8:2, 5:1, 1:1}
        storage_spec={},            # no storage needed
    )
    return run_scenario('5 - Recycling covers everything (Big Tree → Big Tree)', plan)


def scenario_6():
    """Partial recycling for a workbench-only product.

    Big Tree needs {8:2, 5:1, 1:1}.
    Recycle E-Stop (81) → returns {8:1, 1:1} → net_aidlist = {8:1, 5:1}.
    Storage provides the remaining {8:1, 5:1}.
    """
    plan = build_plan(
        produce_ids=[8518],         # Big Tree (workbench-only)
        recycle_ids=[81],           # recycle E-Stop → gives {8:1, 1:1}
        storage_spec={1: [8, 5]},   # provides remaining {8:1, 5:1}
    )
    return run_scenario('6 - Partial recycling (Big Tree + recycle E-Stop)', plan)


def scenario_7():
    """Three products: cargo 7+8 full (in-transit), third goes to workbench."""
    # E-Stop(81) → cargo7, Battery(34) → cargo8, Magnet(13) → workbench
    plan = build_plan(
        produce_ids=[81, 34, 13],
        recycle_ids=[],
        storage_spec={
            1: [8, 3],
            2: [1, 4],
            3: [1, 3],
        },
    )
    return run_scenario('7 - Overflow to workbench (3 products, 2 in-transit slots)', plan, wb_auto_signal_nav=4)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

if __name__ == '__main__':
    results = []
    results.append(('1', scenario_1()))
    results.append(('2', scenario_2()))
    results.append(('3', scenario_3()))
    results.append(('4', scenario_4()))
    results.append(('5', scenario_5()))
    results.append(('6', scenario_6()))
    results.append(('7', scenario_7()))

    print()
    print('=' * 70)
    print('SUMMARY')
    print('=' * 70)
    for num, status in results:
        mark = 'OK' if status == 'PASS' else 'FAIL'
        print(f'  {mark} Scenario {num}: {status}')

    failed = sum(1 for _, s in results if s != 'PASS')
    print()
    print(f'  {len(results) - failed}/{len(results)} scenarios passed')
    sys.exit(0 if failed == 0 else 1)
