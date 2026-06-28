"""
Hybrid executor: follows a static mid list while reacting to workbench
signals and in-transit assembly completions at runtime.

Workbench interrupt rule (M4):
  - AMR in transit      → divert after completing the NEXT station visit.
  - AMR at a station    → divert after the current load/unload finishes.
  - AMR already en route to workbench → ignore the signal (debounce).

Cargo overflow rule (A1):
  If all cargo 2-6 slots are full and no workbench assembly is ready yet,
  go to the workbench anyway to drop partial materials as a buffer.

In-transit assembly rule (M1 clarification):
  Completed products on cargo 7/8 are delivered directly from cargo 7/8
  to the customer counter — they do NOT move to cargo 1 first.
"""

import threading
from dataclasses import dataclass, field
from typing import Dict, List, Optional

from robocup_planner.execution.cargo_state import CargoManager
from robocup_planner.planning.cargo_allocator import CargoAllocator
from robocup_planner.product_catalog import get_material_count


@dataclass
class Plan:
    """Output of the planning phase, consumed by the Executor."""
    mid: List[Dict]                   # ordered pickup sequence (build_mid output)
    workbench_products: List[int]     # product IDs that require the workbench
    intransit_products: List[int]     # product IDs allocated to cargo 7/8
    workbench_station_id: int
    customer_station_id: int
    home_station_id: int
    # Surplus recycled materials (not needed by any assembly; may arrive on cargo
    # after Phase 1 recycle disassembly — tracked for overflow awareness).
    surplus_recycled: Dict[int, int] = field(default_factory=dict)


class Executor:
    """
    Runs the reactive execution loop. Calls blocking helper methods on
    the PlannerNode (navigate, arm_pick, arm_unload, wb_task, arm_deliver).
    Must run in a dedicated thread while the ROS2 node spins separately.
    """

    def __init__(self, plan: Plan, node):
        self._plan = plan
        self._node = node
        self._cargo = CargoManager()
        self._allocator = CargoAllocator()
        self._allocator.allocate(plan.intransit_products)

        # Workbench signal: set by PlannerNode callback when /workbench/product_ready fires.
        self.wb_signal = threading.Event()
        # Remaining workbench-only products not yet assembled.
        self._pending_wb: List[int] = list(plan.workbench_products)
        # Count of deliverables ready (cargo 1 + completed cargo 7/8 slots).
        self._pending_deliveries: int = 0
        # mid_cursor persists across workbench detours (C1 fix).
        self._mid_cursor: int = 0
        # Prevent re-entrant workbench trips.
        self._en_route_to_wb: bool = False

    # ------------------------------------------------------------------
    # Public entry point
    # ------------------------------------------------------------------

    def run(self) -> None:
        self._log("Executor started")

        # Phase 1: recycle pickups (customer counter → workbench → disassembly)
        self._run_recycle_phase()

        # After Phase 1: reclaimed materials may already satisfy a workbench product.
        if self._should_go_workbench():
            self._divert_to_workbench()
        if self._should_deliver():
            self._deliver_all()

        # Phase 2: main pickup loop
        self._run_pickup_phase()

        # Final delivery of anything remaining
        self._deliver_all()

        self._log("Executor finished")

    # ------------------------------------------------------------------
    # Phase 1 — Recycle pickup
    # ------------------------------------------------------------------

    def _run_recycle_phase(self) -> None:
        recycle_entries = [
            e for e in self._plan.mid if e['is_recycle_pickup']
        ]
        if not recycle_entries:
            return

        self._log(f"Phase 1: collecting {len(recycle_entries)} recycled product(s)")

        # Collect all products from customer counters first.
        for entry in recycle_entries:
            self._log(f"  → navigate to customer station {entry['station_id']}")
            self._node.navigate(entry['station_id'])
            self._log(f"  → pick product {entry['recycle_product_id']}")
            self._node.arm_pick_product(
                station_id=entry['station_id'],
                product_id=entry['recycle_product_id'],
            )

        # Bring all collected products to the workbench for disassembly.
        self._log(f"  → navigate to workbench {self._plan.workbench_station_id}")
        self._node.navigate(self._plan.workbench_station_id)

        for entry in recycle_entries:
            pid = entry['recycle_product_id']
            self._log(f"  → workbench disassembly of product {pid}")
            self._node.wb_task('RECYCLE', pid)
            # After disassembly the arm places reclaimed materials onto cargo 2-6.
            # We update cargo state to reflect the incoming materials.
            reclaimed = get_material_count(pid)
            for mat_id, count in reclaimed.items():
                for _ in range(count):
                    placement = self._cargo.place_material(mat_id)
                    if placement is None:
                        self._log(
                            f"  ! cargo full during recycle unload of mat {mat_id}; "
                            "surplus ignored"
                        )

    # ------------------------------------------------------------------
    # Phase 2 — Main pickup loop
    # ------------------------------------------------------------------

    def _run_pickup_phase(self) -> None:
        storage_entries = [
            e for e in self._plan.mid if not e['is_recycle_pickup']
        ]
        self._mid_cursor = 0

        while self._mid_cursor < len(storage_entries):
            entry = storage_entries[self._mid_cursor]
            self._log(
                f"[{self._mid_cursor}/{len(storage_entries)-1}] "
                f"navigate to storage station {entry['station_id']}"
            )
            self._node.navigate(entry['station_id'])

            # Pick each required material from this station.
            for mat_id in entry['pickup_materials']:
                placement_value = self._decide_placement(mat_id)

                if placement_value is None:
                    # Cargo 2-6 full and no material fits — divert to workbench now.
                    self._log("  ! cargo full — diverting to workbench before pickup")
                    self._divert_to_workbench(forced=True)
                    placement_value = self._decide_placement(mat_id)
                    if placement_value is None:
                        self._log(f"  !! still no space for mat {mat_id} — skipping")
                        continue
                    # Re-navigate to the storage station after the workbench detour.
                    self._node.navigate(entry['station_id'])

                self._log(f"  → pick mat {mat_id} → placement {placement_value}")
                self._node.arm_pick_material(
                    station_id=entry['station_id'],
                    material_id=mat_id,
                    manipulator_slot=placement_value,
                )
                self._on_material_placed(mat_id, placement_value)

            self._mid_cursor += 1

            # Post-station checks (order: workbench first, then delivery).
            if self._should_go_workbench():
                self._divert_to_workbench()

            if self._should_deliver():
                self._deliver_all()

    # ------------------------------------------------------------------
    # Cargo placement decision
    # ------------------------------------------------------------------

    def _decide_placement(self, material_id: int) -> Optional[int]:
        """
        Returns the manipulator slot value for placing material_id, or None
        if no space is available anywhere.

        Priority:
          1. Cargo 7/8 if waiting for this block as the next in build order.
          2. Cargo 2-6 lowest-preferred free position.
        """
        cargo_intransit = self._allocator.find_slot_for_block(material_id)
        if cargo_intransit is not None:
            # In-transit cargo uses placement index 0 (single stack position).
            return cargo_intransit * 10 + 0

        return self._cargo.place_material(material_id)

    def _on_material_placed(self, material_id: int, placement_value: int) -> None:
        """Update state after a block is successfully placed."""
        cargo_id = placement_value // 10
        if cargo_id in (7, 8):
            complete = self._allocator.confirm_placed(cargo_id, material_id)
            if complete:
                self._log(
                    f"  [DONE] in-transit assembly complete on cargo {cargo_id}"
                )
                self._pending_deliveries += 1

    # ------------------------------------------------------------------
    # Workbench divert
    # ------------------------------------------------------------------

    def _should_go_workbench(self) -> bool:
        if self._en_route_to_wb:
            return False
        if self.wb_signal.is_set():
            return True
        # Check if any pending workbench product has all its materials ready.
        return self._cargo.can_assemble_for_workbench(self._pending_wb) is not None

    def _divert_to_workbench(self, forced: bool = False) -> None:
        """
        Navigate to the workbench, unload materials for one product,
        start workbench assembly, then return.  The mid_cursor is preserved
        so pickup resumes from the same station (C1 fix).
        """
        if self._en_route_to_wb and not forced:
            return

        self._en_route_to_wb = True
        self.wb_signal.clear()

        self._log(f"  → divert to workbench {self._plan.workbench_station_id}")
        self._node.navigate(self._plan.workbench_station_id)

        ready_pid = self._cargo.can_assemble_for_workbench(self._pending_wb)
        if ready_pid is not None:
            self._unload_product_materials(ready_pid)
            self._log(f"  → workbench assemble product {ready_pid}")
            self._node.wb_task('PRODUCE', ready_pid)
            self._pending_wb.remove(ready_pid)
            self._cargo.add_finished_product()
            self._pending_deliveries += 1
        elif forced:
            # Overflow drop: unload whatever is on cargo to free space.
            self._log("  → overflow drop: unloading all cargo to workbench")
            self._unload_all_materials()

        self._en_route_to_wb = False

    def _unload_product_materials(self, product_id: int) -> None:
        """Remove materials for product_id from cargo 2-6 (arm drops them at workbench)."""
        slots = self._cargo.find_materials_for_product(product_id)
        if slots is None:
            return
        for cargo_id, placement_idx, mat_id in slots:
            self._node.arm_unload_material(cargo_id, placement_idx)
            self._cargo.remove_material(cargo_id, placement_idx)

    def _unload_all_materials(self) -> None:
        """Drop everything in cargo 2-6 at the workbench (overflow buffer)."""
        for cargo_id, placement_idx, mat_id in list(self._cargo.all_materials()):
            self._node.arm_unload_material(cargo_id, placement_idx)
            self._cargo.remove_material(cargo_id, placement_idx)

    # ------------------------------------------------------------------
    # Delivery
    # ------------------------------------------------------------------

    def _should_deliver(self) -> bool:
        return self._pending_deliveries > 0

    def _deliver_all(self) -> None:
        if not self._should_deliver():
            return

        self._log(f"  → navigate to customer {self._plan.customer_station_id}")
        self._node.navigate(self._plan.customer_station_id)

        # Deliver products from cargo 1 (assembled at workbench).
        while self._cargo.finished_on_cargo1 > 0:
            self._log("  → deliver product from cargo 1")
            self._node.arm_deliver(from_cargo_id=1)
            self._cargo.consume_finished_product()
            self._pending_deliveries -= 1

        # Deliver in-transit assembled products directly from cargo 7/8 (M1 clarification).
        for slot in list(self._allocator.get_completed_slots()):
            self._log(
                f"  → deliver in-transit product {slot.product_id} from cargo {slot.cargo_id}"
            )
            self._node.arm_deliver(from_cargo_id=slot.cargo_id)
            self._allocator.free_slot(slot.cargo_id)
            self._pending_deliveries -= 1

    # ------------------------------------------------------------------
    # Utility
    # ------------------------------------------------------------------

    def _log(self, msg: str) -> None:
        self._node.get_logger().info(f"[Executor] {msg}")
