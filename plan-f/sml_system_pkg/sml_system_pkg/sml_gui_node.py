"""Plan D runtime visualization GUI for sml_system_pkg.

This GUI intentionally keeps ROS messages unchanged. It subscribes to /sml/task,
requests /sml/get_plan, and applies planned steps to a virtual AMR/station state.

Compared with the first GUI draft, this version:
- removes the static reference tables (object dictionary / raw IDs / complexity),
- makes the map the main view,
- loads the same station coordinate JSON used by the planner,
- supports y-axis inversion for planner coordinates whose origin is bottom-left,
- draws only the active zone by default instead of wasting space on the full arena,
- renders station/AMR cargo as colored block cells so placement is visually inspectable,
- updates virtual station/WB/AMR contents when steps are applied, including batch consumption,
- can use an official-photo-like one-zone station layout so the screen matches the arena figure,
- keeps plan steps, AMR slots, stations, and orders as operational views.

Run:
    ros2 run sml_system_pkg sml_gui_node

Optional:
    ros2 run sml_system_pkg sml_gui_node --ros-args \
      -p side:=a \
      -p station_coord_json:=/home/user/ros2_ws/src/sml_system_pkg/config/station_coordinates_a_zone.json \
      -p gui_invert_y:=true

If tkinter is missing:
    sudo apt install python3-tk
"""

from __future__ import annotations

import json
import os
import threading
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node

from sml_msgs.msg import Task, Step
from sml_msgs.srv import GetPlan

try:
    import tkinter as tk
    from tkinter import ttk, messagebox
except Exception as exc:  # pragma: no cover
    tk = None
    ttk = None
    messagebox = None
    _TK_IMPORT_ERROR = exc
else:
    _TK_IMPORT_ERROR = None

try:
    from ament_index_python.packages import get_package_share_directory
except Exception:  # pragma: no cover
    get_package_share_directory = None


# ---------------------------------------------------------------------------
# Static Plan D knowledge used only for labels/classification
# ---------------------------------------------------------------------------

PRODUCT_DB: Dict[int, Tuple[str, List[int]]] = {
    34: ("Battery", [3, 4]),
    13: ("Magnet", [1, 3]),
    81: ("E-Stop", [8, 1]),
    442: ("Carrot", [4, 4, 2]),
    241: ("Traffic Light", [2, 4, 1]),
    462: ("Small Tree", [4, 6, 2]),
    711: ("Hammer", [1, 1, 7]),
    4482: ("Big Carrot", [4, 4, 8, 2]),
    8518: ("Burger", [8, 5, 1, 8]),
    48132: ("Ice Cream", [4, 8, 1, 3, 2]),
    46262: ("Big Tree", [4, 6, 2, 6, 2]),
}

AMR_PRODUCIBLE_PRODUCTS = {34, 13, 81, 442, 241, 462, 711, 4482}
WB_ONLY_PRODUCTS = {8518, 48132, 46262}

RAW_NAMES = {
    1: "2x2_red",
    2: "2x2_green",
    3: "2x2_blue",
    4: "2x2_yellow",
    5: "4x2_red",
    6: "4x2_green",
    7: "4x2_blue",
    8: "4x2_yellow",
}

BATCH_TO_RAW = {10: 1, 20: 2, 30: 3, 40: 4, 50: 5, 60: 6, 70: 7, 80: 8}

STATION_TYPE_NAMES = {1: "Storage", 2: "Workbench", 3: "Customer", 4: "Hybrid"}
STEP_TYPE_NAMES = {0: "AMR", 1: "WB"}
STEP_ACTION_NAMES = {0: "LOAD", 1: "UNLOAD", 2: "PRODUCE", 3: "RECYCLE", 4: "GOAL"}
SLOT_LABELS = {
    0: "0 Product",
    1: "1 Raw",
    2: "2 Raw",
    3: "3 Raw",
    4: "4 Raw",
    5: "5 Raw",
    6: "6 Assembly",
    7: "7 Assembly",
}


RAW_COLORS = {
    1: "#e53935",  # red
    2: "#43a047",  # green
    3: "#039be5",  # blue
    4: "#fdd835",  # yellow
    5: "#ef9a9a",  # 4x2 red
    6: "#a5d6a7",  # 4x2 green
    7: "#81d4fa",  # 4x2 blue
    8: "#ffe082",  # 4x2 yellow
}
PRODUCT_COLOR = "#ffb74d"
BATCH_BORDER = "#6d4c41"
EMPTY_COLOR = "#f5f5f5"
TEXT_DARK = "#222222"



@dataclass
class SlotObject:
    object_id: int
    slide_id: int
    label: str = ""

    @property
    def slot_index(self) -> int:
        return decode_slide_id(self.slide_id)[1]


@dataclass
class VirtualState:
    station_items: Dict[int, List[int]] = field(default_factory=dict)
    station_names: Dict[int, str] = field(default_factory=dict)
    station_types: Dict[int, int] = field(default_factory=dict)
    amr_slots: Dict[int, List[SlotObject]] = field(default_factory=lambda: {i: [] for i in range(8)})
    wb_items: Dict[int, List[int]] = field(default_factory=dict)
    batch_remaining: Dict[Tuple[int, int], int] = field(default_factory=dict)
    amr_station: int = 0
    wb_status: str = "idle"

    def clone_from_task(self, task: Task) -> None:
        self.station_items.clear()
        self.station_names.clear()
        self.station_types.clear()
        self.wb_items.clear()
        self.batch_remaining.clear()
        self.amr_slots = {i: [] for i in range(8)}
        self.amr_station = 0
        self.wb_status = "idle"

        for st in task.arena_layout:
            sid = int(st.station_id)
            self.station_items[sid] = list(getattr(st, "material_ids", []))
            self.station_names[sid] = getattr(st, "station_name", f"station_{sid}") or f"station_{sid}"
            self.station_types[sid] = int(st.station_type)
            for obj in list(getattr(st, "material_ids", [])):
                if int(obj) in BATCH_TO_RAW:
                    self.batch_remaining[(sid, int(obj))] = 5
            if int(st.station_type) == 2:
                self.wb_items[sid] = list(getattr(st, "material_ids", []))


# ---------------------------------------------------------------------------
# Utilities
# ---------------------------------------------------------------------------

def product_name(pid: int) -> str:
    return PRODUCT_DB.get(int(pid), (str(pid), []))[0]


def recipe(pid: int) -> List[int]:
    return list(PRODUCT_DB.get(int(pid), ("", []))[1])


def object_label(object_id: int) -> str:
    oid = int(object_id)
    if oid in RAW_NAMES:
        return f"{oid}:{RAW_NAMES[oid]}"
    if oid in BATCH_TO_RAW:
        return f"{oid}:batch(raw {BATCH_TO_RAW[oid]})"
    if oid == 90:
        return "90:mix_batch"
    if oid in PRODUCT_DB:
        return f"{oid}:{PRODUCT_DB[oid][0]}"
    return str(oid)


def decode_slide_id(slide_id: int) -> Tuple[Optional[int], int, bool]:
    """Return (order_or_return_station, slot_index, is_return)."""
    sid = int(slide_id)
    if sid < 0:
        val = abs(sid)
        return val // 10, val % 10, True
    return sid // 10, sid % 10, False


def slide_label(slide_id: int) -> str:
    order_or_station, slot, is_return = decode_slide_id(slide_id)
    if is_return:
        return f"{slide_id}(ret S{order_or_station},slot {slot})"
    return f"{slide_id}(order {order_or_station},slot {slot})"


def compact_list(values: List[int], limit: int = 8) -> str:
    vals = list(values)
    if len(vals) <= limit:
        return "[" + ",".join(map(str, vals)) + "]"
    head = ",".join(map(str, vals[:limit]))
    return f"[{head},...+{len(vals) - limit}]"


def remove_one(values: List[int], obj: int) -> bool:
    try:
        values.remove(int(obj))
        return True
    except ValueError:
        return False


def batch_id_for_raw(raw: int) -> Optional[int]:
    for batch_id, batch_raw in BATCH_TO_RAW.items():
        if int(batch_raw) == int(raw):
            return int(batch_id)
    return None


# ---------------------------------------------------------------------------
# GUI Node
# ---------------------------------------------------------------------------

class SmlGuiNode(Node):
    def __init__(self, root: "tk.Tk"):
        super().__init__("sml_gui_node")
        self.root = root

        self.declare_parameter("task_topic", "/sml/task")
        self.declare_parameter("get_plan_service", "/sml/get_plan")
        self.declare_parameter("side", "a")
        self.declare_parameter("station_coord_json", "")
        self.declare_parameter("gui_invert_y", True)
        self.declare_parameter("gui_zone_mode", "zone")  # zone, full, active_side, auto
        self.declare_parameter("gui_layout_mode", "official_photo")  # official_photo or json
        self.declare_parameter("gui_show_cargo", True)

        self.task_topic = self.get_parameter("task_topic").value
        self.get_plan_service = self.get_parameter("get_plan_service").value
        self.side = str(self.get_parameter("side").value).lower().strip() or "a"
        self.gui_invert_y = bool(self.get_parameter("gui_invert_y").value)
        self.gui_zone_mode = str(self.get_parameter("gui_zone_mode").value or "zone").lower().strip()
        self.gui_layout_mode = str(self.get_parameter("gui_layout_mode").value or "official_photo").lower().strip()
        self.gui_show_cargo = bool(self.get_parameter("gui_show_cargo").value)

        self.current_task: Optional[Task] = None
        self.steps: List[Step] = []
        self.state = VirtualState()
        self.applied_step_ids: set[int] = set()
        self.next_apply_index = 0

        self.map_width_m = 10.0
        self.map_height_m = 7.5
        self.coord_source_path = "fallback"
        self.station_coords: Dict[int, Tuple[float, float]] = {}
        self.station_coord_names: Dict[int, str] = {}
        self.station_coords = self._load_station_coords()

        self.create_subscription(Task, self.task_topic, self._task_callback, 10)
        self.plan_client = self.create_client(GetPlan, self.get_plan_service)

        self._build_ui()
        self._redraw_arena()
        self._refresh_state_tables()

        self.get_logger().info(
            f"SML GUI 시작 | task_topic={self.task_topic} | get_plan_service={self.get_plan_service} | "
            f"side={self.side} | layout={self.gui_layout_mode} | coord_json={self.coord_source_path} | invert_y={self.gui_invert_y}"
        )

    # ----------------------------- UI --------------------------------------

    def _build_ui(self) -> None:
        self.root.title("SML Plan D Runtime GUI")
        self.root.geometry("1480x900")

        main = ttk.Frame(self.root)
        main.pack(fill="both", expand=True)

        top = ttk.Frame(main)
        top.pack(fill="x", padx=6, pady=4)
        self.status_var = tk.StringVar(value="대기 중: /sml/task 수신 전")
        ttk.Label(top, textvariable=self.status_var).pack(side="left", padx=4)
        ttk.Button(top, text="GetPlan 요청", command=self.request_plan).pack(side="right", padx=4)
        ttk.Button(top, text="상태 초기화", command=self.reset_virtual_state).pack(side="right", padx=4)
        ttk.Button(top, text="다음 step 적용", command=self.apply_next_step).pack(side="right", padx=4)
        ttk.Button(top, text="선택 step 적용", command=self.apply_selected_step).pack(side="right", padx=4)

        pane = ttk.PanedWindow(main, orient="horizontal")
        pane.pack(fill="both", expand=True, padx=6, pady=4)
        left = ttk.Frame(pane)
        right = ttk.Frame(pane)
        pane.add(left, weight=7)
        pane.add(right, weight=5)

        # Large arena canvas.
        canvas_frame = ttk.LabelFrame(left, text="Arena map / virtual state")
        canvas_frame.pack(fill="both", expand=True)
        self.canvas = tk.Canvas(canvas_frame, bg="white", height=640)
        self.canvas.pack(fill="both", expand=True, padx=4, pady=4)
        self.canvas.bind("<Configure>", lambda _e: self._redraw_arena())

        map_info = ttk.Frame(left)
        map_info.pack(fill="x", pady=(3, 0))
        self.coord_var = tk.StringVar(value=f"coord: {self.coord_source_path}")
        ttk.Label(map_info, textvariable=self.coord_var).pack(side="left", padx=4)
        ttk.Button(map_info, text="지도 다시 그리기", command=self._redraw_arena).pack(side="right", padx=4)

        # Right top: AMR + Stations.
        top_right = ttk.PanedWindow(right, orient="vertical")
        top_right.pack(fill="both", expand=True)

        state_pane = ttk.PanedWindow(top_right, orient="horizontal")
        top_right.add(state_pane, weight=3)

        amr_frame = ttk.LabelFrame(state_pane, text="AMR slots")
        state_pane.add(amr_frame, weight=2)
        self.slot_tree = ttk.Treeview(amr_frame, columns=("slot", "type", "contents"), show="headings", height=8)
        for col, text, width in [("slot", "Slot", 70), ("type", "Type", 100), ("contents", "Contents", 360)]:
            self.slot_tree.heading(col, text=text)
            self.slot_tree.column(col, width=width, anchor="w")
        self.slot_tree.pack(fill="both", expand=True, side="left")
        slot_scroll = ttk.Scrollbar(amr_frame, orient="vertical", command=self.slot_tree.yview)
        self.slot_tree.configure(yscrollcommand=slot_scroll.set)
        slot_scroll.pack(side="right", fill="y")

        station_frame = ttk.LabelFrame(state_pane, text="Stations")
        state_pane.add(station_frame, weight=3)
        self.station_tree = ttk.Treeview(
            station_frame,
            columns=("id", "name", "type", "items"),
            show="headings",
            height=10,
        )
        for col, text, width in [("id", "ID", 45), ("name", "Name", 160), ("type", "Type", 95), ("items", "Items", 240)]:
            self.station_tree.heading(col, text=text)
            self.station_tree.column(col, width=width, anchor="w")
        self.station_tree.pack(fill="both", expand=True, side="left")
        station_scroll = ttk.Scrollbar(station_frame, orient="vertical", command=self.station_tree.yview)
        self.station_tree.configure(yscrollcommand=station_scroll.set)
        station_scroll.pack(side="right", fill="y")

        order_frame = ttk.LabelFrame(top_right, text="Orders")
        top_right.add(order_frame, weight=2)
        self.order_tree = ttk.Treeview(
            order_frame,
            columns=("idx", "type", "product", "name", "recipe", "class"),
            show="headings",
            height=7,
        )
        for col, text, width in [
            ("idx", "Idx", 45), ("type", "Type", 75), ("product", "PID", 70),
            ("name", "Name", 120), ("recipe", "Recipe", 120), ("class", "Class", 180),
        ]:
            self.order_tree.heading(col, text=text)
            self.order_tree.column(col, width=width, anchor="w")
        self.order_tree.pack(fill="both", expand=True, side="left")
        order_scroll = ttk.Scrollbar(order_frame, orient="vertical", command=self.order_tree.yview)
        self.order_tree.configure(yscrollcommand=order_scroll.set)
        order_scroll.pack(side="right", fill="y")

        step_frame = ttk.LabelFrame(top_right, text="Plan steps")
        top_right.add(step_frame, weight=5)
        self.step_tree = ttk.Treeview(
            step_frame,
            columns=("id", "type", "action", "station", "objects", "slides", "deps", "state"),
            show="headings",
            height=16,
        )
        headings = {
            "id": "ID", "type": "Type", "action": "Action", "station": "Station",
            "objects": "Objects", "slides": "Slide IDs", "deps": "Depends", "state": "State",
        }
        widths = {"id": 45, "type": 60, "action": 80, "station": 65, "objects": 155, "slides": 190, "deps": 120, "state": 65}
        for col, text in headings.items():
            self.step_tree.heading(col, text=text)
            self.step_tree.column(col, width=widths[col], anchor="center" if col in {"id", "type", "action", "station", "state"} else "w")
        self.step_tree.pack(fill="both", expand=True, side="left")
        step_scroll = ttk.Scrollbar(step_frame, orient="vertical", command=self.step_tree.yview)
        self.step_tree.configure(yscrollcommand=step_scroll.set)
        step_scroll.pack(side="right", fill="y")

    # ----------------------------- ROS callbacks ----------------------------

    def _task_callback(self, msg: Task) -> None:
        self.root.after(0, lambda: self._on_task_received(msg))

    def _on_task_received(self, task: Task) -> None:
        self.current_task = task
        self.steps = []
        self.applied_step_ids.clear()
        self.next_apply_index = 0
        self.state.clone_from_task(task)
        self.status_var.set(
            f"Task 수신: orders={len(task.order_list)}, stations={len(task.arena_layout)} | GetPlan 요청 가능"
        )
        self._refresh_order_table()
        self._refresh_step_table()
        self._refresh_state_tables()
        self._redraw_arena()

    def request_plan(self) -> None:
        if self.current_task is None:
            messagebox.showwarning("No task", "아직 /sml/task를 수신하지 않았습니다.")
            return
        if not self.plan_client.service_is_ready():
            if not self.plan_client.wait_for_service(timeout_sec=0.2):
                messagebox.showwarning("GetPlan", f"서비스가 준비되지 않았습니다: {self.get_plan_service}")
                return
        future = self.plan_client.call_async(GetPlan.Request())
        future.add_done_callback(lambda fut: self.root.after(0, lambda: self._on_plan_response(fut)))
        self.status_var.set("GetPlan 요청 중...")

    def _on_plan_response(self, future) -> None:
        try:
            res = future.result()
        except Exception as exc:
            messagebox.showerror("GetPlan failed", str(exc))
            self.status_var.set("GetPlan 실패")
            return
        if not getattr(res, "success", True):
            messagebox.showerror("GetPlan failed", getattr(res, "message", "unknown error"))
            self.status_var.set("GetPlan 실패")
            return
        self.steps = list(res.steps)
        self.applied_step_ids.clear()
        self.next_apply_index = 0
        self.reset_virtual_state(redraw=False)
        self.status_var.set(f"Plan 수신: {len(self.steps)} steps")
        self._refresh_step_table()
        self._refresh_state_tables()
        self._redraw_arena()

    # --------------------------- Virtual state ------------------------------

    def reset_virtual_state(self, redraw: bool = True) -> None:
        if self.current_task is not None:
            self.state.clone_from_task(self.current_task)
        self.applied_step_ids.clear()
        self.next_apply_index = 0
        if redraw:
            self.status_var.set("가상 상태 초기화 완료")
            self._refresh_step_table()
            self._refresh_state_tables()
            self._redraw_arena()

    def apply_selected_step(self) -> None:
        selected = self.step_tree.selection()
        if not selected:
            messagebox.showinfo("선택 없음", "적용할 step을 선택하세요.")
            return
        step_id = int(self.step_tree.item(selected[0], "values")[0])
        for st in self.steps:
            if int(st.step_id) == step_id:
                self._apply_step(st)
                self._refresh_step_table()
                self._refresh_state_tables()
                self._redraw_arena()
                return

    def apply_next_step(self) -> None:
        if self.next_apply_index >= len(self.steps):
            messagebox.showinfo("완료", "더 이상 적용할 step이 없습니다.")
            return
        st = self.steps[self.next_apply_index]
        self._apply_step(st)
        self.next_apply_index += 1
        while self.next_apply_index < len(self.steps) and int(self.steps[self.next_apply_index].step_id) in self.applied_step_ids:
            self.next_apply_index += 1
        self._refresh_step_table()
        self._refresh_state_tables()
        self._redraw_arena()

    def _apply_step(self, st: Step) -> None:
        sid = int(st.step_id)
        if sid in self.applied_step_ids:
            return

        step_type = int(st.type)
        action = int(st.action)
        station_id = int(st.station_id)
        objects = list(getattr(st, "object_ids", []))
        slides = list(getattr(st, "slide_ids", []))
        self.state.amr_station = station_id

        if step_type == 0:  # AMR
            if action == 0:  # LOAD
                self._state_amr_load(station_id, objects, slides)
            elif action == 1:  # UNLOAD
                self._state_amr_unload(station_id, objects, slides)
            elif action == 2:  # AMR PRODUCE
                self._state_amr_produce(objects, slides)
            elif action == 4:  # GOAL
                self.state.amr_station = 0
        elif step_type == 1:  # WB
            if action == 2:  # PRODUCE complete
                for obj in objects:
                    self._state_wb_produce_complete(station_id, int(obj))
            elif action == 3:  # RECYCLE complete
                for pid in objects:
                    self._state_wb_recycle_complete(station_id, int(pid))

        self.applied_step_ids.add(sid)
        self.status_var.set(
            f"step {sid} 적용: {STEP_TYPE_NAMES.get(step_type, step_type)} "
            f"{STEP_ACTION_NAMES.get(action, action)} objects={objects} station={station_id}"
        )

    def _consume_station_object(self, station_id: int, obj: int) -> bool:
        """Remove one object from station visualization.

        Raw objects may be supplied by a batch material in the Task. In that case
        we decrement the visual batch count instead of leaving the batch unchanged.
        """
        station_id = int(station_id)
        obj = int(obj)
        items = self.state.station_items.setdefault(station_id, [])

        # Direct object/product/raw match.
        if remove_one(items, obj):
            return True

        # If AMR loads raw 1~8 from a batch object 10~80, decrement that batch.
        if 1 <= obj <= 8:
            batch_id = batch_id_for_raw(obj)
            if batch_id is not None and batch_id in items:
                key = (station_id, batch_id)
                remaining = int(self.state.batch_remaining.get(key, 5))
                remaining -= 1
                if remaining <= 0:
                    remove_one(items, batch_id)
                    self.state.batch_remaining.pop(key, None)
                else:
                    self.state.batch_remaining[key] = remaining
                return True

        return False

    def _consume_wb_object(self, station_id: int, obj: int) -> bool:
        station_id = int(station_id)
        obj = int(obj)
        removed = False
        if station_id in self.state.wb_items:
            removed = remove_one(self.state.wb_items[station_id], obj) or removed
        removed = self._consume_station_object(station_id, obj) or removed
        return removed

    def _remove_from_amr_slot(self, obj: int, slide: int) -> bool:
        obj = int(obj)
        slide = int(slide)
        _, slot, _ = decode_slide_id(slide)
        # Exact match first.
        for slot_obj in list(self.state.amr_slots.get(slot, [])):
            if slot_obj.object_id == obj and slot_obj.slide_id == slide:
                self.state.amr_slots[slot].remove(slot_obj)
                return True
        # Fallback by object ID. Useful when a mock/older planner omits slide IDs.
        for slot_items in self.state.amr_slots.values():
            for slot_obj in list(slot_items):
                if slot_obj.object_id == obj:
                    slot_items.remove(slot_obj)
                    return True
        return False

    def _state_amr_load(self, station_id: int, objects: List[int], slides: List[int]) -> None:
        if not slides:
            slides = [0 for _ in objects]
        for obj, slide in zip(objects, slides):
            _, slot, _ = decode_slide_id(int(slide))
            if slot not in self.state.amr_slots:
                self.state.amr_slots[slot] = []

            # Remove the loaded object from the visual source station/WB.
            if self.state.station_types.get(int(station_id)) == 2:
                self._consume_wb_object(station_id, int(obj))
            else:
                self._consume_station_object(station_id, int(obj))

            self.state.amr_slots[slot].append(SlotObject(int(obj), int(slide), object_label(int(obj))))

    def _state_amr_unload(self, station_id: int, objects: List[int], slides: List[int]) -> None:
        if not slides:
            slides = [0 for _ in objects]
        station_id = int(station_id)
        self.state.station_items.setdefault(station_id, [])
        for obj, slide in zip(objects, slides):
            self._remove_from_amr_slot(int(obj), int(slide))
            self.state.station_items[station_id].append(int(obj))
            if self.state.station_types.get(station_id) == 2:
                self.state.wb_items.setdefault(station_id, []).append(int(obj))

    def _state_amr_produce(self, objects: List[int], slides: List[int]) -> None:
        if not objects:
            return
        product_id = int(objects[0])
        if slides:
            _, slot, _ = decode_slide_id(int(slides[0]))
            used_slots = [decode_slide_id(int(s))[1] for s in slides]
            out_slide = int(slides[0])
        else:
            slot = 6
            used_slots = []
            out_slide = slot
        for us in used_slots:
            if us in self.state.amr_slots:
                self.state.amr_slots[us].clear()
        self.state.amr_slots.setdefault(slot, []).append(SlotObject(product_id, out_slide, object_label(product_id)))

    def _state_wb_produce_complete(self, station_id: int, product_id: int) -> None:
        station_id = int(station_id)
        product_id = int(product_id)
        mats = recipe(product_id)
        self.state.station_items.setdefault(station_id, [])
        self.state.wb_items.setdefault(station_id, [])
        # Consume input raw materials from WB visualization.
        for raw in mats:
            remove_one(self.state.station_items[station_id], int(raw))
            remove_one(self.state.wb_items[station_id], int(raw))
        # Add produced product waiting at WB.
        self.state.station_items[station_id].append(product_id)
        self.state.wb_items[station_id].append(product_id)
        self.state.wb_status = f"produced {product_id}"

    def _state_wb_recycle_complete(self, station_id: int, product_id: int) -> None:
        station_id = int(station_id)
        product_id = int(product_id)
        mats = recipe(product_id)
        self.state.station_items.setdefault(station_id, [])
        self.state.wb_items.setdefault(station_id, [])
        # Consume recycled product from WB visualization.
        remove_one(self.state.station_items[station_id], product_id)
        remove_one(self.state.wb_items[station_id], product_id)
        # Add disassembled raw materials waiting at WB.
        self.state.station_items[station_id].extend(mats)
        self.state.wb_items[station_id].extend(mats)
        self.state.wb_status = f"recycled {product_id}"

    # --------------------------- Refresh tables -----------------------------

    def _refresh_order_table(self) -> None:
        self.order_tree.delete(*self.order_tree.get_children())
        if self.current_task is None:
            return
        for idx, order in enumerate(self.current_task.order_list):
            order_type = int(order.order_type)
            pid = int(order.product_id)
            if order_type == 1:
                typ = "PRODUCE"
                cls = "AMR-producible" if pid in AMR_PRODUCIBLE_PRODUCTS else "WB-only"
            elif order_type == 2:
                typ = "RECYCLE"
                cls = "WB disassembly"
            else:
                typ = str(order_type)
                cls = "unknown"
            self.order_tree.insert("", "end", values=(idx, typ, pid, product_name(pid), str(recipe(pid)), cls))

    def _refresh_step_table(self) -> None:
        self.step_tree.delete(*self.step_tree.get_children())
        for st in self.steps:
            sid = int(st.step_id)
            values = (
                sid,
                STEP_TYPE_NAMES.get(int(st.type), str(st.type)),
                STEP_ACTION_NAMES.get(int(st.action), str(st.action)),
                int(st.station_id),
                compact_list(list(st.object_ids), 10),
                compact_list(list(st.slide_ids), 10),
                compact_list(list(st.depends_on), 10),
                "done" if sid in self.applied_step_ids else "wait",
            )
            self.step_tree.insert("", "end", values=values)

    def _refresh_state_tables(self) -> None:
        self.slot_tree.delete(*self.slot_tree.get_children())
        for slot in range(8):
            objs = self.state.amr_slots.get(slot, [])
            contents = "; ".join(f"{o.object_id}@{slide_label(o.slide_id)}" for o in objs) or "empty"
            slot_type = "Product" if slot == 0 else "Raw slide" if 1 <= slot <= 5 else "Assembly"
            self.slot_tree.insert("", "end", values=(slot, slot_type, contents))

        self.station_tree.delete(*self.station_tree.get_children())
        for sid in sorted(self.state.station_items):
            items = self.state.station_items.get(sid, [])
            name = self.state.station_names.get(sid, self.station_coord_names.get(sid, f"station_{sid}"))
            stype = STATION_TYPE_NAMES.get(self.state.station_types.get(sid, 0), str(self.state.station_types.get(sid, "")))
            display_items = []
            for it in self._station_display_items(sid):
                if isinstance(it, tuple):
                    bid, rem = it
                    display_items.append(f"{bid}(x{rem})")
                else:
                    display_items.append(str(it))
            self.station_tree.insert("", "end", values=(sid, name, stype, ", ".join(display_items)))

    # --------------------------- Arena canvas --------------------------------

    def _official_photo_station_coords(self) -> Dict[int, Tuple[float, float]]:
        """One-zone coordinates arranged to resemble the official arena figure.

        The visible zone is 5.0 m × 7.5 m. A-side and B-side are drawn with the
        same local layout; B station IDs 9~16 are converted to local 1~8 by
        _coord_for_station().
        """
        self.map_width_m = 5.0
        self.map_height_m = 7.5
        self.coord_source_path = "official_photo_layout_5x7.5"
        self.station_coord_names = {
            0: "START_GOAL",
            1: "STORAGE_1",
            2: "STORAGE_2",
            3: "WORKBENCH_1",
            4: "STORAGE_3",
            5: "HYBRID_1",
            6: "WORKBENCH_2",
            7: "WORKBENCH_3",
            8: "CUSTOMER_1",
        }
        # Origin is bottom-left. y is inverted when drawn by default.
        return {
            0: (3.35, 0.95),   # start/goal near bottom-right of the active zone
            1: (1.35, 0.70),
            2: (0.80, 0.70),
            3: (0.45, 2.35),
            4: (0.50, 3.45),
            5: (1.65, 3.95),
            6: (2.45, 3.95),
            7: (3.15, 3.45),
            8: (3.45, 6.55),
        }

    def _load_station_coords(self) -> Dict[int, Tuple[float, float]]:
        if self.gui_layout_mode in {"official", "official_photo", "photo"}:
            return self._official_photo_station_coords()

        param_path = str(self.get_parameter("station_coord_json").value or "").strip()
        candidates = []
        if param_path:
            candidates.append(param_path)
        # Prefer source tree path because user usually edits this file and planner logs this path.
        candidates.append(os.path.expanduser("~/ros2_ws/src/sml_system_pkg/config/station_coordinates_a_zone.json"))
        if get_package_share_directory is not None:
            try:
                share = get_package_share_directory("sml_system_pkg")
                candidates.append(os.path.join(share, "config", "station_coordinates_a_zone.json"))
            except Exception:
                pass

        for path in candidates:
            if not path or not os.path.exists(path):
                continue
            try:
                with open(path, "r", encoding="utf-8") as f:
                    data = json.load(f)

                map_info = data.get("map", {}) if isinstance(data, dict) else {}
                self.map_width_m = float(map_info.get("width_m", self.map_width_m))
                self.map_height_m = float(map_info.get("height_m", self.map_height_m))

                coords: Dict[int, Tuple[float, float]] = {}
                names: Dict[int, str] = {}
                raw = data.get("station_coordinates", data) if isinstance(data, dict) else {}
                for key, val in raw.items():
                    try:
                        sid = int(key)
                    except Exception:
                        continue
                    if isinstance(val, dict):
                        coords[sid] = (float(val.get("x", 0.0)), float(val.get("y", 0.0)))
                        if val.get("name"):
                            names[sid] = str(val.get("name"))
                    elif isinstance(val, (list, tuple)) and len(val) >= 2:
                        coords[sid] = (float(val[0]), float(val[1]))

                if coords:
                    self.station_coord_names = names
                    self.coord_source_path = path
                    self.get_logger().info(
                        f"GUI station coordinates loaded: {path} | map={self.map_width_m}x{self.map_height_m}m | stations={len(coords)}"
                    )
                    return coords
            except Exception as exc:
                self.get_logger().warn(f"GUI station coordinate load failed: {path}: {exc}")

        self.coord_source_path = "fallback approximate coordinates"
        return self._default_station_coords()

    def _default_station_coords(self) -> Dict[int, Tuple[float, float]]:
        self.map_width_m = 10.0
        self.map_height_m = 7.5
        # Coordinates are bottom-left-origin; GUI defaults to invert_y=True.
        return {
            0: (3.99, 1.12),
            1: (2.25, 0.67),
            2: (1.06, 0.67),
            3: (0.34, 1.93),
            4: (0.35, 3.01),
            5: (1.52, 4.40),
            6: (2.42, 4.05),
            7: (0.70, 5.70),
            8: (2.70, 6.65),
        }

    def _coord_for_station(self, sid: int) -> Tuple[float, float]:
        # GUI uses one active zone. B-side station IDs are converted to local 1~8
        # unless the JSON explicitly provides global 9~16 coordinates.
        sid = int(sid)
        if sid in self.station_coords:
            return self.station_coords[sid]
        local = sid - 8 if sid >= 9 else sid
        if local in self.station_coords:
            return self.station_coords[local]
        return self._default_station_coords().get(local, (self.map_width_m / 2.0, self.map_height_m / 2.0))

    def _station_short_name(self, sid: int) -> str:
        name = self.state.station_names.get(sid, self.station_coord_names.get(sid, f"station_{sid}"))
        name = name.replace("side_a_", "A_").replace("side_b_", "B_")
        name = name.replace("storage_", "STORAGE_").replace("workbench_", "WB_")
        name = name.replace("customer_", "CUSTOMER_").replace("hybrid_", "HYBRID_")
        return name

    def _active_map_bounds(self) -> Tuple[float, float, float, float, str]:
        """Visible world bounds in meters: x_min, y_min, x_max, y_max, label."""
        mode = self.gui_zone_mode
        mw = max(float(self.map_width_m), 1.0)
        mh = max(float(self.map_height_m), 1.0)

        # In the updated GUI, the normal view is one active zone at its actual
        # zone size. This avoids cutting a side-specific JSON in half.
        if mode in {"zone", "active_side", "full"}:
            side_label = "Side B" if self.side == "b" else "Side A"
            return 0.0, 0.0, mw, mh, f"{side_label} zone"

        # Auto crop: station coordinate bounding box + padding.
        ids = list(self.state.station_items.keys()) if self.state.station_items else [sid for sid in self.station_coords if sid != 0]
        if 0 in self.station_coords:
            ids.append(0)
        pts = []
        for sid in ids:
            try:
                pts.append(self._coord_for_station(int(sid)))
            except Exception:
                pass
        if not pts:
            return 0.0, 0.0, mw, mh, "auto zone"
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        pad_x = max(0.45, mw * 0.04)
        pad_y = max(0.45, mh * 0.04)
        x0 = max(0.0, min(xs) - pad_x)
        x1 = min(mw, max(xs) + pad_x)
        y0 = max(0.0, min(ys) - pad_y)
        y1 = min(mh, max(ys) + pad_y)
        return x0, y0, x1, y1, "auto crop"

    def _cargo_color_and_text(self, obj: int) -> Tuple[str, str, str]:
        oid = int(obj)
        if oid in RAW_COLORS:
            return RAW_COLORS[oid], str(oid), TEXT_DARK
        if oid in BATCH_TO_RAW:
            raw = BATCH_TO_RAW[oid]
            return RAW_COLORS.get(raw, "#eeeeee"), f"B{raw}", TEXT_DARK
        if oid == 90:
            return "#ce93d8", "MIX", TEXT_DARK
        if oid in PRODUCT_DB:
            return PRODUCT_COLOR, str(oid), TEXT_DARK
        return "#eeeeee", str(oid), TEXT_DARK

    def _draw_cargo_cells(self, c: "tk.Canvas", x: float, y: float, items: List[int], *,
                          max_cols: int = 4, cell: int = 16, anchor: str = "center", limit: int = 12) -> None:
        """Draw small LEGO-like cargo cells. (x, y) is the top-left or center depending on anchor."""
        if not self.gui_show_cargo or not items:
            return
        vals = list(items)
        shown = vals[:limit]
        rows = (len(shown) + max_cols - 1) // max_cols
        width = max_cols * cell
        height = max(rows, 1) * cell
        if anchor == "center":
            x0 = x - width / 2.0
            y0 = y - height / 2.0
        else:
            x0, y0 = x, y
        for idx, obj in enumerate(shown):
            col = idx % max_cols
            row = idx // max_cols
            cx0 = x0 + col * cell
            cy0 = y0 + row * cell
            fill, text, fg = self._cargo_color_and_text(int(obj))
            c.create_rectangle(cx0 + 1, cy0 + 1, cx0 + cell - 1, cy0 + cell - 1,
                               fill=fill, outline=BATCH_BORDER if int(obj) in BATCH_TO_RAW else "#333")
            c.create_text(cx0 + cell / 2.0, cy0 + cell / 2.0, text=text, fill=fg, font=("Arial", 7, "bold"))
        if len(vals) > limit:
            c.create_text(x0 + width + 5, y0 + height - cell / 2.0, text=f"+{len(vals)-limit}", anchor="w", font=("Arial", 8, "bold"))

    def _station_display_items(self, sid: int) -> List[object]:
        # Return normal item IDs and batch display tuples: (batch_id, remaining).
        result: List[object] = []
        for obj in self.state.station_items.get(sid, []):
            obj = int(obj)
            if obj in BATCH_TO_RAW:
                result.append((obj, int(self.state.batch_remaining.get((int(sid), obj), 5))))
            else:
                result.append(obj)
        return result

    def _cargo_color_and_text_display(self, obj) -> Tuple[str, str, str, bool]:
        if isinstance(obj, tuple):
            batch_id, remaining = int(obj[0]), int(obj[1])
            raw = BATCH_TO_RAW.get(batch_id, 0)
            return RAW_COLORS.get(raw, "#eeeeee"), f"B{raw}×{remaining}", TEXT_DARK, True
        fill, text, fg = self._cargo_color_and_text(int(obj))
        return fill, text, fg, int(obj) in BATCH_TO_RAW

    def _draw_cargo_cells_display(self, c: "tk.Canvas", x: float, y: float, items: List[object], *,
                                  max_cols: int = 4, cell: int = 16, anchor: str = "center", limit: int = 12) -> None:
        if not self.gui_show_cargo or not items:
            return
        vals = list(items)
        shown = vals[:limit]
        rows = (len(shown) + max_cols - 1) // max_cols
        width = max_cols * cell
        height = max(rows, 1) * cell
        if anchor == "center":
            x0 = x - width / 2.0
            y0 = y - height / 2.0
        else:
            x0, y0 = x, y
        for idx, obj in enumerate(shown):
            col = idx % max_cols
            row = idx // max_cols
            cx0 = x0 + col * cell
            cy0 = y0 + row * cell
            fill, text, fg, is_batch = self._cargo_color_and_text_display(obj)
            c.create_rectangle(cx0 + 1, cy0 + 1, cx0 + cell - 1, cy0 + cell - 1,
                               fill=fill, outline=BATCH_BORDER if is_batch else "#333")
            c.create_text(cx0 + cell / 2.0, cy0 + cell / 2.0, text=text, fill=fg, font=("Arial", 6, "bold"))
        if len(vals) > limit:
            c.create_text(x0 + width + 5, y0 + height - cell / 2.0, text=f"+{len(vals)-limit}", anchor="w", font=("Arial", 8, "bold"))

    def _draw_station_box(self, c: "tk.Canvas", sid: int, rx: float, ry: float) -> None:
        stype = self.state.station_types.get(sid, 0)
        fill = {1: "#2ecc71", 2: "#d86adf", 3: "#ff8a50", 4: "#7bd88f"}.get(stype, "#dddddd")
        items = self._station_display_items(sid)
        half_w, half_h = 42, 31
        c.create_rectangle(rx - half_w, ry - half_h, rx + half_w, ry + half_h, fill=fill, outline="#222", width=1)
        c.create_text(rx, ry - half_h + 9, text=f"S{sid}", font=("Arial", 9, "bold"))
        if items:
            self._draw_cargo_cells_display(c, rx, ry + 8, items, max_cols=3, cell=17, anchor="center", limit=6)
        else:
            c.create_text(rx, ry + 9, text="empty", font=("Arial", 7), fill="#555")

    def _draw_amr_slots_external(self, c: "tk.Canvas", x: float, y: float, max_w: float) -> None:
        """Draw AMR slot/cargo panel outside the arena rectangle when there is room."""
        panel_w = min(max_w, 230)
        if panel_w < 160:
            return
        row_h = 23
        panel_h = 8 * row_h + 30
        c.create_rectangle(x, y, x + panel_w, y + panel_h, fill="#ffffff", outline="#bbbbbb")
        c.create_text(x + 8, y + 14, text="AMR cargo slots", anchor="w", font=("Arial", 10, "bold"))
        for idx, slot in enumerate(range(8)):
            y0 = y + 28 + idx * row_h
            c.create_rectangle(x + 6, y0, x + panel_w - 6, y0 + row_h - 3, outline="#cccccc", fill="#fafafa")
            label = SLOT_LABELS.get(slot, str(slot))
            c.create_text(x + 10, y0 + 9, text=label, anchor="w", font=("Arial", 7))
            objs = [o.object_id for o in self.state.amr_slots.get(slot, [])]
            self._draw_cargo_cells(c, x + 110, y0 + 2, objs, max_cols=5, cell=15, anchor="nw", limit=5)

    def _redraw_arena(self) -> None:
        c = self.canvas
        c.delete("all")
        w = max(c.winfo_width(), 760)
        h = max(c.winfo_height(), 540)

        # Extra left/right/top/bottom room lets labels live outside the actual map border.
        left_margin = 34
        top_margin = 54
        bottom_margin = 46
        right_panel_margin = 260  # AMR cargo panel space outside map
        available_map_w = max(420, w - left_margin - right_panel_margin - 18)
        available_map_h = max(360, h - top_margin - bottom_margin)

        bx0, by0, bx1, by1, zone_label = self._active_map_bounds()
        bw = max(bx1 - bx0, 0.1)
        bh = max(by1 - by0, 0.1)
        scale = min(available_map_w / bw, available_map_h / bh)
        draw_w = bw * scale
        draw_h = bh * scale
        ox = left_margin + (available_map_w - draw_w) / 2.0
        oy = top_margin + (available_map_h - draw_h) / 2.0

        def sx(x: float) -> float:
            return ox + (float(x) - bx0) * scale

        def sy(y: float) -> float:
            if self.gui_invert_y:
                return oy + (by1 - float(y)) * scale
            return oy + (float(y) - by0) * scale

        # Header labels outside map rectangle.
        c.create_text(ox + draw_w / 2.0, oy - 30, text="Customers", font=("Arial", 12, "bold"))
        c.create_text(ox + draw_w / 2.0, oy + draw_h + 30, text="Robot fleets / Start area", font=("Arial", 10))
        side_text = "Side B" if self.side == "b" else "Side A"
        c.create_text(ox + 10, oy + draw_h + 30, text=side_text, anchor="w", font=("Arial", 10, "bold"))
        c.create_text(ox + draw_w - 10, oy + draw_h + 30, text=f"{zone_label} | {bw:.2f}m × {bh:.2f}m", anchor="e", font=("Arial", 8), fill="#555")

        # Arena active-zone outline and metric grid.
        c.create_rectangle(sx(bx0), sy(by0), sx(bx1), sy(by1), outline="#222", width=2)
        start_x = int(bx0) + 1
        end_x = int(bx1)
        for gx in range(start_x, end_x + 1):
            if bx0 < gx < bx1:
                c.create_line(sx(gx), sy(by0), sx(gx), sy(by1), fill="#e0e0e0")
        start_y = int(by0) + 1
        end_y = int(by1)
        for gy in range(start_y, end_y + 1):
            if by0 < gy < by1:
                c.create_line(sx(bx0), sy(gy), sx(bx1), sy(gy), fill="#e0e0e0")

        # Draw a photo-like guide line at the robot-fleet edge only for JSON/full layouts.
        # In one-zone official layout, the whole rectangle already is the active side.
        if self.gui_layout_mode not in {"official", "official_photo", "photo"}:
            mid_x = self.map_width_m / 2.0
            if bx0 < mid_x < bx1:
                c.create_line(sx(mid_x), sy(by0), sx(mid_x), sy(by1), fill="#ff7043", dash=(4, 3), width=2)

        # Simplified official-schematic wall cues. They are cosmetic only.
        if self.gui_layout_mode in {"official", "official_photo", "photo"}:
            wall_blue = "#1e88e5"
            wall_purple = "#8e24aa"
            # bottom/front wall segments
            for xa, xb in [(0.05, 0.85), (1.10, 1.85), (2.05, 2.75), (3.65, 4.75)]:
                c.create_rectangle(sx(xa), sy(0.18)-4, sx(xb), sy(0.18)+4, fill=wall_blue, outline=wall_blue)
            # left wall segments
            for ya, yb in [(0.45, 1.35), (2.85, 3.70), (4.55, 5.35)]:
                c.create_rectangle(sx(0.08)-4, sy(ya), sx(0.08)+4, sy(yb), fill=wall_blue, outline=wall_blue)
            # small purple short walls near WB/storage lanes
            for xw, yw in [(0.20, 5.05), (1.65, 3.55), (2.45, 3.55), (3.10, 2.85), (3.45, 6.95)]:
                c.create_rectangle(sx(xw)-12, sy(yw)-3, sx(xw)+12, sy(yw)+3, fill=wall_purple, outline=wall_purple)

        # Legend outside the map, not over stations.
        legend_x = ox
        legend_y = oy - 14
        legend = [("Storage", "#2ecc71"), ("Workbench", "#d86adf"), ("Customer", "#ff8a50"), ("Hybrid", "#7bd88f"), ("GOAL", "#fff3e0"), ("AMR", "#222222")]
        x_cursor = legend_x
        for txt, color in legend:
            c.create_rectangle(x_cursor, legend_y - 6, x_cursor + 14, legend_y + 6, fill=color, outline="#333")
            c.create_text(x_cursor + 18, legend_y, text=txt, anchor="w", font=("Arial", 8))
            x_cursor += 86

        # Draw GOAL/start coordinate when known and visible.
        if 0 in self.station_coords:
            gx, gy = self._coord_for_station(0)
            if bx0 <= gx <= bx1 and by0 <= gy <= by1:
                grx, gry = sx(gx), sy(gy)
                c.create_rectangle(grx - 40, gry - 28, grx + 40, gry + 28, outline="#ff1744", dash=(3, 2), width=2, fill="#fff8e1")
                c.create_text(grx, gry, text="GOAL\n0", font=("Arial", 8, "bold"), fill="#d50000")

        # Draw stations from task, otherwise from known coords 1..8.
        stations = sorted(self.state.station_items.keys()) if self.state.station_items else [sid for sid in sorted(self.station_coords) if sid != 0]
        for sid in stations:
            if sid == 0:
                continue
            x, y = self._coord_for_station(sid)
            if not (bx0 <= x <= bx1 and by0 <= y <= by1):
                continue
            self._draw_station_box(c, sid, sx(x), sy(y))

        # Draw AMR at virtual current station.
        if self.state.amr_station == 0 and 0 in self.station_coords:
            ax, ay = self.station_coords[0]
        else:
            ax, ay = self._coord_for_station(self.state.amr_station)
        if bx0 <= ax <= bx1 and by0 <= ay <= by1:
            arx, ary = sx(ax), sy(ay)
        else:
            # If AMR is outside current crop, pin it near the closest edge and mark it.
            arx = sx(min(max(ax, bx0), bx1))
            ary = sy(min(max(ay, by0), by1))
        c.create_rectangle(arx - 28, ary - 28, arx + 28, ary + 28, outline="#ff1744", width=2)
        c.create_oval(arx - 14, ary - 14, arx + 14, ary + 14, fill="#212121", outline="white", width=2)
        c.create_text(arx, ary, text="AMR", fill="white", font=("Arial", 7, "bold"))

        # Draw external AMR cargo panel on the right side of the canvas.
        panel_x = ox + draw_w + 18
        panel_y = oy
        self._draw_amr_slots_external(c, panel_x, panel_y, max(0, w - panel_x - 18))

        # Coordinate source footer.
        self.coord_var.set(f"coord: {self.coord_source_path} | visible: x[{bx0:.2f},{bx1:.2f}] y[{by0:.2f},{by1:.2f}] | invert_y={self.gui_invert_y}")

    # ----------------------------- shutdown ---------------------------------

    def destroy(self) -> None:
        try:
            self.destroy_node()
        except Exception:
            pass


def main(args=None) -> None:
    if _TK_IMPORT_ERROR is not None:
        raise RuntimeError(
            "tkinter를 import하지 못했습니다. Ubuntu에서는 `sudo apt install python3-tk`로 설치하세요. "
            f"원인: {_TK_IMPORT_ERROR}"
        )

    rclpy.init(args=args)
    root = tk.Tk()
    node = SmlGuiNode(root)

    def spin_ros():
        try:
            rclpy.spin(node)
        except Exception as exc:
            node.get_logger().error(f"ROS spin error: {exc}")

    spin_thread = threading.Thread(target=spin_ros, daemon=True)
    spin_thread.start()

    def on_close():
        node.destroy()
        rclpy.shutdown()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    try:
        root.mainloop()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()