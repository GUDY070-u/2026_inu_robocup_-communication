"""
RoboCup Planner Node

Subscribes to the task topic, computes the full plan, then runs the
reactive executor in a background thread while the ROS2 node spins
normally in the main thread.

Interfaces:
  Sub  /eai/task              sml_msgs/Task        — task definition
  Sub  <wb_ready_topic>       std_msgs/Int32        — workbench product_id ready
  Act  <nav_action>           sml_msgs/NavTask      — navigate to station
  Act  <wb_action>            sml_msgs/WbTask       — workbench work
  Srv  <arm_service>          arm_interfaces/Cargo  — arm pick/place

Blocking helper methods (navigate, arm_*, wb_task) are called from the
executor thread and use threading.Event to wait for ROS2 async results.
"""

import threading
from typing import Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from sml_msgs.action import NavTask, WbTask
from sml_msgs.msg import Task
from sml_msgs.srv import ArmCommand
from std_msgs.msg import Int32

from robocup_planner.planning.aidlist_builder import compute_net_aidlist
from robocup_planner.planning.cargo_allocator import CargoAllocator
from robocup_planner.planning.distance_calculator import DistanceCalculator
from robocup_planner.planning.midlist_builder import (
    build_full_midlist,
    build_mid,
    check_storage_satisfies,
)
from robocup_planner.execution.executor import Executor, Plan
from robocup_planner.product_catalog import is_intransit_eligible

# Workbench WbTask goal strings
WB_PRODUCE = 'PRODUCE'
WB_RECYCLE = 'RECYCLE'

# Arm Cargo.srv action strings (agree with arm team)
ARM_PICK = 'PICK'
ARM_PLACE = 'PLACE'
ARM_DELIVER = 'DELIVER'


class PlannerNode(Node):

    def __init__(self):
        super().__init__('robocup_planner')

        # --- Parameters ---
        # waypoint_yaml: ament share 디렉토리에서 자동 탐색
        try:
            from ament_index_python.packages import get_package_share_directory
            import os as _os
            _default_wp = _os.path.join(
                get_package_share_directory('robocup_planner'),
                'config',
                'robocup_waypoint.yaml',
            )
        except Exception:
            _default_wp = ''
        self.declare_parameter('waypoint_yaml', _default_wp)
        self.declare_parameter('task_topic', '/sml/task')
        self.declare_parameter('nav_action', 'navigate_to_station')
        self.declare_parameter('wb_action', 'wb_task')
        self.declare_parameter('arm_service', '/amr_robot_command')
        self.declare_parameter('wb_ready_topic', '/workbench/product_ready')
        self.declare_parameter('driving_velocity', 0.5)
        self.declare_parameter('parking_duration', 1.5)
        self.declare_parameter('exiting_duration', 1.0)

        wp_path = self.get_parameter('waypoint_yaml').get_parameter_value().string_value
        task_topic = self.get_parameter('task_topic').get_parameter_value().string_value
        nav_action = self.get_parameter('nav_action').get_parameter_value().string_value
        wb_action = self.get_parameter('wb_action').get_parameter_value().string_value
        arm_service = self.get_parameter('arm_service').get_parameter_value().string_value
        wb_ready_topic = self.get_parameter('wb_ready_topic').get_parameter_value().string_value

        if not wp_path:
            self.get_logger().warning("waypoint_yaml parameter is empty; distances will be inf")
            self._calc: Optional[DistanceCalculator] = None
        else:
            self._calc = DistanceCalculator(wp_path)

        # --- ROS interfaces ---
        self._task_sub = self.create_subscription(
            Task, task_topic, self._on_task, 10
        )
        self._wb_ready_sub = self.create_subscription(
            Int32, wb_ready_topic, self._on_wb_ready, 10
        )
        self._nav_client = ActionClient(self, NavTask, nav_action)
        self._wb_client = ActionClient(self, WbTask, wb_action)
        self._arm_client = self.create_client(ArmCommand, arm_service)

        # Active executor (one at a time)
        self._executor_thread: Optional[threading.Thread] = None
        self._active_executor: Optional[Executor] = None
        self._exec_lock = threading.Lock()

        self.get_logger().info("RoboCup Planner ready — waiting for task")

    # ------------------------------------------------------------------
    # Task callback — triggers planning + execution
    # ------------------------------------------------------------------

    def _on_task(self, msg: Task) -> None:
        with self._exec_lock:
            if self._executor_thread and self._executor_thread.is_alive():
                self.get_logger().warning(
                    "New task received while execution is running — ignoring"
                )
                return

        self.get_logger().info("Task received — planning...")
        try:
            plan = self._plan(msg)
        except Exception as e:
            self.get_logger().error(f"Planning failed: {e}")
            return

        executor = Executor(plan, self)
        self._active_executor = executor

        thread = threading.Thread(target=executor.run, daemon=True, name='executor')
        self._executor_thread = thread
        thread.start()

    # ------------------------------------------------------------------
    # Workbench ready signal — sets the executor's event
    # ------------------------------------------------------------------

    def _on_wb_ready(self, msg: Int32) -> None:
        self.get_logger().info(
            f"Workbench signal: product {msg.data} ready"
        )
        if self._active_executor is not None:
            self._active_executor.wb_signal.set()

    # ------------------------------------------------------------------
    # Planning phase
    # ------------------------------------------------------------------

    # batch ID(10,20,...,80) → raw material ID(1,2,...,8) 변환표
    # order_server가 arena_layout에 배치 ID를 넣으므로 raw ID로 풀어야 함
    _BATCH_TO_RAW: dict = {
        10: 1, 20: 2, 30: 3, 40: 4,
        50: 5, 60: 6, 70: 7, 80: 8,
    }

    @staticmethod
    def _resolve_material_ids(raw_ids) -> list:
        """배치 ID가 섞여 있어도 raw material ID 목록으로 변환한다."""
        result = []
        for mid in raw_ids:
            mid = int(mid)
            resolved = PlannerNode._BATCH_TO_RAW.get(mid, mid)
            result.append(resolved)
        return result

    def _plan(self, msg: Task) -> Plan:
        from sml_msgs.msg import Station as StationMsg

        # Parse orders
        produce_ids = [o.product_id for o in msg.order_list if o.order_type == 1]
        recycle_ids = [o.product_id for o in msg.order_list if o.order_type == 2]

        self.get_logger().info(
            f"Plan: produce={produce_ids}, recycle={recycle_ids}"
        )

        # Categorise stations
        storage_stations = []
        workbench_station_id = None
        customer_station_id = None
        customer_stations = []

        for st in msg.arena_layout:
            if st.station_type in (StationMsg.ST_STORAGE, StationMsg.ST_HYBRID):
                # 버그 1 수정: 배치 ID → raw material ID 변환
                raw_mat_ids = self._resolve_material_ids(st.material_ids)
                storage_stations.append({
                    'station_id': st.station_id,
                    'material_ids': raw_mat_ids,
                })
                self.get_logger().info(
                    f"Station {st.station_id}: raw material_ids={raw_mat_ids} "
                    f"(original={list(st.material_ids)})"
                )
            if st.station_type in (StationMsg.ST_WORKBENCH, StationMsg.ST_HYBRID):
                if workbench_station_id is None:
                    workbench_station_id = st.station_id
            if st.station_type == StationMsg.ST_CUSTOMER:
                customer_station_id = st.station_id
                customer_stations.append(st)

        if workbench_station_id is None:
            raise RuntimeError("No workbench station in arena layout")
        if customer_station_id is None:
            raise RuntimeError("No customer station in arena layout")

        home_id = 0

        # Compute aidlist and net_aidlist
        aidlist, net_aidlist, recycled_materials = compute_net_aidlist(
            produce_ids, recycle_ids
        )
        self.get_logger().info(
            f"aidlist={dict(aidlist)}, net_aidlist={dict(net_aidlist)}"
        )

        # Build storage midlist from home to check availability
        if self._calc:
            from robocup_planner.planning.midlist_builder import build_storage_midlist
            storage_mid = build_storage_midlist(storage_stations, self._calc, home_id)
        else:
            storage_mid = [
                {'station_id': s['station_id'], 'materials': s['material_ids'],
                 'distance': 0.0, 'is_recycle_pickup': False, 'recycle_product_id': None}
                for s in storage_stations
            ]

        satisfied, missing = check_storage_satisfies(storage_mid, net_aidlist)
        needs_recycling = bool(recycle_ids)

        if not satisfied and not recycle_ids:
            self.get_logger().warning(
                f"Cannot satisfy aidlist even with recycling — missing: {dict(missing)}"
            )

        # Build recycle orders (map each recycle product to the customer station)
        recycle_orders = [
            {'station_id': customer_station_id, 'product_id': pid}
            for pid in recycle_ids
        ]

        # Build full midlist
        if self._calc:
            full_midlist = build_full_midlist(
                storage_stations=storage_stations,
                customer_stations=[],
                recycle_orders=recycle_orders,
                calc=self._calc,
                home_station_id=home_id,
                workbench_station_id=workbench_station_id,
                needs_recycling=needs_recycling,
            )
        else:
            full_midlist = [
                {'station_id': o['station_id'], 'materials': [],
                 'distance': 0.0, 'is_recycle_pickup': True,
                 'recycle_product_id': o['product_id']}
                for o in recycle_orders
            ] + storage_mid

        # Build final mid list
        mid = build_mid(full_midlist, net_aidlist)

        # 버그 2 수정: 인트랜짓 비활성화 — 모든 제품을 워크벤치로 처리
        # (인트랜짓은 픽업 순서와 build_order가 일치해야 하는 미구현 제약이 있음)
        intransit_ids: list = []
        workbench_ids = list(produce_ids)

        # Surplus recycled materials (obtained beyond what net_aidlist needs)
        surplus = {}
        for mat, cnt in recycled_materials.items():
            extra = cnt - (aidlist.get(mat, 0))
            if extra > 0:
                surplus[mat] = extra

        plan = Plan(
            mid=mid,
            workbench_products=workbench_ids,
            intransit_products=intransit_ids,
            workbench_station_id=workbench_station_id,
            customer_station_id=customer_station_id,
            home_station_id=home_id,
            surplus_recycled=surplus,
        )

        self.get_logger().info(
            f"Plan ready: {len(mid)} pickup entries, "
            f"workbench={workbench_ids}, in-transit={intransit_ids}"
        )
        return plan

    # ------------------------------------------------------------------
    # Blocking helpers called by Executor (run in executor thread)
    # ------------------------------------------------------------------

    def navigate(self, station_id: int) -> bool:
        """Block until the AMR reaches station_id. Returns True on success."""
        self._nav_client.wait_for_server()

        done = threading.Event()
        success_holder = [False]

        def _result_cb(future):
            result = future.result()
            success_holder[0] = result.result.success
            done.set()

        def _goal_cb(future):
            gh = future.result()
            if not gh.accepted:
                self.get_logger().error(f"NavTask goal rejected for station {station_id}")
                done.set()
                return
            gh.get_result_async().add_done_callback(_result_cb)

        goal = NavTask.Goal()
        goal.station_id = station_id
        self._nav_client.send_goal_async(goal).add_done_callback(_goal_cb)
        done.wait()

        if not success_holder[0]:
            self.get_logger().error(f"Navigation to station {station_id} failed")
        return success_holder[0]

    def wb_task(self, work_type: str, product_id: int) -> bool:
        """Block until the workbench completes the requested work."""
        self._wb_client.wait_for_server()

        done = threading.Event()
        success_holder = [False]

        def _result_cb(future):
            success_holder[0] = future.result().result.success
            done.set()

        def _goal_cb(future):
            gh = future.result()
            if not gh.accepted:
                self.get_logger().error(f"WbTask goal rejected ({work_type} {product_id})")
                done.set()
                return
            gh.get_result_async().add_done_callback(_result_cb)

        goal = WbTask.Goal()
        goal.work_type = work_type
        goal.product_id = product_id
        self._wb_client.send_goal_async(goal).add_done_callback(_goal_cb)
        done.wait()
        return success_holder[0]

    def _arm_call(
        self,
        action: str,
        object_ids: list,
        location: int = 0,
        slide_ids: list = None,
    ) -> bool:
        """Send one ArmCommand service call to the arm. Blocks until response."""
        self._arm_client.wait_for_service()

        req = ArmCommand.Request()
        req.action = action
        req.object_ids = [int(x) for x in object_ids]
        req.location = int(location)
        req.slide_ids = [int(x) for x in (slide_ids or [])]

        future = self._arm_client.call_async(req)
        done = threading.Event()

        def _cb(f):
            done.set()

        future.add_done_callback(_cb)
        done.wait()
        return future.result().success

    def arm_pick_material(
        self, station_id: int, material_id: int, manipulator_slot: int
    ) -> bool:
        """Pick one material block from a storage station and place it on cargo."""
        # action=LOAD, object_ids=[material_id], location=station_id,
        # slide_ids=[manipulator_slot] (cargo slot to place the block on)
        return self._arm_call(
            ARM_PICK,
            object_ids=[material_id],
            location=station_id,
            slide_ids=[manipulator_slot],
        )

    def arm_pick_product(self, station_id: int, product_id: int) -> bool:
        """Pick an assembled product from a customer counter (for recycling)."""
        return self._arm_call(
            ARM_PICK,
            object_ids=[product_id],
            location=station_id,
            slide_ids=[0],
        )

    def arm_unload_material(self, cargo_id: int, placement_idx: int) -> bool:
        """Unload a material block from cargo (drop at workbench)."""
        slot_value = cargo_id * 10 + placement_idx
        return self._arm_call(
            ARM_PLACE,
            object_ids=[0],
            location=0,
            slide_ids=[slot_value],
        )

    def arm_deliver(self, from_cargo_id: int) -> bool:
        """Deliver a finished product from cargo to the customer counter."""
        return self._arm_call(
            ARM_DELIVER,
            object_ids=[0],
            location=0,
            slide_ids=[from_cargo_id],
        )


# ------------------------------------------------------------------
# Entry point
# ------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()

    # MultiThreadedExecutor lets action/service callbacks run while
    # the executor thread is blocking inside navigate() / wb_task().
    ros_executor = MultiThreadedExecutor(num_threads=4)
    ros_executor.add_node(node)

    try:
        ros_executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
