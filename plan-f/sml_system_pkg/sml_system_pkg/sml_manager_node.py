"""
sml_manager_node.py
GetPlan 서비스로 스텝 목록을 받아
depends_on 기반으로 AMR / WB를 병렬 실행하는 노드.

A/B 경기장 대응:
  - side:=a 또는 side:=b 파라미터 사용
  - 일반 station은 Step.station_id를 그대로 AMR에 전달
  - GOAL/복귀 station_id는 A면 0, B면 9를 사용
"""

import threading
import time
from collections import deque

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from sml_msgs.action import NavTask, WbTask
from sml_msgs.msg import Step, Task
from sml_msgs.srv import ArmCommand, GetPlan

from sml_system_pkg.arena_side_utils import (
    normalize_side,
    nav_target_for_station,
    side_to_fixed_workbench_station,
)
from sml_system_pkg.planning.planner_config import RAW_SLOT_INDICES


ARM_ACTION_ASSEMBLE = 'ASSEMBLE'


class SmlManagerNode(Node):

    def __init__(self):
        super().__init__('sml_manager_node')
        self.cbg = ReentrantCallbackGroup()

        # ── 실행 상태 ──────────────────────────────────────
        self._lock = threading.Lock()
        self.pending_steps   = []       # 아직 실행 안 된 스텝
        self.completed_steps = set()    # 완료된 step_id 집합
        self.amr_busy        = False    # AMR 트랙 점유 여부
        self.wb_busy         = False    # WB 트랙 점유 여부
        self.wb_reserved_by_amr = None  # WB 접근을 예약한 AMR step_id
        self.plan_requested  = False    # GetPlan 요청 여부 (중복 방지)

        # AMR ASSEMBLE은 이동과 AMR 내부 조립이 동시에 진행된다.
        # step 완료 조건은 "NAV 도착 완료 AND AMR 조립 완료"이다.
        self._amr_assemble_states = {}

        # Plan D ARM command slot conversion state.
        # Planner Step.slide_ids keep logical ids such as order_index*10+slot_index.
        # The physical AMR arm expects raw-slide ids in the form
        #     raw_slide_no * 10 + pick_position
        # for raw slides. Product/assembly slots are sent as their physical slot number.
        self._arm_raw_slot_indices = list(RAW_SLOT_INDICES)
        self._arm_raw_slots = {
            slot: {'units': [None, None, None], 'items': {}}
            for slot in self._arm_raw_slot_indices
        }
        self._arm_item_keys = {}             # (logical_slide_id, object_id) -> deque(uid)
        self._arm_next_uid = 1
        self._arm_cmd_slide_cache = {}       # step_id -> converted slide ids used for retries
        self._arm_pending_removals = {}      # step_id -> [uid, ...] to clear after success

        # step 소요 시간 측정 관련
        self.step_start_times       = {}     # step_id -> time.monotonic() 시작 시각
        self.step_elapsed_times     = {}     # step_id -> 완료까지 걸린 시간 [s]
        self.step_records           = {}     # step_id -> 로그 요약용 metadata
        self.plan_start_time        = None   # 전체 실행 시작 시각
        self.plan_end_time          = None   # 전체 실행 종료 시각
        self._duration_summary_done = False  # 최종 요약 중복 출력 방지

        # GetPlan 재시도 관련
        self._plan_retry_count = 0
        self._plan_timer       = None
        self._max_plan_retries = 10

        self.declare_parameter('side', 'a')
        self.side = normalize_side(self.get_parameter('side').value)
        self.fixed_workbench_station = side_to_fixed_workbench_station(self.side)

        self.declare_parameter(
            'post_process_service_name',
            '/robocup_navigator/post_process',
        )

        # ── Subscriber ─────────────────────────────────────
        self.declare_parameter('task_topic', '/sml/task')
        task_topic = self.get_parameter('task_topic').value

        self.task_sub = self.create_subscription(
            Task, task_topic,
            self.task_callback, 10,
            callback_group=self.cbg)

        # ── Service Clients ────────────────────────────────
        self.get_plan_client = self.create_client(
            GetPlan, '/sml/get_plan',
            callback_group=self.cbg)
        self.arm_client = self.create_client(
            ArmCommand, '/amr_robot_command',
            callback_group=self.cbg)
        self.post_process_client = self.create_client(
            Trigger,
            self.get_parameter('post_process_service_name').value,
            callback_group=self.cbg)

        # ── Action Clients ─────────────────────────────────
        self.nav_client = ActionClient(
            self, NavTask, 'navigate_to_station',
            callback_group=self.cbg)
        self.wb_client = ActionClient(
            self, WbTask, 'wb_task',
            callback_group=self.cbg)

        # ── Status Publisher ───────────────────────────────
        self.status_pub = self.create_publisher(
            String, '/sml/status', 10)

        self.get_logger().info(
            f'[MANAGER] sml_manager_node 시작 | task_topic={task_topic} | side={self.side}'
        )

    # ──────────────────────────────────────────────────────
    # Task 수신 → GetPlan 요청
    # ──────────────────────────────────────────────────────

    def task_callback(self, msg):
        with self._lock:
            if self.plan_requested:
                return
            self.plan_requested = True

        self.get_logger().info('[MANAGER] Task 수신 → 1초 후 GetPlan 요청')
        self._plan_retry_count = 0
        self._plan_timer = self.create_timer(1.0, self._try_get_plan)

    def _try_get_plan(self):
        if self._plan_timer:
            self._plan_timer.cancel()
            self._plan_timer = None

        if not self.get_plan_client.wait_for_service(timeout_sec=1.0):
            self._retry_get_plan('GetPlan 서비스 없음')
            return

        future = self.get_plan_client.call_async(GetPlan.Request())
        future.add_done_callback(self._on_get_plan_response)

    def _on_get_plan_response(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f'[MANAGER] GetPlan 호출 예외: {e}')
            self._retry_get_plan('GetPlan 호출 예외')
            return

        if not response.success:
            self._retry_get_plan('계획 미생성')
            return

        self.get_logger().info(
            f'[MANAGER] 계획 수신 완료: {len(response.steps)}개 스텝')
        self._log_steps(response.steps)

        with self._lock:
            self.pending_steps = list(response.steps)
            self.completed_steps.clear()
            self.amr_busy = False
            self.wb_busy = False
            self.wb_reserved_by_amr = None

            # 새 계획 기준으로 시간 측정값 초기화
            self.step_start_times.clear()
            self.step_elapsed_times.clear()
            self.step_records.clear()
            self.plan_start_time = time.monotonic()
            self.plan_end_time = None
            self._duration_summary_done = False

            for step in response.steps:
                self.step_records[int(step.step_id)] = self._make_step_record(step)

        self._dispatch()

    def _retry_get_plan(self, reason):
        self._plan_retry_count += 1
        if self._plan_retry_count <= self._max_plan_retries:
            self.get_logger().warn(
                f'[MANAGER] {reason}, 재시도 '
                f'({self._plan_retry_count}/{self._max_plan_retries})')
            self._plan_timer = self.create_timer(0.5, self._try_get_plan)
            return

        self.get_logger().error('[MANAGER] GetPlan 최대 재시도 초과')
        with self._lock:
            self.plan_requested = False

    # ──────────────────────────────────────────────────────
    # 스텝 디스패치
    # ──────────────────────────────────────────────────────

    def _dispatch(self):
        """ready 스텝을 찾아 AMR / WB 트랙에 각각 1개씩 실행."""
        amr_step = None
        wb_step  = None

        with self._lock:
            for step in list(self.pending_steps):
                deps_ok = all(
                    d in self.completed_steps
                    for d in step.depends_on)
                if not deps_ok:
                    continue

                if step.type == Step.AMR and not self.amr_busy \
                        and amr_step is None:
                    needs_wb = self._is_amr_wb_interaction(step)
                    if needs_wb and (
                        self.wb_busy or self.wb_reserved_by_amr is not None
                    ):
                        continue
                    self.amr_busy = True
                    if needs_wb:
                        self.wb_reserved_by_amr = int(step.step_id)
                    self.pending_steps.remove(step)
                    amr_step = step

                elif step.type == Step.WB and not self.wb_busy \
                        and self.wb_reserved_by_amr is None \
                        and wb_step is None:
                    self.wb_busy = True
                    self.pending_steps.remove(step)
                    wb_step = step

                if amr_step and wb_step:
                    break

            remaining = len(self.pending_steps)
            all_done  = (remaining == 0
                         and not self.amr_busy and not self.wb_busy
                         and amr_step is None and wb_step is None)

            should_log_all_done = all_done and not self._duration_summary_done
            if should_log_all_done:
                self._duration_summary_done = True
                self.plan_end_time = time.monotonic()

        if amr_step:
            self._mark_step_started(amr_step)
            self.get_logger().info(
                f'[MANAGER] AMR step {amr_step.step_id} 시작 '
                f'(action={amr_step.action}, '
                f'objects={list(amr_step.object_ids)}, '
                f'station={amr_step.station_id})')
            self._publish_status(
                f'AMR step {amr_step.step_id} 실행 중')
            self._execute_amr(amr_step)

        if wb_step:
            self._mark_step_started(wb_step)
            self.get_logger().info(
                f'[MANAGER] WB step {wb_step.step_id} 시작 '
                f'(action={wb_step.action}, '
                f'objects={list(wb_step.object_ids)})')
            self._publish_status(
                f'WB step {wb_step.step_id} 실행 중')
            self._execute_wb(wb_step)

        if should_log_all_done:
            self.get_logger().info('[MANAGER] ✅ 모든 스텝 완료!')
            self._log_step_duration_summary()
            self._publish_status('완료')

    def _is_amr_wb_interaction(self, step):
        return (
            int(step.type) == int(Step.AMR)
            and int(step.action) in (int(Step.LOAD), int(Step.UNLOAD))
            and int(step.station_id) == int(self.fixed_workbench_station)
        )

    def _set_amr_idle(self, step):
        """Release the AMR track and any WB access reservation held by this step."""
        with self._lock:
            self.amr_busy = False
            if self.wb_reserved_by_amr == int(step.step_id):
                self.wb_reserved_by_amr = None

    def _on_step_complete(self, step_id):
        with self._lock:
            now = time.monotonic()
            start_time = self.step_start_times.get(step_id)
            elapsed = None
            if start_time is not None:
                elapsed = now - start_time
                self.step_elapsed_times[step_id] = elapsed

            self.completed_steps.add(step_id)
            completed = sorted(self.completed_steps)
            remaining = len(self.pending_steps)

        if elapsed is None:
            self.get_logger().warn(
                f'[TIME] step {step_id} 시작 시간이 없어 소요 시간을 계산할 수 없습니다')
        else:
            self.get_logger().info(
                f'[TIME] step {step_id} 소요 시간: {elapsed:.2f}s')

        self.get_logger().info(
            f'[MANAGER] step {step_id} 완료 '
            f'| 완료: {completed} '
            f'| 남은 스텝: {remaining}개')

        # 소요 시간 로그를 먼저 남긴 뒤 다음 ready step을 실행한다.
        self._dispatch()

    # ──────────────────────────────────────────────────────
    # AMR 스텝 실행
    #
    # 일반 LOAD/UNLOAD:
    #   NAV Action 완료 → ARM Service 실행 → post_process → step 완료
    #
    # AMR ASSEMBLE:
    #   NAV Action과 AMR 내부 조립 Service를 동시에 시작한다.
    #   NAV 도착과 조립 완료가 모두 끝난 뒤 post_process를 수행하고 step 완료 처리한다.
    # ──────────────────────────────────────────────────────

    def _assign_nav_goal_target(self, goal, station_id: int) -> str:
        """
        navigator goal에 target을 넣는다.

        - A면/B면 start/goal을 포함해 숫자 station_id를 그대로 사용한다.
        - NavTask.Goal.station_id가 string 타입이면 숫자를 문자열로 변환한다.
        - goal에 location/target/station_name 같은 string 필드가 있으면 함께 채운다.
        """
        nav_target = nav_target_for_station(int(station_id), self.side)
        field_types = goal.get_fields_and_field_types()

        # 보조 문자열 필드가 존재하면 채움
        for string_field in ('location', 'target', 'station_name', 'station_label'):
            if string_field in field_types and field_types[string_field] == 'string':
                setattr(goal, string_field, str(nav_target))

        if 'station_id' in field_types:
            station_id_type = field_types['station_id']

            if station_id_type == 'string':
                goal.station_id = str(nav_target)
                return nav_target

            goal.station_id = int(nav_target)
            return nav_target

        # station_id 필드가 없고 target/location만 있는 경우
        return nav_target

    def _execute_amr(self, step, retry=0):
        MAX_RETRY = 1

        # Plan C:
        # Step.PRODUCE enum은 메시지 호환용이며, AMR에서는 ASSEMBLE을 뜻한다.
        # 기존 LOAD/UNLOAD처럼 NAV 완료 후 ARM을 실행하면 이동 중 조립이 되지 않으므로
        # 별도 경로에서 NAV와 ASSEMBLE service를 동시에 시작한다.
        if step.action == Step.PRODUCE:
            self._execute_amr_assemble(step, retry=retry)
            return

        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(
                f'[NAV] step {step.step_id}: nav 서버 없음')
            self._set_amr_idle(step)
            return

        goal = NavTask.Goal()
        nav_target = self._assign_nav_goal_target(goal, int(step.station_id))

        self.get_logger().info(
            f'[NAV] step {step.step_id} → '
            f'station_id={step.station_id}, nav_target={nav_target} 이동')

        send_future = self.nav_client.send_goal_async(goal)
        send_future.add_done_callback(
            lambda f, s=step, r=retry: self._on_nav_accepted(f, s, r))

    def _on_nav_accepted(self, future, step, retry):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(
                f'[NAV] step {step.step_id} goal 거절됨')
            self._set_amr_idle(step)
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f, s=step, r=retry: self._on_nav_result(f, s, r))

    def _on_nav_result(self, future, step, retry):
        MAX_RETRY = 1
        result = future.result().result

        if not result.success:
            self.get_logger().error(
                f'[NAV] step {step.step_id} 실패: {result.fail_reason}')
            if retry < MAX_RETRY and result.fail_reason == 'NAV_FAILED':
                self.get_logger().warn(
                    f'[NAV] step {step.step_id} 재시도 ({retry+1}/{MAX_RETRY})')
                self._execute_amr(step, retry + 1)
            else:
                self.get_logger().error(
                    f'[NAV] step {step.step_id} 최종 실패')
                self._set_amr_idle(step)
            return

        self.get_logger().info(
            f'[NAV] step {step.step_id} 도착 완료')

        if step.action == Step.GOAL:
            self.get_logger().info(
                f'[NAV] step {step.step_id} GOAL 도착 → ARM 생략, 완료 처리')
            self._set_amr_idle(step)
            self._on_step_complete(step.step_id)
            return

        self.get_logger().info(f'[NAV] step {step.step_id} → ARM 실행')
        self._execute_arm(step)

    # ──────────────────────────────────────────────────────
    # AMR ASSEMBLE
    # ──────────────────────────────────────────────────────

    def _execute_amr_assemble(self, step, retry=0):
        """
        Step 메시지의 PRODUCE enum으로 전달되는 AMR ASSEMBLE 처리.

        의미:
            - step.station_id로 이동하면서 AMR 내부 조립공간에서 product_id를 조립한다.
            - NAV 도착과 조립 완료가 모두 끝나야 step 완료로 본다.
            - 이동이 먼저 끝나면 목적지에서 조립 완료까지 대기한다.
            - 조립이 먼저 끝나면 목적지 도착까지 대기한다.
        """
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(
                f'[AMR ASSEMBLE] step {step.step_id}: nav 서버 없음')
            self._set_amr_idle(step)
            return

        if not self.arm_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(
                f'[AMR ASSEMBLE] step {step.step_id}: arm 서비스 없음')
            self._set_amr_idle(step)
            return

        goal = NavTask.Goal()
        nav_target = self._assign_nav_goal_target(goal, int(step.station_id))

        with self._lock:
            self._amr_assemble_states[int(step.step_id)] = {
                'nav_done': False,
                'arm_done': False,
                'failed': False,
                'post_started': False,
                'nav_target': nav_target,
            }

        self.get_logger().info(
            f'[AMR ASSEMBLE] step {step.step_id} 시작 | '
            f'product={list(step.object_ids)} | '
            f'station_id={step.station_id}, nav_target={nav_target} | '
            'NAV와 AMR 조립을 동시에 실행'
        )

        # 1) NAV 시작
        self._send_amr_assemble_nav(step, goal, retry=retry)

        # 2) AMR 내부 조립 시작
        self._send_amr_assemble_arm(step, nav_target, retry=0)

    def _send_amr_assemble_nav(self, step, goal=None, retry=0):
        if goal is None:
            goal = NavTask.Goal()
            nav_target = self._assign_nav_goal_target(goal, int(step.station_id))
        else:
            nav_target = self._get_amr_assemble_nav_target(step)

        self.get_logger().info(
            f'[NAV/ASSEMBLE] step {step.step_id} → '
            f'station_id={step.station_id}, nav_target={nav_target} 이동 시작'
        )

        send_future = self.nav_client.send_goal_async(goal)
        send_future.add_done_callback(
            lambda f, s=step, r=retry: self._on_amr_assemble_nav_accepted(f, s, r))

    # ──────────────────────────────────────────────────────
    # Plan D ARM slot conversion
    # ──────────────────────────────────────────────────────

    def _logical_slot_index(self, slide_id):
        sid = int(slide_id)
        return abs(sid) % 10

    def _is_raw_slot_index(self, slot_index):
        return int(slot_index) in self._arm_raw_slot_indices

    def _raw_slide_no_from_slot(self, slot_index):
        # Physical AMR raw slides are numbered 1~5, while planner slot_index is 2~6.
        return self._arm_raw_slot_indices.index(int(slot_index)) + 1

    def _raw_size_for_arm_slot(self, object_id):
        # Current official Plan D rule used by the planner:
        # raw 1~4: size 1, raw 5~8: size 2.
        # If the real hardware size table changes, update only this function or load it from YAML.
        try:
            raw = int(object_id)
        except Exception:
            return 1
        return 1 if 1 <= raw <= 4 else 2

    def _new_arm_uid(self):
        uid = self._arm_next_uid
        self._arm_next_uid += 1
        return uid

    def _find_free_raw_units(self, slot_index, size):
        slot_state = self._arm_raw_slots.setdefault(
            int(slot_index), {'units': [None, None, None], 'items': {}}
        )
        units = slot_state['units']
        size = int(size)
        if size <= 1:
            for unit_idx, used_by in enumerate(units):
                if used_by is None:
                    return [unit_idx]
            return None
        # size 2 object occupies two consecutive unit cells.
        for unit_idx in (0, 1):
            if units[unit_idx] is None and units[unit_idx + 1] is None:
                return [unit_idx, unit_idx + 1]
        return None

    def _pick_position_from_units(self, occupied_units):
        # Unit layout: [0], [1], [2]
        # size 1 centers: 0, 2, 4
        # size 2 centers: 1 or 3
        if len(occupied_units) == 1:
            return int(occupied_units[0]) * 2
        return int(min(occupied_units)) * 2 + 1

    def _assign_raw_arm_position(self, logical_slide_id, object_id):
        logical_slide_id = int(logical_slide_id)
        object_id = int(object_id) if object_id is not None else 0
        slot_index = self._logical_slot_index(logical_slide_id)
        slide_no = self._raw_slide_no_from_slot(slot_index)
        size = self._raw_size_for_arm_slot(object_id)
        occupied_units = self._find_free_raw_units(slot_index, size)
        if occupied_units is None:
            self.get_logger().warn(
                f'[ARM SLOT] raw slot {slot_index}에 object={object_id}, size={size}를 넣을 공간이 없습니다. '
                f'fallback으로 slide_no*10 사용'
            )
            return slide_no * 10

        uid = self._new_arm_uid()
        pos = self._pick_position_from_units(occupied_units)
        arm_slide_id = slide_no * 10 + pos
        slot_state = self._arm_raw_slots[slot_index]
        for unit_idx in occupied_units:
            slot_state['units'][unit_idx] = uid
        slot_state['items'][uid] = {
            'uid': uid,
            'logical_slide_id': logical_slide_id,
            'object_id': object_id,
            'slot_index': slot_index,
            'slide_no': slide_no,
            'position': pos,
            'arm_slide_id': arm_slide_id,
            'units': list(occupied_units),
        }
        key = (logical_slide_id, object_id)
        self._arm_item_keys.setdefault(key, deque()).append(uid)

        self.get_logger().info(
            f'[ARM SLOT] assign logical_slide={logical_slide_id}, object={object_id} '
            f'-> raw_slide={slide_no}, pos={pos}, arm_slide_id={arm_slide_id}'
        )
        return arm_slide_id

    def _lookup_raw_arm_position(self, logical_slide_id, object_id=None, remove_after_success=False, step_id=None):
        logical_slide_id = int(logical_slide_id)
        candidate_uid = None

        if object_id is not None:
            key = (logical_slide_id, int(object_id))
            queue = self._arm_item_keys.get(key)
            if queue:
                candidate_uid = queue[0]

        if candidate_uid is None:
            # Fallback for AMR ASSEMBLE where object_ids contain only the product id
            # while slide_ids contain all recipe slots.
            for (sid, _obj), queue in self._arm_item_keys.items():
                if sid == logical_slide_id and queue:
                    candidate_uid = queue[0]
                    break

        if candidate_uid is None:
            slot_index = self._logical_slot_index(logical_slide_id)
            slide_no = self._raw_slide_no_from_slot(slot_index)
            self.get_logger().warn(
                f'[ARM SLOT] logical_slide={logical_slide_id}, object={object_id}에 대한 기존 적재 정보를 찾지 못했습니다. '
                f'fallback으로 raw_slide={slide_no}, pos=0 사용'
            )
            return slide_no * 10

        slot_index = self._logical_slot_index(logical_slide_id)
        entry = self._arm_raw_slots[slot_index]['items'].get(candidate_uid)
        if entry is None:
            slide_no = self._raw_slide_no_from_slot(slot_index)
            return slide_no * 10

        if remove_after_success and step_id is not None:
            self._arm_pending_removals.setdefault(int(step_id), []).append(candidate_uid)
        return int(entry['arm_slide_id'])

    def _commit_arm_slot_removals(self, step):
        step_id = int(step.step_id)
        uids = self._arm_pending_removals.pop(step_id, [])
        if not uids:
            return
        for uid in uids:
            for slot_index, slot_state in self._arm_raw_slots.items():
                entry = slot_state['items'].pop(uid, None)
                if entry is None:
                    continue
                for unit_idx in entry.get('units', []):
                    if 0 <= unit_idx < len(slot_state['units']) and slot_state['units'][unit_idx] == uid:
                        slot_state['units'][unit_idx] = None
                key = (int(entry['logical_slide_id']), int(entry['object_id']))
                queue = self._arm_item_keys.get(key)
                if queue and uid in queue:
                    try:
                        queue.remove(uid)
                    except ValueError:
                        pass
                    if not queue:
                        self._arm_item_keys.pop(key, None)
                self.get_logger().info(
                    f'[ARM SLOT] release logical_slide={entry["logical_slide_id"]}, '
                    f'object={entry["object_id"]}, arm_slide_id={entry["arm_slide_id"]}'
                )
                break

    def _commit_amr_assemble_slot_changes(self, step):
        """
        ASSEMBLE이 성공하면 첫 번째 slide의 기준 재료를 완성품으로 교체한다.

        조립 결과물은 planner가 지정한 첫 번째 slide(assembly/base 위치)에
        그대로 남는다. 이 위치가 raw slide의 세부 위치로 변환된 경우에도
        기존 arm_slide_id와 점유 unit을 보존해야 다음 UNLOAD가 같은 위치를
        사용한다. 나머지 ASSEMBLE 입력은 기존 방식대로 해제한다.
        """
        step_id = int(step.step_id)
        logical_slide_ids = list(getattr(step, 'slide_ids', []))
        product_ids = list(getattr(step, 'object_ids', []))
        pending_uids = self._arm_pending_removals.get(step_id, [])

        if not logical_slide_ids or not product_ids or not pending_uids:
            self._commit_arm_slot_removals(step)
            return

        output_slide_id = int(logical_slide_ids[0])
        product_id = int(product_ids[0])
        replacement_uid = None

        # pending_uids에는 이번 ASSEMBLE 명령에서 실제로 참조한 항목만 있다.
        # 같은 logical slide에 여러 물체가 있어도 정확히 사용한 항목을 승계한다.
        for uid in pending_uids:
            for slot_state in self._arm_raw_slots.values():
                entry = slot_state['items'].get(uid)
                if entry is None or int(entry['logical_slide_id']) != output_slide_id:
                    continue

                old_object_id = int(entry['object_id'])
                old_key = (output_slide_id, old_object_id)
                old_queue = self._arm_item_keys.get(old_key)
                if old_queue and uid in old_queue:
                    old_queue.remove(uid)
                    if not old_queue:
                        self._arm_item_keys.pop(old_key, None)

                entry['object_id'] = product_id
                self._arm_item_keys.setdefault(
                    (output_slide_id, product_id), deque()
                ).append(uid)
                replacement_uid = uid

                self.get_logger().info(
                    f'[ARM SLOT] replace logical_slide={output_slide_id}, '
                    f'object={old_object_id}->{product_id}, '
                    f'arm_slide_id={entry["arm_slide_id"]}'
                )
                break
            if replacement_uid is not None:
                break

        if replacement_uid is not None:
            self._arm_pending_removals[step_id] = [
                uid for uid in pending_uids if uid != replacement_uid
            ]

        self._commit_arm_slot_removals(step)

    def _convert_step_slide_ids_for_arm(self, step, action_name):
        step_id = int(step.step_id)
        if step_id in self._arm_cmd_slide_cache:
            return list(self._arm_cmd_slide_cache[step_id])

        action_name = str(action_name).upper()
        is_assemble = action_name == ARM_ACTION_ASSEMBLE
        logical_slide_ids = list(getattr(step, 'slide_ids', []))
        object_ids = list(step.object_ids)
        converted = []

        for idx, sid in enumerate(logical_slide_ids):
            slot_index = self._logical_slot_index(sid)
            # ASSEMBLE object_ids are output product IDs, not input raw IDs.
            # Look up each input by logical slide so batched products are handled
            # without falsely matching a product ID to a raw slot.
            object_id = (
                None
                if is_assemble
                else object_ids[idx] if idx < len(object_ids) else None
            )

            if self._is_raw_slot_index(slot_index):
                if action_name == 'LOAD':
                    converted.append(self._assign_raw_arm_position(sid, object_id))
                elif action_name == 'UNLOAD' or is_assemble:
                    converted.append(self._lookup_raw_arm_position(
                        sid, object_id,
                        remove_after_success=True,
                        step_id=step_id,
                    ))
                else:
                    converted.append(self._lookup_raw_arm_position(sid, object_id))
            else:
                # Product slot and assembly slots are already physical slot numbers.
                converted.append(slot_index)

        self._arm_cmd_slide_cache[step_id] = list(converted)
        return converted

    def _set_arm_request_slide_ids(self, req, step, action_name):
        converted = self._convert_step_slide_ids_for_arm(step, action_name)
        if hasattr(req, 'slide_ids'):
            req.slide_ids = list(converted)
        else:
            if converted:
                self.get_logger().warn(
                    '[ARM SLOT] ArmCommand.srv에 slide_ids 필드가 없어 변환된 적재 위치를 전달하지 못합니다. '
                    'sml_msgs/srv/ArmCommand.srv에 int32[] slide_ids를 추가해야 합니다.'
                )
        return converted

    def _send_amr_assemble_arm(self, step, nav_target, retry=0):
        req = ArmCommand.Request()
        # Step.PRODUCE is retained only as the sml_msgs compatibility enum.
        req.action = ARM_ACTION_ASSEMBLE
        req.object_ids = list(step.object_ids)
        req.location = int(step.station_id)
        arm_slide_ids = self._set_arm_request_slide_ids(req, step, req.action)

        self.get_logger().info(
            f'[ARM/ASSEMBLE] step {step.step_id} → '
            f'{req.action} product={list(step.object_ids)} | '
            f'location={req.location} | arm_slide_ids={arm_slide_ids}'
        )

        future = self.arm_client.call_async(req)
        future.add_done_callback(
            lambda f, s=step, r=retry: self._on_amr_assemble_arm_result(f, s, r))

    def _get_amr_assemble_nav_target(self, step):
        with self._lock:
            state = self._amr_assemble_states.get(int(step.step_id), {})
            return state.get('nav_target', str(step.station_id))

    def _on_amr_assemble_nav_accepted(self, future, step, retry):
        try:
            goal_handle = future.result()
        except Exception as e:
            self._fail_amr_assemble_step(
                step, f'NAV goal 전송 예외: {e}')
            return

        if not goal_handle.accepted:
            self._fail_amr_assemble_step(
                step, 'NAV goal 거절됨')
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f, s=step, r=retry: self._on_amr_assemble_nav_result(f, s, r))

    def _on_amr_assemble_nav_result(self, future, step, retry):
        MAX_RETRY = 1

        try:
            result = future.result().result
        except Exception as e:
            self._fail_amr_assemble_step(
                step, f'NAV 결과 수신 예외: {e}')
            return

        if not result.success:
            self.get_logger().error(
                f'[NAV/ASSEMBLE] step {step.step_id} 실패: {result.fail_reason}')

            if retry < MAX_RETRY and result.fail_reason == 'NAV_FAILED':
                self.get_logger().warn(
                    f'[NAV/ASSEMBLE] step {step.step_id} 재시도 '
                    f'({retry+1}/{MAX_RETRY})')
                goal = NavTask.Goal()
                self._assign_nav_goal_target(goal, int(step.station_id))
                self._send_amr_assemble_nav(step, goal, retry + 1)
            else:
                self._fail_amr_assemble_step(
                    step, f'NAV 최종 실패: {result.fail_reason}')
            return

        self.get_logger().info(
            f'[NAV/ASSEMBLE] step {step.step_id} 도착 완료')
        self._mark_amr_assemble_part_done(step, 'nav')

    def _on_amr_assemble_arm_result(self, future, step, retry):
        MAX_RETRY = 1

        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(
                f'[ARM/ASSEMBLE] step {step.step_id} 예외: {e}')
            if retry < MAX_RETRY:
                nav_target = self._get_amr_assemble_nav_target(step)
                self.get_logger().warn(
                    f'[ARM/ASSEMBLE] step {step.step_id} 재시도 '
                    f'({retry+1}/{MAX_RETRY})')
                self._send_amr_assemble_arm(step, nav_target, retry + 1)
            else:
                self._fail_amr_assemble_step(
                    step, f'ARM ASSEMBLE 예외: {e}')
            return

        if not response.success:
            message = getattr(response, 'message', '')
            self.get_logger().error(
                f'[ARM/ASSEMBLE] step {step.step_id} 실패: {message}')

            retriable = 'object not found' not in message.lower()
            if retry < MAX_RETRY and retriable:
                nav_target = self._get_amr_assemble_nav_target(step)
                self.get_logger().warn(
                    f'[ARM/ASSEMBLE] step {step.step_id} 재시도 '
                    f'({retry+1}/{MAX_RETRY})')
                self._send_amr_assemble_arm(step, nav_target, retry + 1)
            else:
                self._fail_amr_assemble_step(
                    step, f'ARM ASSEMBLE 최종 실패: {message}')
            return

        self.get_logger().info(
            f'[ARM/ASSEMBLE] step {step.step_id} 조립 완료 '
            f'| slots={list(response.slots)}')
        self._commit_amr_assemble_slot_changes(step)
        self._mark_amr_assemble_part_done(step, 'arm')

    def _mark_amr_assemble_part_done(self, step, part):
        ready_to_finish = False

        with self._lock:
            state = self._amr_assemble_states.get(int(step.step_id))
            if state is None or state.get('failed', False):
                return

            if part == 'nav':
                state['nav_done'] = True
            elif part == 'arm':
                state['arm_done'] = True
            else:
                self.get_logger().warn(
                    f'[AMR ASSEMBLE] 알 수 없는 완료 part={part}')
                return

            if state['nav_done'] and state['arm_done'] and not state['post_started']:
                state['post_started'] = True
                ready_to_finish = True

        if ready_to_finish:
            self.get_logger().info(
                f'[AMR ASSEMBLE] step {step.step_id} '
                'NAV 도착 + 조립 완료 → navigator 후처리 실행'
            )
            self._execute_nav_post_process(step)

    def _fail_amr_assemble_step(self, step, reason):
        should_log = False

        with self._lock:
            state = self._amr_assemble_states.get(int(step.step_id))
            if state is None:
                self.amr_busy = False
                should_log = True
            elif not state.get('failed', False):
                state['failed'] = True
                self.amr_busy = False
                should_log = True

        if should_log:
            self.get_logger().error(
                f'[AMR ASSEMBLE] step {step.step_id} 실패: {reason}')
            
    def _execute_arm(self, step, retry=0):
        MAX_RETRY = 1

        if not self.arm_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(
                f'[ARM] step {step.step_id}: arm 서비스 없음')
            self._set_amr_idle(step)
            return

        req = ArmCommand.Request()

        if step.action == Step.LOAD:
            req.action = 'LOAD'
        elif step.action == Step.UNLOAD:
            req.action = 'UNLOAD'
        elif step.action == Step.PRODUCE:
            req.action = ARM_ACTION_ASSEMBLE
        else:
            self.get_logger().error(
                f'[ARM] step {step.step_id}: 지원하지 않는 action={step.action}')
            self._set_amr_idle(step)
            return

        req.object_ids = list(step.object_ids)
        req.location = int(step.station_id)
        arm_slide_ids = self._set_arm_request_slide_ids(req, step, req.action)

        self.get_logger().info(
            f'[ARM] step {step.step_id} → '
            f'{req.action} {list(step.object_ids)} | location={req.location} | '
            f'logical_slide_ids={list(getattr(step, "slide_ids", []))} | '
            f'arm_slide_ids={arm_slide_ids}')

        future = self.arm_client.call_async(req)
        future.add_done_callback(
            lambda f, s=step, r=retry: self._on_arm_result(f, s, r))

    def _on_arm_result(self, future, step, retry):
        MAX_RETRY = 1

        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(
                f'[ARM] step {step.step_id} 예외: {e}')
            self._set_amr_idle(step)
            return

        if not response.success:
            self.get_logger().error(
                f'[ARM] step {step.step_id} 실패: {response.message}')
            retriable = 'object not found' not in response.message.lower()
            if retry < MAX_RETRY and retriable:
                self.get_logger().warn(
                    f'[ARM] step {step.step_id} 재시도 ({retry+1}/{MAX_RETRY})')
                self._execute_arm(step, retry + 1)
            else:
                self.get_logger().error(
                    f'[ARM] step {step.step_id} 최종 실패')
                self._set_amr_idle(step)
            return

        self.get_logger().info(
            f'[ARM] step {step.step_id} 완료 '
            f'| slots={list(response.slots)}')
        self._commit_arm_slot_removals(step)
        self._execute_nav_post_process(step)

    def _execute_nav_post_process(self, step, retry=0):
        MAX_RETRY = 1

        if not self.post_process_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(
                f'[POST] step {step.step_id}: post_process 서비스 없음')
            self._set_amr_idle(step)
            return

        self.get_logger().info(
            f'[POST] step {step.step_id} → navigator 후처리 실행')

        future = self.post_process_client.call_async(Trigger.Request())
        future.add_done_callback(
            lambda f, s=step, r=retry: self._on_nav_post_process_result(
                f, s, r))

    def _on_nav_post_process_result(self, future, step, retry):
        MAX_RETRY = 1

        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(
                f'[POST] step {step.step_id} 예외: {e}')
            if retry < MAX_RETRY:
                self.get_logger().warn(
                    f'[POST] step {step.step_id} 재시도 '
                    f'({retry+1}/{MAX_RETRY})')
                self._execute_nav_post_process(step, retry + 1)
            else:
                self._set_amr_idle(step)
            return

        if not response.success:
            self.get_logger().error(
                f'[POST] step {step.step_id} 실패: {response.message}')
            if retry < MAX_RETRY and response.message != 'NO_PENDING_POST_PROCESS':
                self.get_logger().warn(
                    f'[POST] step {step.step_id} 재시도 '
                    f'({retry+1}/{MAX_RETRY})')
                self._execute_nav_post_process(step, retry + 1)
            else:
                self._set_amr_idle(step)
            return

        self.get_logger().info(f'[POST] step {step.step_id} 완료')
        with self._lock:
            if self.wb_reserved_by_amr == int(step.step_id):
                self.wb_reserved_by_amr = None
            self.amr_busy = False
            self._amr_assemble_states.pop(int(step.step_id), None)
        self._on_step_complete(step.step_id)

    # ──────────────────────────────────────────────────────
    # WB 스텝 실행
    # ──────────────────────────────────────────────────────

    def _execute_wb(self, step, retry=0):
        if not self.wb_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(
                f'[WB] step {step.step_id}: WB 서버 없음')
            with self._lock:
                self.wb_busy = False
            return

        goal = WbTask.Goal()
        goal.work_type  = ('PRODUCE'
                           if step.action == Step.PRODUCE
                           else 'RECYCLE')
        goal.product_id = step.object_ids[0]

        self.get_logger().info(
            f'[WB] step {step.step_id} → '
            f'{goal.work_type} {list(step.object_ids)}')

        send_future = self.wb_client.send_goal_async(
            goal,
            feedback_callback=lambda fb, s=step: self._on_wb_feedback(fb, s))
        send_future.add_done_callback(
            lambda f, s=step, r=retry: self._on_wb_accepted(f, s, r))

    def _on_wb_feedback(self, feedback_msg, step):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f'[WB] step {step.step_id} 진행 중: '
            f'{fb.status}')

    def _on_wb_accepted(self, future, step, retry):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(
                f'[WB] step {step.step_id} goal 거절됨')
            with self._lock:
                self.wb_busy = False
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f, s=step, r=retry: self._on_wb_result(f, s, r))

    def _on_wb_result(self, future, step, retry):
        result = future.result().result

        if not result.success:
            self.get_logger().error(
                f'[WB] step {step.step_id} 실패: {result.fail_reason}')
            with self._lock:
                self.wb_busy = False
            return

        self.get_logger().info(f'[WB] step {step.step_id} 완료')
        with self._lock:
            self.wb_busy = False
        self._on_step_complete(step.step_id)

    # ──────────────────────────────────────────────────────
    # 유틸리티
    # ──────────────────────────────────────────────────────

    def _step_type_name(self, step_type):
        type_map = {Step.AMR: 'AMR', Step.WB: 'WB '}
        return type_map.get(step_type, '??')

    def _step_action_name(self, step):
        if step.type == Step.AMR and step.action == Step.PRODUCE:
            return 'ASSEMBLE'
        action_map = {
            Step.LOAD:    'LOAD   ',
            Step.UNLOAD:  'UNLOAD ',
            Step.PRODUCE: 'PRODUCE',
            Step.RECYCLE: 'RECYCLE',
            Step.GOAL:    'GOAL   ',
        }
        return action_map.get(step.action, '?')

    def _make_step_record(self, step):
        return {
            'step_id': int(step.step_id),
            'type': self._step_type_name(step.type),
            'action': self._step_action_name(step),
            'objects': list(step.object_ids),
            'station': int(step.station_id),
            'depends_on': list(step.depends_on),
        }

    def _mark_step_started(self, step):
        step_id = int(step.step_id)
        with self._lock:
            self.step_start_times.setdefault(step_id, time.monotonic())
            self.step_records[step_id] = self._make_step_record(step)

    def _log_step_duration_summary(self):
        with self._lock:
            records = dict(self.step_records)
            elapsed_times = dict(self.step_elapsed_times)
            plan_start_time = self.plan_start_time
            plan_end_time = self.plan_end_time

        self.get_logger().info('===== Step 소요 시간 요약 =====')

        total_step_elapsed = 0.0
        for step_id in sorted(records):
            record = records[step_id]
            elapsed = elapsed_times.get(step_id)

            if elapsed is None:
                elapsed_text = '미완료'
            else:
                elapsed_text = f'{elapsed:.2f}s'
                total_step_elapsed += elapsed

            self.get_logger().info(
                f'[{step_id:2d}] {record["type"]} | '
                f'{record["action"]} | '
                f'objects={record["objects"]} | '
                f'station={record["station"]} | '
                f'elapsed={elapsed_text}'
            )

        self.get_logger().info(
            f'개별 step 소요 시간 합계: {total_step_elapsed:.2f}s')

        if plan_start_time is not None and plan_end_time is not None:
            wall_elapsed = plan_end_time - plan_start_time
            self.get_logger().info(
                f'전체 실행 wall-clock 시간: {wall_elapsed:.2f}s ')

        self.get_logger().info('==============================')

    def _publish_status(self, msg: str):
        status = String()
        status.data = msg
        self.status_pub.publish(status)

    def _log_steps(self, steps):
        type_map   = {Step.AMR: 'AMR', Step.WB: 'WB '}
        action_map = {
            Step.LOAD:    'LOAD   ',
            Step.UNLOAD:  'UNLOAD ',
            Step.RECYCLE: 'RECYCLE',
            Step.GOAL:    'GOAL   ',
        }
        self.get_logger().info('===== 수신된 스텝 시퀀스 =====')
        for s in steps:
            nav_target = nav_target_for_station(int(s.station_id), self.side) if s.type == Step.AMR else '-'
            action_name = (
                'ASSEMBLE'
                if s.type == Step.AMR and s.action == Step.PRODUCE
                else 'PRODUCE'
                if s.type == Step.WB and s.action == Step.PRODUCE
                else action_map.get(s.action, '?')
            )
            self.get_logger().info(
                f'[{s.step_id:2d}] {type_map.get(s.type, "??")} | '
                f'{action_name} | '
                f'objects={list(s.object_ids)} | '
                f'station={s.station_id} | '
                f'nav_target={nav_target} | '
                f'depends_on={list(s.depends_on)}')
        self.get_logger().info('==============================')


def main(args=None):
    rclpy.init(args=args)
    node = SmlManagerNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
