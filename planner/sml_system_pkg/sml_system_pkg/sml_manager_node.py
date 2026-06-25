"""
sml_manager_node.py
GetPlan 서비스로 스텝 목록을 받아
depends_on 기반으로 AMR / WB를 병렬 실행하는 노드.

A/B 경기장 대응:
  - side:=a 또는 side:=b 파라미터 사용
  - 일반 station은 Step.station_id를 그대로 AMR에 전달
  - GOAL/복귀 station_id=0은 navigator goal 타입이 string이면 "a"/"b"로 전달
  - navigator goal 타입이 int32이면 현재 인터페이스 한계상 0을 유지하고 경고 로그를 출력
"""

import threading

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
)


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
        self.plan_requested  = False    # GetPlan 요청 여부 (중복 방지)

        # GetPlan 재시도 관련
        self._plan_retry_count = 0
        self._plan_timer       = None
        self._max_plan_retries = 10

        self.declare_parameter('side', 'a')
        self.side = normalize_side(self.get_parameter('side').value)

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
                    self.amr_busy = True
                    self.pending_steps.remove(step)
                    amr_step = step

                elif step.type == Step.WB and not self.wb_busy \
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

        if amr_step:
            self.get_logger().info(
                f'[MANAGER] AMR step {amr_step.step_id} 시작 '
                f'(action={amr_step.action}, '
                f'objects={list(amr_step.object_ids)}, '
                f'station={amr_step.station_id})')
            self._publish_status(
                f'AMR step {amr_step.step_id} 실행 중')
            self._execute_amr(amr_step)

        if wb_step:
            self.get_logger().info(
                f'[MANAGER] WB step {wb_step.step_id} 시작 '
                f'(action={wb_step.action}, '
                f'objects={list(wb_step.object_ids)})')
            self._publish_status(
                f'WB step {wb_step.step_id} 실행 중')
            self._execute_wb(wb_step)

        if all_done:
            self.get_logger().info('[MANAGER] ✅ 모든 스텝 완료!')
            self._publish_status('완료')

    def _on_step_complete(self, step_id):
        with self._lock:
            self.completed_steps.add(step_id)
            self.get_logger().info(
                f'[MANAGER] step {step_id} 완료 '
                f'| 완료: {sorted(self.completed_steps)} '
                f'| 남은 스텝: {len(self.pending_steps)}개')
        self._dispatch()

    # ──────────────────────────────────────────────────────
    # AMR 스텝 실행: NAV Action → ARM Service
    # ──────────────────────────────────────────────────────

    def _assign_nav_goal_target(self, goal, station_id: int) -> str:
        """
        navigator goal에 target을 넣는다.

        - station_id == 0이면 side별로 "a"/"b"를 목표로 사용한다.
        - NavTask.Goal.station_id가 string 타입이면 "a"/"b"를 그대로 넣는다.
        - NavTask.Goal.station_id가 int 타입이면 인터페이스 한계상 0을 넣고 경고한다.
        - goal에 location/target/station_name 같은 string 필드가 있으면 함께 채운다.
        """
        nav_target = nav_target_for_station(int(station_id), self.side)
        field_types = goal.get_fields_and_field_types()

        # 보조 문자열 필드가 존재하면 채움
        for string_field in ('location', 'target', 'station_name', 'station_label'):
            if string_field in field_types and field_types[string_field] == 'string':
                setattr(goal, string_field, nav_target)

        if 'station_id' in field_types:
            station_id_type = field_types['station_id']

            if station_id_type == 'string':
                goal.station_id = nav_target
                return nav_target

            # int 계열 station_id
            if nav_target in ('a', 'b'):
                self.get_logger().warn(
                    '[NAV] NavTask.Goal.station_id가 숫자 타입입니다. '
                    f'복귀 label={nav_target}를 직접 넣을 수 없어 station_id=0으로 전송합니다. '
                    'navigator에서 side 파라미터로 0을 a/b home으로 해석해야 합니다.'
                )
                goal.station_id = 0
            else:
                goal.station_id = int(nav_target)
            return nav_target

        # station_id 필드가 없고 target/location만 있는 경우
        return nav_target

    def _execute_amr(self, step, retry=0):
        MAX_RETRY = 1

        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(
                f'[NAV] step {step.step_id}: nav 서버 없음')
            with self._lock:
                self.amr_busy = False
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
            with self._lock:
                self.amr_busy = False
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
                with self._lock:
                    self.amr_busy = False
            return

        self.get_logger().info(
            f'[NAV] step {step.step_id} 도착 완료')

        if step.action == Step.GOAL:
            self.get_logger().info(
                f'[NAV] step {step.step_id} GOAL 도착 → ARM 생략, 완료 처리')
            with self._lock:
                self.amr_busy = False
            self._on_step_complete(step.step_id)
            return

        self.get_logger().info(f'[NAV] step {step.step_id} → ARM 실행')
        self._execute_arm(step)

    def _execute_arm(self, step, retry=0):
        MAX_RETRY = 1

        if not self.arm_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(
                f'[ARM] step {step.step_id}: arm 서비스 없음')
            with self._lock:
                self.amr_busy = False
            return

        req = ArmCommand.Request()
        req.action     = 'LOAD' if step.action == Step.LOAD else 'UNLOAD'
        req.object_ids = list(step.object_ids)
        req.location   = ''

        self.get_logger().info(
            f'[ARM] step {step.step_id} → '
            f'{req.action} {list(step.object_ids)}')

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
            with self._lock:
                self.amr_busy = False
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
                with self._lock:
                    self.amr_busy = False
            return

        self.get_logger().info(
            f'[ARM] step {step.step_id} 완료 '
            f'| slots={list(response.slots)}')
        self._execute_nav_post_process(step)

    def _execute_nav_post_process(self, step, retry=0):
        MAX_RETRY = 1

        if not self.post_process_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(
                f'[POST] step {step.step_id}: post_process 서비스 없음')
            with self._lock:
                self.amr_busy = False
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
                with self._lock:
                    self.amr_busy = False
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
                with self._lock:
                    self.amr_busy = False
            return

        self.get_logger().info(f'[POST] step {step.step_id} 완료')
        with self._lock:
            self.amr_busy = False
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

    def _publish_status(self, msg: str):
        status = String()
        status.data = msg
        self.status_pub.publish(status)

    def _log_steps(self, steps):
        type_map   = {Step.AMR: 'AMR', Step.WB: 'WB '}
        action_map = {
            Step.LOAD:    'LOAD   ',
            Step.UNLOAD:  'UNLOAD ',
            Step.PRODUCE: 'PRODUCE',
            Step.RECYCLE: 'RECYCLE',
            Step.GOAL:    'GOAL   ',
        }
        self.get_logger().info('===== 수신된 스텝 시퀀스 =====')
        for s in steps:
            nav_target = nav_target_for_station(int(s.station_id), self.side) if s.type == Step.AMR else '-'
            self.get_logger().info(
                f'[{s.step_id:2d}] {type_map.get(s.type, "??")} | '
                f'{action_map.get(s.action, "?")} | '
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