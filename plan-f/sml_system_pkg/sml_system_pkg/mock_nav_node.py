"""
mock_nav_node.py
navigate_to_station Action 서버 mock.
+ /robocup_navigator/post_process (std_srvs/Trigger) 서비스 mock.

시간 모델:
  - use_distance_time=False: 기존처럼 delay_sec 고정 지연
  - use_distance_time=True : 현재 station → 목표 station 거리 / amr_speed_mps + nav_overhead_sec
"""

import json
import math
import time

import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

from sml_msgs.action import NavTask

DEFAULT_STATION_COORD_JSON_PATH = (
    '/home/user/ros2_ws/src/sml_system_pkg/config/station_coordinates_a_zone.json'
)


class MockNavNode(Node):

    def __init__(self):
        super().__init__('mock_nav_node')
        self.cbg = ReentrantCallbackGroup()

        self.declare_parameter('delay_sec', 1.0)
        self.declare_parameter('use_distance_time', False)
        self.declare_parameter('amr_speed_mps', 0.50)
        self.declare_parameter('nav_overhead_sec', 0.0)
        self.declare_parameter('post_process_delay_sec', 0.0)
        self.declare_parameter('start_station_id', 0)
        self.declare_parameter('station_coord_json_path', DEFAULT_STATION_COORD_JSON_PATH)

        self.current_station_id = int(self.get_parameter('start_station_id').value)
        self.station_coords = self._load_station_coords()

        self._action_server = ActionServer(
            self,
            NavTask,
            'navigate_to_station',
            execute_callback=self._execute_cb,
            callback_group=self.cbg,
        )
        self.get_logger().info(
            '[MOCK NAV] navigate_to_station 서버 시작 | '
            f'use_distance_time={bool(self.get_parameter("use_distance_time").value)}, '
            f'fixed_delay={float(self.get_parameter("delay_sec").value):.2f}s, '
            f'speed={float(self.get_parameter("amr_speed_mps").value):.2f}m/s'
        )

        self._post_process_srv = self.create_service(
            Trigger,
            '/robocup_navigator/post_process',
            self._post_process_cb,
            callback_group=self.cbg,
        )
        self.get_logger().info('[MOCK NAV] post_process 서비스 시작')

    def _load_station_coords(self):
        path = str(self.get_parameter('station_coord_json_path').value).strip()
        if not path:
            return {}
        try:
            with open(path, 'r', encoding='utf-8') as f:
                data = json.load(f)
            raw = data.get('station_coordinates', data)
            coords = {}
            for key, value in raw.items():
                station_id = int(key)
                coords[station_id] = (float(value['x']), float(value['y']))
            self.get_logger().info(
                f'[MOCK NAV] station 좌표 로드 완료: {len(coords)}개, path={path}'
            )
            return coords
        except Exception as exc:
            self.get_logger().warn(
                f'[MOCK NAV] station 좌표 로드 실패: {exc}. fixed delay를 사용합니다.'
            )
            return {}

    def _station_coord(self, station_id: int):
        station_id = int(station_id)
        if station_id in self.station_coords:
            return self.station_coords[station_id]
        return (float(station_id), 0.0)

    def _travel_delay(self, target_station_id: int) -> float:
        use_distance_time = bool(self.get_parameter('use_distance_time').value)
        if not use_distance_time or not self.station_coords:
            return max(0.0, float(self.get_parameter('delay_sec').value))

        start = int(self.current_station_id)
        target = int(target_station_id)
        if start == target:
            dist = 0.0
        else:
            x1, y1 = self._station_coord(start)
            x2, y2 = self._station_coord(target)
            dist = math.hypot(x2 - x1, y2 - y1)

        speed = max(float(self.get_parameter('amr_speed_mps').value), 1e-6)
        overhead = max(0.0, float(self.get_parameter('nav_overhead_sec').value))
        return overhead + dist / speed

    def _execute_cb(self, goal_handle):
        station_id = int(goal_handle.request.station_id)
        delay_sec = self._travel_delay(station_id)
        self.get_logger().info(
            f'[MOCK NAV] goal 수신: from={self.current_station_id}, '
            f'to={station_id}, delay={delay_sec:.2f}s'
        )

        fb = NavTask.Feedback()
        fb.status = 'MOVING'
        goal_handle.publish_feedback(fb)

        time.sleep(delay_sec)

        fb.status = 'ARRIVED'
        goal_handle.publish_feedback(fb)

        self.current_station_id = station_id
        goal_handle.succeed()

        result = NavTask.Result()
        result.success = True
        result.fail_reason = ''
        self.get_logger().info(f'[MOCK NAV] 완료: station_id={station_id}')
        return result

    def _post_process_cb(self, request, response):
        delay_sec = max(0.0, float(self.get_parameter('post_process_delay_sec').value))
        if delay_sec > 0.0:
            time.sleep(delay_sec)
        self.get_logger().info(f'[MOCK NAV] post_process 호출됨 → success, delay={delay_sec:.2f}s')
        response.success = True
        response.message = ''
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MockNavNode()
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
