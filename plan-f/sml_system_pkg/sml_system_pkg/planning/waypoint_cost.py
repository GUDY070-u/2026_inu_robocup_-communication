"""Waypoint-sequence based travel-cost and GUI map helper.

This module is intentionally ROS-independent.

Planner usage:
    cost_map = load_waypoint_cost_map('/path/to/robocup_waypoint.yaml')
    distance = cost_map.station_distance(0, 6)

GUI usage:
    cost_map = load_waypoint_cost_map('/path/to/robocup_waypoint.yaml')
    gui_data = cost_map.to_gui_map_data(station_items={1: [10, 2], 6: [7]})

The navigator receives a station_id, loads that station's waypoint sequence from
robocup_waypoint.yaml, and follows each waypoint.  The planner should therefore
estimate travel cost using the same station sequence, while the GUI should draw
station positions from the same parsed data.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Optional, Tuple

try:
    import yaml
except ImportError:  # pragma: no cover - ROS images normally include PyYAML.
    yaml = None


XY = Tuple[float, float]


@dataclass(frozen=True)
class WaypointPose:
    name: str
    x: float
    y: float
    z: float = 0.0
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0
    qw: float = 1.0

    @property
    def xy(self) -> XY:
        return (self.x, self.y)

    def as_gui_dict(self) -> Dict[str, Any]:
        return {
            'name': self.name,
            'x': self.x,
            'y': self.y,
            'z': self.z,
            'orientation': {
                'x': self.qx,
                'y': self.qy,
                'z': self.qz,
                'w': self.qw,
            },
        }


@dataclass(frozen=True)
class StationProfile:
    station_id: int
    name: str
    sequence: List[str]
    post_process: bool = False


class WaypointCostMap:
    """Parsed waypoint YAML shared by planner and GUI.

    - Planner uses station_distance() and station_coord().
    - GUI uses to_gui_map_data(), station_display_points(), station_path_segments().
    """

    def __init__(
        self,
        waypoints: Dict[str, XY] | Dict[str, WaypointPose],
        stations: Dict[int, StationProfile],
    ):
        normalized_waypoints: Dict[str, WaypointPose] = {}
        for name, value in waypoints.items():
            if isinstance(value, WaypointPose):
                normalized_waypoints[str(name)] = value
            else:
                x, y = value
                normalized_waypoints[str(name)] = WaypointPose(
                    name=str(name), x=float(x), y=float(y)
                )

        self.waypoint_poses: Dict[str, WaypointPose] = normalized_waypoints
        # Backward-compatible XY dictionary used by older cost code.
        self.waypoints: Dict[str, XY] = {
            name: pose.xy for name, pose in self.waypoint_poses.items()
        }
        self.stations: Dict[int, StationProfile] = {
            int(k): v for k, v in stations.items()
        }

    def __len__(self) -> int:
        return len(self.stations)

    def has_station(self, station_id: int) -> bool:
        return int(station_id) in self.stations

    def station_profile(self, station_id: int) -> Optional[StationProfile]:
        return self.stations.get(int(station_id))

    def waypoint_pose(self, waypoint_name: str) -> Optional[WaypointPose]:
        return self.waypoint_poses.get(str(waypoint_name))

    def station_coord(self, station_id: int) -> Optional[XY]:
        """Return the final goal coordinate of a station profile.

        The last valid waypoint in the station sequence corresponds to the final
        station approach pose, for example station_6_goal.
        """
        pose = self.station_goal_pose(station_id)
        return None if pose is None else pose.xy

    def station_goal_pose(self, station_id: int) -> Optional[WaypointPose]:
        profile = self.station_profile(station_id)
        if profile is None or not profile.sequence:
            return None

        for waypoint_name in reversed(profile.sequence):
            pose = self.waypoint_pose(waypoint_name)
            if pose is not None:
                return pose
        return None

    def station_sequence_poses(self, station_id: int) -> List[WaypointPose]:
        """Return valid waypoint poses in the station's configured sequence."""
        profile = self.station_profile(station_id)
        if profile is None:
            return []
        poses = []
        for waypoint_name in profile.sequence:
            pose = self.waypoint_pose(waypoint_name)
            if pose is not None:
                poses.append(pose)
        return poses

    def station_uses_post_process(self, station_id: int) -> bool:
        profile = self.station_profile(station_id)
        return bool(profile and profile.post_process)

    def station_distance(self, from_station: int, to_station: int) -> Optional[float]:
        """Estimate distance from current station goal to target station sequence.

        Distance model:
            from_station final goal
            -> target station's first waypoint
            -> target station's second waypoint
            -> ...
            -> target station's final goal

        Returns None if the YAML does not contain enough information, so the
        caller can fall back to direct station-coordinate distance.
        """
        if from_station is None or to_station is None:
            return 0.0

        from_station = int(from_station)
        to_station = int(to_station)
        if from_station == to_station:
            return 0.0

        start = self.station_coord(from_station)
        target_points = self._station_sequence_points(to_station)

        if start is None or not target_points:
            return None

        distance = 0.0
        current = start
        for point in target_points:
            distance += _distance(current, point)
            current = point
        return distance

    def _station_sequence_points(self, station_id: int) -> List[XY]:
        return [pose.xy for pose in self.station_sequence_poses(station_id)]

    # ------------------------------------------------------------------
    # GUI helpers
    # ------------------------------------------------------------------

    def station_display_points(
        self,
        station_ids: Optional[Iterable[int]] = None,
    ) -> Dict[int, Dict[str, Any]]:
        """Return station label positions for GUI drawing.

        The display position is the final goal pose of each station.
        """
        ids = list(station_ids) if station_ids is not None else sorted(self.stations)
        result: Dict[int, Dict[str, Any]] = {}
        for station_id in ids:
            profile = self.station_profile(int(station_id))
            goal_pose = self.station_goal_pose(int(station_id))
            if profile is None or goal_pose is None:
                continue
            result[int(station_id)] = {
                'station_id': int(station_id),
                'name': profile.name,
                'x': goal_pose.x,
                'y': goal_pose.y,
                'post_process': profile.post_process,
                'goal_waypoint': goal_pose.name,
                'sequence': list(profile.sequence),
            }
        return result

    def station_path_segments(
        self,
        station_ids: Optional[Iterable[int]] = None,
    ) -> Dict[int, List[Dict[str, Any]]]:
        """Return per-station waypoint path segments for GUI drawing.

        Each segment is a line from one waypoint to the next inside a station
        sequence, for example station_6_sub_goal -> station_6_goal.
        """
        ids = list(station_ids) if station_ids is not None else sorted(self.stations)
        result: Dict[int, List[Dict[str, Any]]] = {}
        for station_id in ids:
            poses = self.station_sequence_poses(int(station_id))
            segments: List[Dict[str, Any]] = []
            for a, b in zip(poses, poses[1:]):
                segments.append({
                    'from': a.as_gui_dict(),
                    'to': b.as_gui_dict(),
                })
            result[int(station_id)] = segments
        return result

    def to_gui_map_data(
        self,
        station_items: Optional[Dict[int, List[int]]] = None,
        station_types: Optional[Dict[int, int | str]] = None,
        station_ids: Optional[Iterable[int]] = None,
        include_waypoints: bool = True,
        include_paths: bool = True,
        margin: float = 0.5,
    ) -> Dict[str, Any]:
        """Build GUI-ready map data from the same loader used by the planner.

        station_items:
            Optional current cargo/material status, keyed by station id.
            Example: {1: [10, 2], 6: [7], 8: []}

        station_types:
            Optional station type/status map from Task.arena_layout.
            Example: {1: 'STORAGE', 6: 'HYBRID', 8: 'CUSTOMER'}
        """
        station_items = station_items or {}
        station_types = station_types or {}
        ids = list(station_ids) if station_ids is not None else sorted(self.stations)

        stations_gui: List[Dict[str, Any]] = []
        all_points: List[XY] = []

        for station_id in ids:
            station_id = int(station_id)
            profile = self.station_profile(station_id)
            goal_pose = self.station_goal_pose(station_id)
            if profile is None or goal_pose is None:
                continue

            seq_poses = self.station_sequence_poses(station_id)
            all_points.extend([p.xy for p in seq_poses])

            station_entry: Dict[str, Any] = {
                'station_id': station_id,
                'name': profile.name,
                'x': goal_pose.x,
                'y': goal_pose.y,
                'post_process': profile.post_process,
                'materials': list(station_items.get(station_id, [])),
                'station_type': station_types.get(station_id),
                'sequence_names': list(profile.sequence),
            }
            if include_waypoints:
                station_entry['waypoints'] = [p.as_gui_dict() for p in seq_poses]
            if include_paths:
                station_entry['path'] = [{'x': p.x, 'y': p.y, 'name': p.name} for p in seq_poses]
            stations_gui.append(station_entry)

        if not all_points:
            bounds = {'min_x': 0.0, 'max_x': 1.0, 'min_y': 0.0, 'max_y': 1.0}
        else:
            xs = [p[0] for p in all_points]
            ys = [p[1] for p in all_points]
            bounds = {
                'min_x': min(xs) - margin,
                'max_x': max(xs) + margin,
                'min_y': min(ys) - margin,
                'max_y': max(ys) + margin,
            }

        return {
            'bounds': bounds,
            'stations': stations_gui,
        }


def load_waypoint_cost_map(path: str, logger=None) -> Optional[WaypointCostMap]:
    """Load robocup_waypoint.yaml and return a WaypointCostMap.

    Failure is non-fatal: the planner can still fall back to the previous JSON
    coordinate based cost model.
    """
    import os

    def _info(msg):
        if logger is not None:
            logger.info(msg)

    def _warn(msg):
        if logger is not None:
            logger.warn(msg)

    def _error(msg):
        if logger is not None:
            logger.error(msg)

    path = (path or '').strip()
    if not path:
        _warn('waypoint_yaml_path가 비어 있습니다. 기존 station 좌표 기반 비용을 사용합니다.')
        return None

    if yaml is None:
        _error('PyYAML을 import할 수 없습니다. sudo apt install python3-yaml 후 다시 실행하세요.')
        return None

    if not os.path.exists(path):
        _warn(f'waypoint YAML 파일을 찾을 수 없습니다: {path}')
        return None

    try:
        with open(path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f) or {}

        raw_waypoints = data.get('waypoints', {}) or {}
        raw_stations = data.get('stations', {}) or {}

        waypoints: Dict[str, WaypointPose] = {}
        for name, entry in raw_waypoints.items():
            pose = _extract_waypoint_pose(str(name), entry)
            if pose is not None:
                waypoints[str(name)] = pose

        stations: Dict[int, StationProfile] = {}
        for station_id_raw, entry in raw_stations.items():
            try:
                station_id = int(station_id_raw)
            except (TypeError, ValueError):
                continue
            if not isinstance(entry, dict):
                continue

            sequence = [str(x) for x in (entry.get('sequence') or [])]
            stations[station_id] = StationProfile(
                station_id=station_id,
                name=str(entry.get('name', f'station_{station_id}')),
                sequence=sequence,
                post_process=bool(entry.get('post_process', False)),
            )

        cost_map = WaypointCostMap(waypoints=waypoints, stations=stations)
        _info(
            f'waypoint YAML 로드 완료: waypoints={len(waypoints)}, '
            f'stations={len(stations)}, path={path}'
        )

        _warn_if_placeholder_b_side(cost_map, _warn)
        return cost_map

    except Exception as e:
        _error(f'waypoint YAML 로드 실패: {e}')
        return None


def _extract_waypoint_pose(name: str, entry) -> Optional[WaypointPose]:
    if not isinstance(entry, dict):
        return None

    # Navigator supports pose: [x, y, z, qx, qy, qz, qw]
    pose = entry.get('pose')
    if isinstance(pose, (list, tuple)) and len(pose) >= 2:
        try:
            x = float(pose[0])
            y = float(pose[1])
            z = float(pose[2]) if len(pose) > 2 else 0.0
            qx = float(pose[3]) if len(pose) > 3 else 0.0
            qy = float(pose[4]) if len(pose) > 4 else 0.0
            qz = float(pose[5]) if len(pose) > 5 else 0.0
            qw = float(pose[6]) if len(pose) > 6 else 1.0
            return WaypointPose(name=name, x=x, y=y, z=z, qx=qx, qy=qy, qz=qz, qw=qw)
        except (TypeError, ValueError):
            return None

    # Navigator also supports position/orientation split format.
    position = entry.get('position')
    if isinstance(position, dict):
        try:
            x = float(position.get('x'))
            y = float(position.get('y'))
            z = float(position.get('z', 0.0))
        except (TypeError, ValueError):
            return None

        orientation = entry.get('orientation') or {}
        if not isinstance(orientation, dict):
            orientation = {}
        try:
            qx = float(orientation.get('x', 0.0))
            qy = float(orientation.get('y', 0.0))
            qz = float(orientation.get('z', 0.0))
            qw = float(orientation.get('w', 1.0))
        except (TypeError, ValueError):
            qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0

        return WaypointPose(name=name, x=x, y=y, z=z, qx=qx, qy=qy, qz=qz, qw=qw)

    return None


# Backward-compatible helper name used by the first planner patch.
def _extract_xy(entry) -> Optional[XY]:
    pose = _extract_waypoint_pose('', entry)
    return None if pose is None else pose.xy


def _distance(a: XY, b: XY) -> float:
    return math.hypot(float(b[0]) - float(a[0]), float(b[1]) - float(a[1]))


def _warn_if_placeholder_b_side(cost_map: WaypointCostMap, warn_func) -> None:
    """Warn if many B-side station goals share exactly the same coordinate."""
    coords = []
    for station_id in range(9, 21):
        coord = cost_map.station_coord(station_id)
        if coord is not None:
            coords.append((round(coord[0], 4), round(coord[1], 4)))

    if len(coords) >= 4 and len(set(coords)) == 1:
        warn_func(
            'station 9~20의 goal 좌표가 모두 동일합니다. '
            'B side waypoint가 임시값이면 side:=b 비용 계산은 A side 정규화 좌표를 쓰거나 '
            '정확한 B side YAML로 교체해야 합니다.'
        )
