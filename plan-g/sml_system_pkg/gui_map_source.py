"""GUI map data source that reuses the planner waypoint loader.

Place this file in:
    ~/ros2_ws/src/sml_system_pkg/sml_system_pkg/gui_map_source.py

The GUI should not read a separate station_coordinates JSON anymore.  Instead,
it should use the same robocup_waypoint.yaml parser that the planner uses.

Example:
    from sml_system_pkg.gui_map_source import PlannerWaypointMapSource

    map_source = PlannerWaypointMapSource(
        '/home/user/ros2_ws/src/sml_system_pkg/config/robocup_waypoint.yaml'
    )

    # task is sml_msgs/msg/Task or compatible object.
    gui_data = map_source.build_from_task(task)

    # gui_data['stations'][i] contains:
    #   station_id, x, y, materials, station_type, path, waypoints
"""

from __future__ import annotations

from typing import Any, Dict, Iterable, List, Optional, Tuple

from .planning.waypoint_cost import WaypointCostMap, load_waypoint_cost_map


class PlannerWaypointMapSource:
    """Build GUI-ready station data from the planner's waypoint YAML.

    This class is ROS-message tolerant: it only assumes that a Task-like object
    has `arena_layout`, and each Station-like object has `station_id`,
    `station_type`, and `material_ids`.
    """

    def __init__(self, waypoint_yaml_path: str, logger=None) -> None:
        self.waypoint_yaml_path = waypoint_yaml_path
        self.logger = logger
        self.cost_map: Optional[WaypointCostMap] = load_waypoint_cost_map(
            waypoint_yaml_path, logger
        )
        if self.cost_map is None:
            raise RuntimeError(f'waypoint YAML을 로드할 수 없습니다: {waypoint_yaml_path}')

    def reload(self) -> None:
        """Reload YAML after the file is updated."""
        self.cost_map = load_waypoint_cost_map(self.waypoint_yaml_path, self.logger)
        if self.cost_map is None:
            raise RuntimeError(f'waypoint YAML을 다시 로드할 수 없습니다: {self.waypoint_yaml_path}')

    def build_from_task(
        self,
        task: Any,
        station_ids: Optional[Iterable[int]] = None,
        include_waypoints: bool = True,
        include_paths: bool = True,
    ) -> Dict[str, Any]:
        """Build GUI map data using current Task arena/material state."""
        station_items, station_types = self._extract_station_state(task)
        ids = station_ids
        if ids is None and station_items:
            # Task에 실제 포함된 station만 그린다.
            ids = sorted(station_items.keys())

        return self.build(
            station_items=station_items,
            station_types=station_types,
            station_ids=ids,
            include_waypoints=include_waypoints,
            include_paths=include_paths,
        )

    def build(
        self,
        station_items: Optional[Dict[int, List[int]]] = None,
        station_types: Optional[Dict[int, int | str]] = None,
        station_ids: Optional[Iterable[int]] = None,
        include_waypoints: bool = True,
        include_paths: bool = True,
    ) -> Dict[str, Any]:
        """Build GUI map data without requiring a Task message."""
        if self.cost_map is None:
            raise RuntimeError('WaypointCostMap이 초기화되지 않았습니다')

        return self.cost_map.to_gui_map_data(
            station_items=station_items,
            station_types=station_types,
            station_ids=station_ids,
            include_waypoints=include_waypoints,
            include_paths=include_paths,
        )

    @staticmethod
    def _extract_station_state(task: Any) -> Tuple[Dict[int, List[int]], Dict[int, int]]:
        station_items: Dict[int, List[int]] = {}
        station_types: Dict[int, int] = {}

        if task is None:
            return station_items, station_types

        for station in getattr(task, 'arena_layout', []) or []:
            station_id = int(getattr(station, 'station_id'))
            station_items[station_id] = list(getattr(station, 'material_ids', []) or [])
            station_types[station_id] = int(getattr(station, 'station_type'))

        return station_items, station_types


def world_to_canvas_transform(
    bounds: Dict[str, float],
    width: int,
    height: int,
    padding: int = 40,
):
    """Return a function converting world x/y to canvas pixel x/y.

    This is GUI-toolkit independent.  Tkinter/PyQt/PySide can all use the
    returned function.
    """
    min_x = float(bounds.get('min_x', 0.0))
    max_x = float(bounds.get('max_x', 1.0))
    min_y = float(bounds.get('min_y', 0.0))
    max_y = float(bounds.get('max_y', 1.0))

    span_x = max(max_x - min_x, 1e-6)
    span_y = max(max_y - min_y, 1e-6)
    usable_w = max(width - 2 * padding, 1)
    usable_h = max(height - 2 * padding, 1)
    scale = min(usable_w / span_x, usable_h / span_y)

    map_w = span_x * scale
    map_h = span_y * scale
    offset_x = (width - map_w) / 2.0
    offset_y = (height - map_h) / 2.0

    def transform(x: float, y: float) -> Tuple[float, float]:
        # Canvas y-axis is downward, map y-axis is upward.
        px = offset_x + (float(x) - min_x) * scale
        py = offset_y + (max_y - float(y)) * scale
        return px, py

    return transform
