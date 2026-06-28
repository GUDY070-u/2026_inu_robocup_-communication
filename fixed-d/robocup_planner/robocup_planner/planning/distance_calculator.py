"""
Distance calculator using the AMR team's waypoint YAML.

Uses the station_N_goal position for each station as the reference point.
Euclidean distance is used as an approximation of travel cost; the YAML
will be updated by the AMR team on competition day, so all distances
are recalculated at runtime by loading the file fresh.
"""

import math
import yaml
from typing import Dict, Optional, Tuple


class DistanceCalculator:
    def __init__(self, waypoint_yaml_path: str):
        with open(waypoint_yaml_path, 'r') as f:
            data = yaml.safe_load(f)
        self._waypoints: dict = data['waypoints']
        self._positions: Dict[int, Tuple[float, float]] = self._parse_positions()

    def _parse_positions(self) -> Dict[int, Tuple[float, float]]:
        positions: Dict[int, Tuple[float, float]] = {}
        # Station 0 (home / starting point)
        wp = self._waypoints.get('station_0_goal')
        if wp:
            positions[0] = (wp['position']['x'], wp['position']['y'])
        # Stations 1-20
        for i in range(1, 21):
            wp = self._waypoints.get(f'station_{i}_goal')
            if wp:
                positions[i] = (wp['position']['x'], wp['position']['y'])
        return positions

    def get_position(self, station_id: int) -> Optional[Tuple[float, float]]:
        return self._positions.get(station_id)

    def station_to_station(self, from_id: int, to_id: int) -> float:
        """Euclidean distance between two station goal positions."""
        a = self._positions.get(from_id)
        b = self._positions.get(to_id)
        if a is None or b is None:
            return float('inf')
        return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

    def point_to_station(self, x: float, y: float, station_id: int) -> float:
        """Euclidean distance from an arbitrary (x, y) to a station goal position."""
        pos = self._positions.get(station_id)
        if pos is None:
            return float('inf')
        return math.sqrt((x - pos[0]) ** 2 + (y - pos[1]) ** 2)

    def estimate_travel_time(
        self,
        from_id: int,
        to_id: int,
        driving_velocity: float,
        parking_duration: float,
        exiting_duration: float,
    ) -> float:
        """
        Rough travel time estimate between two stations.
        Assumes constant velocity cruise; parking/exiting add fixed overhead.
        """
        dist = self.station_to_station(from_id, to_id)
        if driving_velocity <= 0:
            return float('inf')
        return dist / driving_velocity + parking_duration + exiting_duration
