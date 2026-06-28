"""Arena layout parsing and station coordinate loading."""

from collections import Counter

from sml_plan_d_msgs.msg import Station

from .planner_config import (
    BATCH_SIZE,
    BATCH_TO_RAW,
    FIXED_WORKBENCH_STATION_ID,
)


def load_station_coord_json(path, logger=None):
    """Load station coordinates from JSON without depending on a ROS Node."""
    import json
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
        _warn('station_coord_json_path가 비어 있습니다. 이동 시간은 fallback 좌표로 계산됩니다.')
        return {}

    if not os.path.exists(path):
        _warn(f'station 좌표 JSON 파일을 찾을 수 없습니다: {path}')
        return {}

    try:
        with open(path, 'r', encoding='utf-8') as f:
            data = json.load(f)

        coords = {}
        raw = None

        if isinstance(data, dict):
            if 'station_coordinates' in data:
                raw = data['station_coordinates']
            elif 'stations' in data:
                raw = data['stations']
            else:
                raw = data

        if isinstance(raw, dict):
            for station_id_str, entry in raw.items():
                station_id = int(station_id_str)

                if isinstance(entry, dict):
                    x = entry.get('x')
                    y = entry.get('y')
                elif isinstance(entry, list) and len(entry) >= 2:
                    x = entry[0]
                    y = entry[1]
                else:
                    continue

                if x is None or y is None:
                    continue

                coords[station_id] = (float(x), float(y))

        elif isinstance(raw, list):
            for entry in raw:
                if not isinstance(entry, dict):
                    continue

                station_id = entry.get('station_id', entry.get('id'))
                x = entry.get('x')
                y = entry.get('y')

                if station_id is None or x is None or y is None:
                    continue

                coords[int(station_id)] = (float(x), float(y))

        _info(f'station 좌표 JSON 로드 완료: {len(coords)}개 station, path={path}')
        return coords

    except Exception as e:
        _error(f'station 좌표 JSON 로드 실패: {e}')
        return {}


class ArenaParserMixin:
    def _parse_arena(self, arena_layout, material_model):
        station_items = {}
        workbench_ids = []
        customer_id = None
        storage_id = None

        stock_tokens = []
        waste_target_tokens = []
        token_ref = 0

        initial_counts = Counter(material_model['produce_initial_counts'])
        waste_counts = Counter(material_model['recycle_leftover_counts'])

        for station in arena_layout:
            station_items[station.station_id] = list(station.material_ids)

            if station.station_type == Station.ST_WORKBENCH:
                workbench_ids.append(station.station_id)
            elif station.station_type == Station.ST_CUSTOMER:
                customer_id = station.station_id
            elif station.station_type == Station.ST_STORAGE and storage_id is None:
                storage_id = station.station_id

            # CUSTOMER의 material_ids는 recycle product 표시용이므로 raw stock으로 보지 않는다.
            if station.station_type not in (Station.ST_STORAGE, Station.ST_HYBRID):
                continue

            for object_id in station.material_ids:
                raw, raw_count = self._decode_station_object(object_id)
                if raw is None:
                    continue

                token = {
                    'ref': token_ref,
                    'station_id': station.station_id,
                    'object_id': object_id,
                    'raw': raw,
                    'capacity': raw_count,
                    'remaining': raw_count,
                }
                token_ref += 1

                # lifecycle 생성 규칙을 역추론한다.
                # P-C에 해당하는 raw는 초기 생산 재료(stock), R-C는 recycle leftover target.
                if initial_counts[raw] > 0:
                    stock_tokens.append(token)
                    initial_counts[raw] -= raw_count
                elif waste_counts[raw] > 0:
                    waste_target_tokens.append(token)
                    waste_counts[raw] -= raw_count
                else:
                    # 공식 예시나 수동 task에서 모델과 맞지 않는 재료가 들어온 경우,
                    # 생산 재료로 사용할 수 있도록 stock으로 둔다.
                    stock_tokens.append(token)

        if not workbench_ids:
            raise RuntimeError('arena_layout에 WORKBENCH station이 없습니다')

        # 작업로봇은 실제 시스템에서 station 7번에 고정되어 있으므로,
        # 비용 기반으로 WORKBENCH를 선택하지 않는다.
        if FIXED_WORKBENCH_STATION_ID not in workbench_ids:
            raise RuntimeError(
                f'고정 작업로봇 station {FIXED_WORKBENCH_STATION_ID}이 '
                f'arena_layout의 WORKBENCH 목록에 없습니다. '
                f'현재 WORKBENCH 후보={workbench_ids}'
            )
        wb_id = FIXED_WORKBENCH_STATION_ID
        self.get_logger().info(f'[WB] fixed workbench={wb_id} 사용')

        if customer_id is None:
            raise RuntimeError('arena_layout에 CUSTOMER station이 없습니다')
        if storage_id is None:
            raise RuntimeError('arena_layout에 STORAGE station이 없습니다')

        return station_items, stock_tokens, waste_target_tokens, wb_id, customer_id, storage_id

    def _decode_station_object(self, object_id):
        """station material_id를 raw 단위로 해석한다."""
        if object_id in BATCH_TO_RAW:
            return BATCH_TO_RAW[object_id], BATCH_SIZE
        if 1 <= object_id <= 8:
            return object_id, 1
        # product_id 등은 raw stock으로 보지 않음
        return None, 0


