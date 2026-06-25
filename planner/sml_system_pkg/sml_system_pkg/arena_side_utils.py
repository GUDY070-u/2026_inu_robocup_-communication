#!/usr/bin/env python3
"""
arena_side_utils.py

A/B 경기장 번호 체계와 planner 계산용 번호를 분리하기 위한 공통 유틸.

규칙:
- A 경기장 AMR station id: 1~8
- B 경기장 AMR station id: 9~16
- A 조립로봇 station: 6
- B 조립로봇 station: 15
- A 시작/복귀 위치 label: "a"
- B 시작/복귀 위치 label: "b"
- 좌표 계산은 기존 JSON 좌표를 사용하되, B 경기장은 계산 시 A 좌표 key로 변환한다.
"""


def normalize_side(side: str) -> str:
    side = str(side).strip().lower()
    if side in ("a", "side_a"):
        return "a"
    if side in ("b", "side_b"):
        return "b"
    raise ValueError("side must be 'a', 'b', 'side_a', or 'side_b'")


def side_to_eai_topic(side: str) -> str:
    side = normalize_side(side)
    return "/eai/task/side_a" if side == "a" else "/eai/task/side_b"


def side_to_home_label(side: str) -> str:
    side = normalize_side(side)
    return "a" if side == "a" else "b"


def side_to_fixed_workbench_station(side: str) -> int:
    side = normalize_side(side)
    return 6 if side == "a" else 15


# 공식 eai_task_server station name -> 우리 AMR station id
# 공식 topic의 station_id는 그대로 신뢰하지 않고 name 기준으로 변환한다.
EAI_NAME_TO_AMR_ID = {
    # A 경기장: 1~8
    "side_a_storage_1": 1,
    "side_a_storage_2": 2,
    "side_a_storage_3": 3,
    "side_a_storage_4": 4,
    "side_a_workbench_1": 5,
    "side_a_hybrid_1": 6,       # A 조립로봇 위치
    "side_a_workbench_2": 7,
    "side_a_customer_1": 8,

    # B 경기장: 9~16
    "side_b_storage_1": 9,
    "side_b_storage_2": 10,
    "side_b_storage_3": 11,
    "side_b_storage_4": 12,
    "side_b_workbench_1": 13,
    "side_b_workbench_2": 14,
    "side_b_hybrid_1": 15,      # B 조립로봇 위치, 14가 아님
    "side_b_customer_1": 16,
}


# B AMR station id -> planner 계산용 A-zone station id
# AMR에 전달되는 번호는 바꾸지 않고, PlannerCore/JSON 계산에만 사용한다.
B_AMR_ID_TO_PLANNER_ID = {
    9: 1,
    10: 2,
    11: 3,
    12: 4,
    13: 5,
    15: 6,  # B 조립로봇 station 15를 A 계산 좌표 6으로 사용
    14: 7,
    16: 8,
}


# planner 계산용 A-zone station id -> B AMR station id
PLANNER_ID_TO_B_AMR_ID = {
    1: 9,
    2: 10,
    3: 11,
    4: 12,
    5: 13,
    6: 15,  # A 계산 좌표 6은 B 실제 조립로봇 station 15
    7: 14,
    8: 16,
}


def remap_eai_station_id_to_amr_id(station_name: str, original_station_id: int) -> int:
    """
    공식 eai_task_server station name/id를 우리 AMR 기준 station id로 변환.
    이름이 매핑에 없으면 original_station_id를 유지한다.
    """
    name = str(station_name).strip().lower()
    return int(EAI_NAME_TO_AMR_ID.get(name, original_station_id))


def remap_eai_station_type_to_internal_type(
    station_name: str,
    original_station_type: int,
    st_storage: int = 1,
    st_workbench: int = 2,
    st_customer: int = 3,
) -> int:
    """
    공식 HYBRID(type=4)는 우리 시스템에서 조립로봇 위치로 사용하므로
    planner가 WORKBENCH 후보로 인식하도록 WORKBENCH(type=2)로 변환한다.
    """
    name = str(station_name).lower()

    if "storage" in name:
        return int(st_storage)
    if "workbench" in name:
        return int(st_workbench)
    if "hybrid" in name:
        return int(st_workbench)
    if "customer" in name:
        return int(st_customer)

    return int(original_station_type)


def amr_station_to_planner_station(station_id: int, side: str) -> int:
    """
    AMR 실제 station id를 PlannerCore/JSON 계산용 station id로 변환.
    A는 그대로, B는 9~16 -> 1~8 대응 좌표로 변환한다.
    """
    side = normalize_side(side)
    sid = int(station_id)

    if sid == 0:
        return 0

    if side == "b":
        return int(B_AMR_ID_TO_PLANNER_ID.get(sid, sid))

    return sid


def planner_station_to_amr_station(station_id: int, side: str) -> int:
    """
    PlannerCore가 만든 station id를 AMR 실제 station id로 복원.
    A는 그대로, B는 1~8 -> 9~16 대응 번호로 변환한다.
    """
    side = normalize_side(side)
    sid = int(station_id)

    if sid == 0:
        return 0

    if side == "b":
        return int(PLANNER_ID_TO_B_AMR_ID.get(sid, sid))

    return sid


def nav_home_target_for_side(side: str) -> str:
    return side_to_home_label(side)


def nav_target_for_station(station_id: int, side: str) -> str:
    """
    Navigator에 전달할 target label.
    station_id == 0이면 A/B 시작·복귀 위치를 명확히 구분하기 위해 a/b 반환.
    그 외에는 실제 AMR station 번호 문자열 반환.
    """
    sid = int(station_id)
    if sid == 0:
        return nav_home_target_for_side(side)
    return str(sid)