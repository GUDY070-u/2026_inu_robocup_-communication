#!/usr/bin/env bash
# run_test.sh — 테스트 노드 일괄 실행 및 로그 저장
#
# 사용법:
#   ./run_test.sh [옵션]
#
# 옵션 (순서대로 입력):
#   -s  Start side      1=A  2=B          (기본: 1)
#   -t  Tier            1=Entry 2=Beginner 3=Advanced 4=Expert  (기본: 2)
#   -g  Stage           1=Production 2=Recycling 3=Lifecycle    (기본: 1)
#   -m  Mode            1=preset 2=random  (기본: 1)
#   -w  Workspace       colcon 빌드 결과 디렉토리               (기본: ~/robocup_demo)
#   -d  초기화 대기(초) 노드 기동 후 order_server 실행까지 대기  (기본: 3)
#
# 예시:
#   ./run_test.sh                         # 기본값 실행
#   ./run_test.sh -s 1 -t 3 -g 1 -m 2    # Advanced, random

set -eo pipefail

# ── 기본값 ──────────────────────────────────────────────────────────────────
WS="${HOME}/robocup_demo"
START_SIDE=1
TIER=2
STAGE=1
MODE=1
INIT_WAIT=3

# ── 인수 파싱 ────────────────────────────────────────────────────────────────
while getopts "s:t:g:m:w:d:" opt; do
    case $opt in
        s) START_SIDE="$OPTARG" ;;
        t) TIER="$OPTARG" ;;
        g) STAGE="$OPTARG" ;;
        m) MODE="$OPTARG" ;;
        w) WS="$OPTARG" ;;
        d) INIT_WAIT="$OPTARG" ;;
        *) echo "Unknown option: -$OPTARG" >&2; exit 1 ;;
    esac
done

# ── 환경 설정 ────────────────────────────────────────────────────────────────
SETUP="$WS/install/setup.bash"
if [[ ! -f "$SETUP" ]]; then
    echo "[ERROR] setup.bash 없음: $SETUP"
    echo "        먼저 colcon build 를 실행하세요."
    exit 1
fi
source "$SETUP"

# ── 로그 디렉토리 ─────────────────────────────────────────────────────────────
LOG_DIR="$WS/logs/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOG_DIR"
echo "[INFO] 로그 저장 위치: $LOG_DIR"

# ── 프로세스 정리 (Ctrl+C 또는 오류 시) ──────────────────────────────────────
cleanup() {
    echo ""
    echo "[INFO] 노드 종료 중..."
    # jobs -p 는 이 셸이 시작한 모든 백그라운드 잡의 PID를 반환한다.
    # 파이프라인(ros2 | tee)은 tee 만 $! 에 잡히므로, 프로세스 그룹째 kill.
    local job_pids
    job_pids=$(jobs -p 2>/dev/null) || true
    if [[ -n "$job_pids" ]]; then
        echo "$job_pids" | xargs -r kill -- 2>/dev/null || true
    fi
    wait 2>/dev/null || true
    echo "[INFO] 완료. 로그: $LOG_DIR"
}
trap cleanup EXIT INT TERM

# ── 노드 실행 헬퍼 ───────────────────────────────────────────────────────────
start_node() {
    local name="$1"
    local log="$LOG_DIR/${name}.log"
    shift
    echo "[START] $name → $log"
    "$@" 2>&1 | tee "$log" &
}

# ── Mock 노드 + 플래너 시작 ───────────────────────────────────────────────────
start_node "mock_nav_node"  ros2 run sml_system_pkg mock_nav_node
start_node "mock_arm_node"  ros2 run sml_system_pkg mock_arm_node
start_node "mock_wb_node"   ros2 run sml_system_pkg mock_wb_node
start_node "planner_node"   ros2 run robocup_planner planner_node

# ── 노드 기동 대기 ────────────────────────────────────────────────────────────
echo "[INFO] ${INIT_WAIT}초 대기 (노드 초기화)..."
sleep "$INIT_WAIT"

# ── 숫자 옵션 → 문자열 변환 (order_server ROS 파라미터용) ──────────────────────
case "$START_SIDE" in
    1) SIDE_STR="a" ;; 2) SIDE_STR="b" ;; *) SIDE_STR="$START_SIDE" ;;
esac
case "$TIER" in
    1) TIER_STR="entry" ;; 2) TIER_STR="beginner" ;;
    3) TIER_STR="advanced" ;; 4) TIER_STR="expert" ;; *) TIER_STR="$TIER" ;;
esac
case "$STAGE" in
    1) STAGE_STR="production" ;; 2) STAGE_STR="recycling" ;;
    3) STAGE_STR="lifecycle" ;; *) STAGE_STR="$STAGE" ;;
esac
case "$MODE" in
    1) MODE_STR="preset" ;; 2) MODE_STR="random" ;; *) MODE_STR="$MODE" ;;
esac

# ── order_server: --ros-args 파라미터로 전달 + auto_publish ───────────────────
# stdin 파이핑 대신 ROS 파라미터를 직접 넘긴다.
# auto_publish:=true 이면 input() 호출 없이 즉시 publish하고 spin으로 넘어간다.
# order_server 는 publish 후에도 rclpy.spin() 으로 계속 돌므로 백그라운드 실행.
ORDER_LOG="$LOG_DIR/order_server.log"
echo "[START] order_server → $ORDER_LOG"
echo "[INFO]  side=$SIDE_STR  tier=$TIER_STR  stage=$STAGE_STR  mode=$MODE_STR"
ros2 run sml_system_pkg order_server \
    --ros-args \
    -p start_side:="$SIDE_STR" \
    -p tier:="$TIER_STR" \
    -p stage:="$STAGE_STR" \
    -p mode:="$MODE_STR" \
    -p auto_publish:=true \
    2>&1 | tee "$ORDER_LOG" &

# 태스크 발행 확인 (최대 15초 대기)
echo "[INFO] order_server 발행 대기 중..."
ELAPSED=0
while [[ $ELAPSED -lt 15 ]]; do
    if grep -q "Task published" "$ORDER_LOG" 2>/dev/null; then
        echo "[INFO] 태스크 발행 완료 (${ELAPSED}초 경과)"
        break
    fi
    sleep 1
    ELAPSED=$((ELAPSED + 1))
done
if [[ $ELAPSED -ge 15 ]]; then
    echo "[WARN] order_server 발행 확인 실패 — 계속 진행합니다."
fi

# ── 플래너 종료 감지 (최대 5분 대기) ─────────────────────────────────────────
PLANNER_LOG="$LOG_DIR/planner_node.log"
echo "[INFO] 플래너 종료 대기 중 (최대 300초)..."
ELAPSED=0
while [[ $ELAPSED -lt 300 ]]; do
    if grep -q "Executor finished" "$PLANNER_LOG" 2>/dev/null; then
        echo "[INFO] 플래너 완료 감지 (${ELAPSED}초 경과)"
        break
    fi
    sleep 2
    ELAPSED=$((ELAPSED + 2))
done

if [[ $ELAPSED -ge 300 ]]; then
    echo "[WARN] 300초 초과 — 강제 종료합니다."
fi

# cleanup 은 trap 에 의해 자동 호출됨
