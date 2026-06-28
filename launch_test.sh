#!/usr/bin/env bash
# =============================================================================
# launch_test.sh — robocup_planner 통합 테스트 환경 자동 실행
#
# 레이아웃 (한 화면):
#   ┌─────────────┬─────────────┬─────────────┐
#   │  mock_nav   │  mock_arm   │   mock_wb   │
#   ├─────────────┼─────────────┼─────────────┤
#   │   planner   │    order    │   monitor   │
#   └─────────────┴─────────────┴─────────────┘
#
# 사용법:
#   bash launch_test.sh            # robocup_planner 사용 (기본)
#   bash launch_test.sh --plan-d   # 기존 sml_planning_node + sml_manager_node 사용
#
# 종료:
#   tmux kill-session -t robocup
# =============================================================================

SESSION="robocup"
WS_DIR="$(cd "$(dirname "$0")" && pwd)"
SETUP="source /opt/ros/humble/setup.bash && source ${WS_DIR}/install/setup.bash"

# 옵션 파싱
USE_PLAN_D=false
for arg in "$@"; do
  [[ "$arg" == "--plan-d" ]] && USE_PLAN_D=true
done

# 기존 세션 종료
tmux kill-session -t "$SESSION" 2>/dev/null

# ── 새 세션 + pane 0: mock_nav (좌상단) ────────────────────────────────────
tmux new-session -d -s "$SESSION" -x 220 -y 50

# 상단 행: 세로로 3등분
# pane 0 (nav) | pane 1 (arm) | pane 2 (wb)
tmux split-window -t "$SESSION:0.0" -h -p 67   # pane 0·1 분리 → 0=nav, 1=우측
tmux split-window -t "$SESSION:0.1" -h -p 50   # pane 1·2 분리 → 1=arm, 2=wb

# 하단 행: 각 상단 pane을 수평으로 분할
# pane 0 → 3(planner), pane 1 → 4(order), pane 2 → 5(monitor)
tmux split-window -t "$SESSION:0.0" -v -p 50
tmux split-window -t "$SESSION:0.1" -v -p 50
tmux split-window -t "$SESSION:0.2" -v -p 50

# pane 배치 결과:
#  0=nav(상좌)  1=arm(상중)  2=wb(상우)
#  3=planner(하좌)  4=order(하중)  5=monitor(하우)

# ── 각 pane에 명령 전송 ────────────────────────────────────────────────────

# pane 0: mock_nav
tmux send-keys -t "$SESSION:0.0" \
  "$SETUP && ros2 run sml_system_pkg mock_nav_node" Enter

# pane 1: mock_arm
tmux send-keys -t "$SESSION:0.1" \
  "$SETUP && ros2 run sml_system_pkg mock_arm_node" Enter

# pane 2: mock_wb
tmux send-keys -t "$SESSION:0.2" \
  "$SETUP && ros2 run sml_system_pkg mock_wb_node" Enter

# pane 3: planner
if [[ "$USE_PLAN_D" == true ]]; then
  tmux send-keys -t "$SESSION:0.3" \
    "$SETUP && ros2 run sml_system_pkg sml_planning_node" Enter
else
  tmux send-keys -t "$SESSION:0.3" \
    "$SETUP && ros2 run robocup_planner planner_node" Enter
fi

# pane 4: order_server (3초 대기 후 실행, 사용자 입력용)
tmux send-keys -t "$SESSION:0.4" \
  "$SETUP && sleep 3 && ros2 run sml_system_pkg order_server" Enter

# pane 5: monitor (빈 셸)
tmux send-keys -t "$SESSION:0.5" \
  "$SETUP" Enter

# order pane으로 포커스
tmux select-pane -t "$SESSION:0.4"

# pane 제목 설정 (상태바에 표시)
tmux select-pane -t "$SESSION:0.0" -T "mock_nav"
tmux select-pane -t "$SESSION:0.1" -T "mock_arm"
tmux select-pane -t "$SESSION:0.2" -T "mock_wb"
tmux select-pane -t "$SESSION:0.3" -T "planner"
tmux select-pane -t "$SESSION:0.4" -T "order ← 입력"
tmux select-pane -t "$SESSION:0.5" -T "monitor"

echo ""
echo "============================================"
if [[ "$USE_PLAN_D" == true ]]; then
echo " [Plan D 모드] 테스트 환경이 시작되었습니다."
else
echo " [robocup_planner 모드] 테스트 환경이 시작되었습니다."
fi
echo "============================================"
echo " 레이아웃:"
echo "  ┌─────────────┬─────────────┬─────────────┐"
echo "  │  mock_nav   │  mock_arm   │   mock_wb   │"
echo "  ├─────────────┼─────────────┼─────────────┤"
echo "  │   planner   │  order ←★  │   monitor   │"
echo "  └─────────────┴─────────────┴─────────────┘"
echo ""
echo " pane 이동: Ctrl+b → 방향키 (또는 마우스)"
echo " 종료:      tmux kill-session -t robocup"
echo "============================================"

tmux attach-session -t "$SESSION"

