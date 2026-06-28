#!/usr/bin/env bash
# =============================================================================
# launch_test.sh — robocup_planner 통합 테스트 환경 자동 실행
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

# ── 새 세션 생성 (윈도우 0: mock_nav) ──────────────────────────────────────
tmux new-session -d -s "$SESSION" -n "nav" -x 220 -y 50
tmux send-keys -t "$SESSION:nav" "$SETUP && ros2 run sml_system_pkg mock_nav_node" Enter

# ── 윈도우 1: mock_arm ──────────────────────────────────────────────────────
tmux new-window -t "$SESSION" -n "arm"
tmux send-keys -t "$SESSION:arm" "$SETUP && ros2 run sml_system_pkg mock_arm_node" Enter

# ── 윈도우 2: mock_wb ───────────────────────────────────────────────────────
tmux new-window -t "$SESSION" -n "wb"
tmux send-keys -t "$SESSION:wb" "$SETUP && ros2 run sml_system_pkg mock_wb_node" Enter

# ── 윈도우 3: planner ───────────────────────────────────────────────────────
tmux new-window -t "$SESSION" -n "planner"
if [[ "$USE_PLAN_D" == true ]]; then
  echo "[launch] Plan D 모드: sml_planning_node + sml_manager_node"
  # 위/아래로 분할
  tmux send-keys -t "$SESSION:planner" "$SETUP && ros2 run sml_system_pkg sml_planning_node" Enter
  tmux split-window -t "$SESSION:planner" -v
  tmux send-keys -t "$SESSION:planner" "$SETUP && sleep 2 && ros2 run sml_system_pkg sml_manager_node" Enter
else
  echo "[launch] robocup_planner 모드"
  tmux send-keys -t "$SESSION:planner" "$SETUP && ros2 run robocup_planner planner_node" Enter
fi

# ── 윈도우 4: order_server (사용자 입력용, 마지막에 포커스) ──────────────────
tmux new-window -t "$SESSION" -n "order"
tmux send-keys -t "$SESSION:order" "$SETUP && sleep 3 && ros2 run sml_system_pkg order_server" Enter

# ── 윈도우 5: 모니터링 (ros2 topic/node 확인용 빈 셸) ────────────────────────
tmux new-window -t "$SESSION" -n "monitor"
tmux send-keys -t "$SESSION:monitor" "$SETUP" Enter

# order 창으로 포커스 이동
tmux select-window -t "$SESSION:order"

echo ""
echo "============================================"
echo " robocup 테스트 환경이 시작되었습니다."
echo "============================================"
echo " 윈도우 목록:"
echo "   0: nav     — mock_nav_node"
echo "   1: arm     — mock_arm_node"
echo "   2: wb      — mock_wb_node"
if [[ "$USE_PLAN_D" == true ]]; then
echo "   3: planner — sml_planning_node / sml_manager_node"
else
echo "   3: planner — robocup_planner planner_node"
fi
echo "   4: order   — order_server  ← 여기서 task 입력"
echo "   5: monitor — 빈 셸 (ros2 topic echo 등)"
echo ""
echo " 전환: Ctrl+b → 숫자  |  종료: tmux kill-session -t robocup"
echo "============================================"

tmux attach-session -t "$SESSION"
