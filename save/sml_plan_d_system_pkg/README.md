# sml_plan_d_system_pkg

Plan D 전용 시스템 노드 패키지입니다. 기존 `sml_system_pkg`를 유지하고, 새 인터페이스 패키지 `sml_plan_d_msgs`를 사용합니다.

## 포함 노드

- `order_server`
- `sml_planning_node`
- `sml_manager_node`
- `mock_nav_node`
- `mock_arm_node`
- `mock_wb_node`

## 현재 단계

이 패키지는 Plan D 1단계입니다.

- `sml_plan_d_msgs` 사용
- `Step.slide_ids` 로그/전달 지원
- `ArmCommand.slide_ids` 전달 및 mock 로그 지원
- 기존 planner 동작은 크게 유지

다음 단계에서 planner 내부의 실제 Plan D slide allocation/scheduling을 구현합니다.
