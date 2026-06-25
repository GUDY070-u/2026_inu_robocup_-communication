## ▶️ 실행 방법

* **Case 1. 자체 모의 테스트**
  우리가 만든 `order_server`가 `/sml/task`를 직접 발행하고, `sml_planning_node`와 `sml_manager_node`가 이를 받아 전체 흐름을 테스트합니다.

* **Case 2. eai_task_server 기반 모의 테스트**
  공식 `eai_task_server`가 `/eai/task/side_a`를 발행하고, `eai_task_adapter`가 이를 `/sml/task`로 변환하여 우리 시스템에 전달합니다.

모든 터미널에서 공통으로 아래 명령어를 먼저 실행합니다.

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

---

## Case 1. 우리가 만든 모의 테스트

이 방식은 우리 `order_server`를 사용하여 Task를 직접 생성하는 테스트입니다.

### 실행 구조

```text
order_server
    ↓ /sml/task
sml_planning_node
    ↑
    ↓ /sml/get_plan
sml_manager_node
    ↓
mock_nav_node / mock_arm_node / mock_wb_node
```

### 터미널 1 — mock nav

```bash
ros2 run sml_system_pkg mock_nav_node
```

### 터미널 2 — mock arm

```bash
ros2 run sml_system_pkg mock_arm_node
```

### 터미널 3 — mock wb

```bash
ros2 run sml_system_pkg mock_wb_node
```

### 터미널 4 — planning node

```bash
ros2 run sml_system_pkg sml_planning_node
```

### 터미널 5 — manager node

```bash
ros2 run sml_system_pkg sml_manager_node
```

### 터미널 6 — order server

```bash
ros2 run sml_system_pkg order_server
```

`order_server` 실행 후 Tier / Stage를 입력하면 Task가 `/sml/task`로 발행되고 전체 흐름이 시작됩니다.

정상 실행 시 planner에서 다음 순서의 로그가 출력됩니다.

```text
1. 스테이션 예상 로그
2. 시간 비용 기반 WB 작업 순서
3. material model
4. 실행 계획 요약
5. 스텝 시퀀스
```

manager에서 아래 로그가 출력되면 전체 스텝 실행이 완료된 것입니다.

```text
[MANAGER] ✅ 모든 스텝 완료!
```

---

## Case 2. eai_task_server 기반 모의 테스트

이 방식은 공식 `eai_task_server`를 사용하여 실제 경기 환경과 유사하게 Task를 수신하는 테스트입니다.

공식 서버는 `/eai/task/side_a`에 `sml_messages/msg/Task` 타입의 Task를 발행합니다.
우리 planner는 `/sml/task`의 `sml_msgs/msg/Task`를 사용하므로, 중간에 `eai_task_adapter`를 실행하여 메시지를 변환합니다.

### 실행 구조

```text
eai_task_server
    ↓ /eai/task/side_a
eai_task_adapter
    ↓ /sml/task
sml_planning_node
    ↑
    ↓ /sml/get_plan
sml_manager_node
    ↓
mock_nav_node / mock_arm_node / mock_wb_node
```

### 터미널 1 — mock nav

```bash
ros2 run sml_system_pkg mock_nav_node
```

### 터미널 2 — mock arm

```bash
ros2 run sml_system_pkg mock_arm_node
```

### 터미널 3 — mock wb

```bash
ros2 run sml_system_pkg mock_wb_node
```

### 터미널 4 — 공식 eai_task_server

예시: Production / Beginner task 실행

```bash
ros2 launch eai_task_server task_server.launch.py \
  scenario:=production \
  stage:=beginner \
  publish_once:=false
```

다른 시나리오를 테스트하려면 `scenario`와 `stage`를 변경합니다.

```bash
ros2 launch eai_task_server task_server.launch.py \
  scenario:=lifecycle \
  stage:=beginner \
  publish_once:=false
```

사용 가능한 예시는 다음과 같습니다.

```text
scenario:
  production
  recycling
  lifecycle

stage:
  entry
  beginner
  advanced
```

### 터미널 5 — eai_task_adapter

```bash
ros2 run sml_system_pkg eai_task_adapter --ros-args \
  -p input_topic:=/eai/task/side_a \
  -p output_topic:=/sml/task
```

대회장에서 공식 Task topic 이름이 다를 경우 `input_topic`만 변경하면 됩니다.

```bash
ros2 run sml_system_pkg eai_task_adapter --ros-args \
  -p input_topic:=/actual/task/topic \
  -p output_topic:=/sml/task
```

### 터미널 6 — planning node

```bash
ros2 run sml_system_pkg sml_planning_node
```

### 터미널 7 — manager node

```bash
ros2 run sml_system_pkg sml_manager_node
```

---

## eai_task_adapter 동작 확인

adapter가 정상적으로 동작하는지 확인하려면 별도 터미널에서 `/sml/task`를 확인합니다.

```bash
ros2 topic echo /sml/task
```

정상이라면 공식 `/eai/task/side_a`의 내용이 우리 시스템의 `/sml/task` 형식으로 변환되어 출력됩니다.

예시:

```yaml
order_list:
- order_type: 1
  product_id: 81
- order_type: 1
  product_id: 442
arena_layout:
- station_name: side_a_storage_1
  station_type: 1
  station_id: 1
  material_ids:
  - 2
  - 1
- station_name: side_a_storage_2
  station_type: 1
  station_id: 2
  material_ids:
  - 8
- station_name: side_a_storage_3
  station_type: 1
  station_id: 3
  material_ids:
  - 40
```

---

## Topic 확인 명령어

공식 eai task topic 확인:

```bash
ros2 topic list | grep eai
```

예상 출력:

```text
/eai/task
/eai/task/side_a
/eai/task/side_b
```

공식 side_a topic 타입 확인:

```bash
ros2 topic info /eai/task/side_a
```

예상 타입:

```text
Type: sml_messages/msg/Task
```

adapter 출력 topic 확인:

```bash
ros2 topic info /sml/task
```

예상 타입:

```text
Type: sml_msgs/msg/Task
```

---

## 두 테스트 방식의 차이

| 구분            | Case 1. 자체 모의 테스트      | Case 2. eai_task_server 테스트 |
| ------------- | ---------------------- | --------------------------- |
| Task 생성 주체    | `order_server`         | `eai_task_server`           |
| 입력 topic      | `/sml/task`            | `/eai/task/side_a`          |
| adapter 필요 여부 | 필요 없음                  | 필요                          |
| planner 입력    | `/sml/task`            | adapter 변환 후 `/sml/task`    |
| 목적            | 자체 생성 Task 기반 전체 흐름 검증 | 공식 Task 형식 기반 경기 환경 검증      |

---

## 추천 테스트 순서

처음에는 Case 1로 planner와 manager의 기본 동작을 확인합니다.

```text
Case 1:
order_server → planner → manager → mock nodes
```

이후 Case 2로 공식 Task 수신과 adapter 변환까지 포함한 흐름을 확인합니다.

```text
Case 2:
eai_task_server → adapter → planner → manager → mock nodes
```

Case 2까지 정상 동작하면 실제 경기장에서는 `eai_task_server` 대신 대회 서버가 제공하는 Task topic을 adapter가 구독하도록 설정하면 됩니다.
