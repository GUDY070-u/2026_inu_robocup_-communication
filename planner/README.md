# SML System 실행 가이드

이 README는 팀원이 동일한 절차로 시스템을 실행할 수 있도록 정리한 문서입니다.

실행 방식은 크게 두 가지입니다.

1. **테스트용**: 우리가 만든 모의 `order_server`를 사용하는 방식
2. **공식 eai_task_server용**: 공식 `eai_task_server`에서 `/eai/task/side_a` 또는 `/eai/task/side_b`를 받아 어댑터를 통해 `/sml/task`로 변환하는 방식

---

## 0. 공통 준비

모든 터미널에서 먼저 아래 명령어를 실행합니다.

```bash
cd ~/ros2_ws
source install/setup.bash
```

코드를 수정한 뒤에는 반드시 다시 빌드합니다.

```bash
cd ~/ros2_ws
colcon build --packages-select sml_system_pkg
source install/setup.bash
```

공식 `eai_task_server` 패키지를 같이 빌드한 경우에는 전체 빌드를 사용해도 됩니다.

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

---

## 1. 전체 노드 구조

### 1-1. 테스트용 구조

```text
order_server
    ↓
/sml/task
    ↓
sml_planning_node
    ↓
/sml/get_plan
    ↓
sml_manager_node
    ↓
AMR / ARM / Workbench
```

테스트용에서는 우리가 만든 `order_server`가 바로 `/sml/task`를 발행합니다.

---

### 1-2. 공식 eai_task_server용 구조

```text
eai_task_server
    ↓
/eai/task/side_a 또는 /eai/task/side_b
    ↓
eai_task_adapter_node
    ↓
/sml/task
    ↓
sml_planning_node
    ↓
/sml/get_plan
    ↓
sml_manager_node
    ↓
AMR / ARM / Workbench
```

공식 서버의 메시지 타입은 `sml_messages/msg/Task`이고, 우리 planner는 `sml_msgs/msg/Task`를 사용합니다.

따라서 중간에 `eai_task_adapter_node`가 필요합니다.

---

## 2. 경기장 번호 규칙

### 2-1. A 경기장

| 항목 | 값 |
|---|---|
| AMR station 번호 | 1 ~ 8 |
| 조립 로봇 위치 | station 6 |
| 시작/복귀 위치 | `a` |
| adapter 실행 side | `side:=a` |

### 2-2. B 경기장

| 항목 | 값 |
|---|---|
| AMR station 번호 | 9 ~ 16 |
| 조립 로봇 위치 | station 15 |
| 시작/복귀 위치 | `b` |
| adapter 실행 side | `side:=b` |

주의할 점은 B 경기장의 조립 로봇 위치가 단순 대칭 번호인 14가 아니라 **15번**이라는 점입니다.

---

## 3. 테스트용 실행 방법

테스트용은 우리가 만든 모의 `order_server`를 사용합니다.

### 터미널 1: planning node 실행

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run sml_system_pkg sml_planning_node --ros-args -p side:=a
```

기본 테스트는 A 경기장 기준으로 실행합니다.

---

### 터미널 2: manager node 실행

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run sml_system_pkg sml_manager_node --ros-args -p side:=a
```

---

### 터미널 3: order_server 실행

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run sml_system_pkg order_server
```

실행하면 터미널에서 Tier와 Stage를 선택합니다.

예시:

```text
Tier 선택 (1: Entry, 2: Beginner, 3: Advanced, 4: Expert): 2
Stage 선택 (1: Production, 2: Recycling, 3: Lifecycle): 1
```

---

### 테스트용 실행 결과 확인

정상적으로 동작하면 planning node에서 다음과 비슷한 로그가 나옵니다.

```text
[planning_node]: Task 수신 → 계획 생성 시작
```

manager node에서는 다음과 비슷한 로그가 나옵니다.

```text
[MANAGER] Task 수신 → 1초 후 GetPlan 요청
[MANAGER] 계획 수신 완료: N개 스텝
===== 수신된 스텝 시퀀스 =====
```

---

## 4. 공식 eai_task_server용 실행 방법

공식 서버를 사용할 때는 반드시 `eai_task_adapter_node`를 함께 실행해야 합니다.

중요: `publish_once:=true`로 실행할 경우, task가 한 번만 발행됩니다.  
따라서 **planning node와 adapter node를 먼저 실행한 뒤** 공식 task server를 실행해야 합니다.

---

# 4-1. A 경기장 실행

A 경기장은 station 1~8을 사용하고, 조립 로봇은 station 6입니다.

## 터미널 1: planning node

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run sml_system_pkg sml_planning_node --ros-args -p side:=a
```

## 터미널 2: manager node

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run sml_system_pkg sml_manager_node --ros-args -p side:=a
```

## 터미널 3: eai task adapter

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run sml_system_pkg eai_task_adapter_node --ros-args -p side:=a
```

정상 실행 시 adapter에서 다음과 비슷한 로그가 나옵니다.

```text
[ADAPTER] selected side: a
[ADAPTER] input : /eai/task/side_a
[ADAPTER] output: /sml/task
```

## 터미널 4: 공식 eai_task_server

예시: Beginner Production

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch eai_task_server task_server.launch.py scenario:=production stage:=beginner publish_once:=true
```

다른 시나리오는 `scenario`와 `stage` 값을 바꿔 실행합니다.

```bash
ros2 launch eai_task_server task_server.launch.py scenario:=recycling stage:=beginner publish_once:=true
```

```bash
ros2 launch eai_task_server task_server.launch.py scenario:=lifecycle stage:=advanced publish_once:=true
```

---

# 4-2. B 경기장 실행

B 경기장은 station 9~16을 사용하고, 조립 로봇은 station 15입니다.

## 터미널 1: planning node

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run sml_system_pkg sml_planning_node --ros-args -p side:=b
```

## 터미널 2: manager node

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run sml_system_pkg sml_manager_node --ros-args -p side:=b
```

## 터미널 3: eai task adapter

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run sml_system_pkg eai_task_adapter_node --ros-args -p side:=b
```

정상 실행 시 adapter에서 다음과 비슷한 로그가 나옵니다.

```text
[ADAPTER] selected side: b
[ADAPTER] input : /eai/task/side_b
[ADAPTER] output: /sml/task
```

B 경기장의 station 변환은 다음 규칙을 따릅니다.

| 공식 station name | AMR station id |
|---|---:|
| `side_b_storage_1` | 9 |
| `side_b_storage_2` | 10 |
| `side_b_storage_3` | 11 |
| `side_b_workbench_1` | 13 |
| `side_b_workbench_2` | 14 |
| `side_b_hybrid_1` | 15 |
| `side_b_customer_1` | 16 |

## 터미널 4: 공식 eai_task_server

예시: Beginner Production

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch eai_task_server task_server.launch.py scenario:=production stage:=beginner publish_once:=true
```

---

## 5. Topic 확인 명령어

현재 공식 task가 발행되는지 확인:

```bash
ros2 topic info /eai/task/side_a -v
ros2 topic echo /eai/task/side_a --once
```

B 경기장 확인:

```bash
ros2 topic info /eai/task/side_b -v
ros2 topic echo /eai/task/side_b --once
```

adapter가 변환한 `/sml/task` 확인:

```bash
ros2 topic info /sml/task -v
ros2 topic echo /sml/task --once
```

planner 서비스 확인:

```bash
ros2 service list | grep get_plan
```

노드 확인:

```bash
ros2 node list
```

---

## 6. 자주 발생하는 문제

### 6-1. planning node가 Task를 못 받는 경우

`publish_once:=true`일 때 공식 서버를 먼저 실행하면 adapter가 메시지를 못 받을 수 있습니다.

해결 방법:

1. `sml_planning_node` 실행
2. `eai_task_adapter_node` 실행
3. `sml_manager_node` 실행
4. 마지막으로 `eai_task_server` 실행

---

### 6-2. 고정 작업로봇 station이 없다는 에러

예시:

```text
계획 생성 실패: 고정 작업로봇 station 6이 arena_layout의 WORKBENCH 목록에 없습니다.
```

확인할 것:

- A 경기장인데 `side:=b`로 실행하지 않았는지
- B 경기장인데 `side:=a`로 실행하지 않았는지
- adapter가 실행 중인지
- `/sml/task`에 변환된 station id가 정상적으로 들어갔는지

A 경기장은 조립 로봇 station이 6이고, B 경기장은 15입니다.

---

### 6-3. Fast DDS shared memory 에러

다음과 같은 에러가 나올 수 있습니다.

```text
[RTPS_TRANSPORT_SHM Error] Failed init_port fastrtps_portXXXX
```

대부분 이전 ROS2 프로세스나 DDS shared memory 파일이 남아서 생기는 문제입니다.

해결:

```bash
sudo rm -f /dev/shm/fastrtps_port*
```

필요하면 남아있는 ROS2 프로세스를 종료합니다.

```bash
ps -ef | grep ros2
kill -9 <PID>
```

---

## 7. 권장 실행 순서 요약

### 테스트용

```text
1. sml_planning_node
2. sml_manager_node
3. order_server
```

### 공식 eai_task_server A 경기장

```text
1. sml_planning_node --ros-args -p side:=a
2. sml_manager_node  --ros-args -p side:=a
3. eai_task_adapter_node --ros-args -p side:=a
4. eai_task_server
```

### 공식 eai_task_server B 경기장

```text
1. sml_planning_node --ros-args -p side:=b
2. sml_manager_node  --ros-args -p side:=b
3. eai_task_adapter_node --ros-args -p side:=b
4. eai_task_server
```