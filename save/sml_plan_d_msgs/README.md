# sml_plan_d_msgs

Plan D용 ROS2 interface package입니다.
기존 `sml_msgs`를 유지하면서 Plan D의 `slide_ids`를 별도 패키지에서 실험하기 위해 분리했습니다.

## 변경 핵심

- `msg/Step.msg`
  - `int32[] slide_ids` 추가
- `srv/ArmCommand.srv`
  - `int32[] slide_ids` 추가
- `msg/Task.msg`, `srv/GetPlan.srv`
  - 패키지명을 `sml_plan_d_msgs`로 변경

## slide_id 규칙

일반 주문 재고:

```text
slide_id = order_index * 10 + slot_index
```

- `order_index`: `order_list` 기준 0~9
- `slot_index`
  - `0~4`: raw material slide
  - `5`: AMR assembly slot 0
  - `6`: AMR assembly slot 1
  - `9`: product slot, recycle product/WB-only product 전용

storage 반환 재고:

```text
slide_id = -(local_station_id * 10 + slot_index)
```

- `local_station_id`: 1~8
- home 0은 반환 대상이 아니므로 사용하지 않음

예:

```text
0   = order 0 raw slide 0
15  = order 1 AMR assembly slot 0
29  = order 2 product slot
-42 = local station 4로 반환할 재료, raw slide 2
```
