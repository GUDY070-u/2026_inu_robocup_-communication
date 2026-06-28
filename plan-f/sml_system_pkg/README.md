# sml_system_pkg

Plan D system package for RoboCup SML.

## What this planner does

- Uses `sml_msgs`.
- Generates non-empty `Step.slide_ids`.
- Sends AMR-capable products through AMR internal assembly instead of WB.
- Uses WB only for products `{8518, 48132, 46262}` and recycle tasks.
- Uses slide IDs:
  - `order_index * 10 + slot_index`
  - raw slides: `0~4`
  - assembly slots: `5, 6`
  - product slot: `9`
  - return-to-storage: `-(local_station_id * 10 + slot_index)`
- Enforces raw slide capacity 3 units and prevents the same order from using the same raw slide twice.
- Keeps AMR from interacting with the WB while the WB is active by step dependencies.
- GOAL/home is numeric station `0`.

## Build

```bash
cd ~/ros2_ws/src
unzip ~/Downloads/sml_msgs.zip
unzip ~/Downloads/sml_system_pkg.zip

cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select sml_msgs sml_system_pkg
source install/setup.bash
```

## Run with mocks

Terminal 1:
```bash
ros2 run sml_system_pkg mock_nav_node
```

Terminal 2:
```bash
ros2 run sml_system_pkg mock_arm_node
```

Terminal 3:
```bash
ros2 run sml_system_pkg mock_wb_node --ros-args -p delay_sec:=5.0
```

Terminal 4:
```bash
ros2 run sml_system_pkg sml_planning_node --ros-args -p side:=a
```

Terminal 5:
```bash
ros2 run sml_system_pkg sml_manager_node --ros-args -p side:=a
```

Terminal 6:
```bash
ros2 run sml_system_pkg order_server
```
