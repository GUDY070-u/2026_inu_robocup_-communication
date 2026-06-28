cmake_minimum_required(VERSION 3.8)
project(sml_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Order.msg"
  "msg/Station.msg"
  "msg/Task.msg"
  "msg/Step.msg"
  "srv/GetPlan.srv"
  "srv/ArmCommand.srv"
  "action/NavTask.action"
  "action/WbTask.action"
  DEPENDENCIES action_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
