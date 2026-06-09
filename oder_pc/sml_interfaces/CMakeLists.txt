cmake_minimum_required(VERSION 3.8)
project(sml_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# 1. 의존성 탐색 구역에 아래 내용을 추가합니다.
find_package(rosidl_default_generators REQUIRED)

# 2. 메시지 및 서비스 파일을 등록하는 구문을 여기에 추가합니다.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TaskPlan.msg"
  "msg/AmrCommand.msg"
  "msg/WbCommand.msg"
  "srv/ReportTaskResult.srv"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ⚠️ ament_package()는 항상 파일의 가장 마지막에 있어야 합니다.
ament_package()
