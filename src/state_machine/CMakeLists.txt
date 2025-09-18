cmake_minimum_required(VERSION 3.8)
project(state_machine)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yasmin REQUIRED)
find_package(yasmin_ros REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(yasmin_viewer REQUIRED)
find_package(example_interfaces REQUIRED)

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

add_executable(state_machine_node src/publisher_demo.cpp)
add_executable(state_machine_yasmin src/yasmin_dji_tello.cpp)

ament_target_dependencies(state_machine_node
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  example_interfaces
  yasmin
  yasmin_ros
  yasmin_viewer
)

ament_target_dependencies(state_machine_yasmin
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  example_interfaces
  yasmin
  yasmin_ros
  yasmin_viewer
)

install(TARGETS
  state_machine_node
  state_machine_yasmin
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
