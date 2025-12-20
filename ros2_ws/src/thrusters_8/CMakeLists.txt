cmake_minimum_required(VERSION 3.8)
project(thrusters_8)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(thrusters_8_node src/thrusters_8_node.cpp)
target_include_directories(thrusters_8_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(thrusters_8_node
  rclcpp
  geometry_msgs
)

add_executable(joy_to_twist_node src/joy_to_twist_node.cpp)
ament_target_dependencies(joy_to_twist_node rclcpp geometry_msgs sensor_msgs)

install(TARGETS thrusters_8_node joy_to_twist_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()