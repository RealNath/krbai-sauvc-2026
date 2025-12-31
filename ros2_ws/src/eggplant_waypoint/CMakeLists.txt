cmake_minimum_required(VERSION 3.8)
project(eggplant_waypoint)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(displacement_visualizer src/displacement_visualizer.cpp)
ament_target_dependencies(displacement_visualizer rclcpp geometry_msgs nav_msgs tf2 tf2_geometry_msgs)

# Add Executable
add_executable(waypoint_sequencer src/waypoint_sequencer.cpp)

# Include directories
target_include_directories(waypoint_sequencer PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link ROS2 dependencies
ament_target_dependencies(waypoint_sequencer
  rclcpp
  geometry_msgs
  std_srvs
)

# Explicitly link yaml-cpp
target_link_libraries(waypoint_sequencer yaml-cpp)

# Install rules
install(TARGETS
  waypoint_sequencer
  displacement_visualizer
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()