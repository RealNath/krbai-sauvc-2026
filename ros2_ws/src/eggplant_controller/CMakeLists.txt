cmake_minimum_required(VERSION 3.8)
project(eggplant_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# --- Kinematics Node ---
add_executable(thrusters_8_kinematics src/thrusters_8_kinematics.cpp)
target_include_directories(
  thrusters_8_kinematics
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
# Added tf2 and sensor_msgs dependencies for balancing mechanism
ament_target_dependencies(thrusters_8_kinematics 
  rclcpp 
  geometry_msgs 
  std_msgs 
  sensor_msgs 
  tf2 
  tf2_geometry_msgs
)

# --- Serial Manager Node ---
add_executable(serial_manager src/serial_manager.cpp)
target_include_directories(
  serial_manager
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(serial_manager
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

# --- Joy to Twist Node ---
add_executable(joy_to_twist_node src/joy_to_twist_node.cpp)
ament_target_dependencies(joy_to_twist_node rclcpp geometry_msgs sensor_msgs)

# --- Installation ---
install(TARGETS
  thrusters_8_kinematics
  serial_manager
  joy_to_twist_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()