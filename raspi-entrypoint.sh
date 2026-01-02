#!/bin/bash
source /opt/ros/humble/setup.bash
cd krbai-sauvc-2026/ros2_ws
colcon build
source install/setup.bash
ros2 launch eggplant_bringup eggplant.launch.py port:=/dev/ttyACM0 mission:=mission.yaml &
exec bash