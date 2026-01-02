#!/bin/bash
sudo docker run \
    --interactive \
    --tty \
    --rm \
    --device /dev/ttyACM0:/dev/ttyACM0 \
    --volume ./ros2_ws:/home/krbai-sauvc-2026/ros2_ws \
    ros-humble-core-dev:latest