#!/bin/bash
CONTAINER_ID=$(sudo docker ps -q | head -n 1)

if [ -z "$CONTAINER_ID" ]; then
    echo "No running ROS2 containers found, make sure setup is done properly."
    exit 1
fi

sudo docker attach "$CONTAINER_ID"
