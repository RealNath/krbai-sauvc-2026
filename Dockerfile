FROM ros:humble-ros-core

# We build the needed tools from ros core docker image, because i can't find one compatible docker image with the necessary build tools
# Basic tools + ROS build tools
RUN apt-get update && apt-get install -y \
    ros-humble-tf2 \
    ros-humble-tf2-geometry-msgs \
    libyaml-cpp-dev \
    python3-colcon-common-extensions \
    python3-rosdep \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init || true
RUN rosdep update

# Initialize shell with correct environment
COPY raspi-entrypoint.sh /home
WORKDIR /home
RUN chmod +x raspi-entrypoint.sh
ENTRYPOINT ["./raspi-entrypoint.sh"]
CMD ["bash"]