# syntax=docker/dockerfile:1
FROM ros:humble-ros-base

ARG DEBIAN_FRONTEND=noninteractive

# Install build essentials and common ROS 2 tooling
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-serial \
    ros-humble-rosbridge-server \
    ros-humble-tf-transformations \
    ros-humble-tf2-ros \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-slam-toolbox \
    ros-humble-rviz2 \
    && rm -rf /var/lib/apt/lists/*

# Configure rosdep (needed when building in-container)
RUN rosdep init || true && rosdep update

ENV ROS_WS=/workspace/ros2_ws \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

WORKDIR ${ROS_WS}

COPY docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
