# Start from the official ROS 2 Jazzy desktop image
FROM osrf/ros:jazzy-desktop

# Set environment to non-interactive to avoid timezone prompts during build
ENV DEBIAN_FRONTEND=noninteractive

# Update apt and install our required tools: Gazebo and Foxglove Bridge
RUN apt-get update && apt-get install -y \
    ros-jazzy-ros-gz \
    ros-jazzy-foxglove-bridge \
    ros-jazzy-robot-localization \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-gz-ros2-control \
    nano \
    && rm -rf /var/lib/apt/lists/*

# Set up our workspace directory inside the container
WORKDIR /workspace

# Automatically source ROS 2 when we open a terminal in the container
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
