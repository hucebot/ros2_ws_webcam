# Use official ROS2 Humble base image
FROM ros:humble-ros-base

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-opencv \
    ros-humble-cv-bridge \
	ros-humble-image-transport \
    ros-humble-compressed-image-transport \
    ros-humble-rviz2 \
    ros-humble-rqt \
    ros-humble-rqt-image-view \
    libgl1-mesa-glx \
    libx11-xcb1 \
    libxkbcommon-x11-0 \
 && rm -rf /var/lib/apt/lists/*

# Copy your ROS2 workspace into container
COPY . /ros2_ws_webcam

# Build the workspace
WORKDIR /ros2_ws_webcam
RUN . /opt/ros/humble/setup.sh && colcon build

# Source ROS2 and workspace
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /ros2_ws_webcam/install/setup.bash" >> ~/.bashrc