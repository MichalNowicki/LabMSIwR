FROM osrf/ros:humble-desktop-full

# Dependencies we use, catkin tools is very good build system
# https://github.com/ethz-asl/kalibr/wiki/installation
RUN apt-get update && DEBIAN_FRONTEND=noninteractive \
	apt-get install -y \
	git wget vim python3-pip ros-humble-foxglove-bridge ros-humble-vision-msgs

# YOLOv8 installation
RUN pip3 install ultralytics

# Create the workspace and build kalibr in it
RUN mkdir -p /ros2_ws/src

# Camera driver
WORKDIR /ros2_ws/src
RUN git clone -b pr-colcon-build-fix https://github.com/openvmp/opencv_cam.git
RUN git clone https://github.com/ptrmu/ros2_shared.git
RUN git clone https://github.com/ros2/detection_visualizer

# Build environment
WORKDIR /ros2_ws
RUN . /opt/ros/humble/setup.sh && \
    colcon build
    
# Copy video and camera file
COPY pexels_videos_2053100.mp4 /ros2_ws/
COPY opencv_cam_params.yaml /ros2_ws/

# SETUP
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc
