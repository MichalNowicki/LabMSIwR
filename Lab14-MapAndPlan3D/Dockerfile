FROM osrf/ros:melodic-desktop-full 

# SETUP ENVS
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV DEBIAN_FRONTEND noninteractive
ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# INSTALL SOME ESSENTIAL PROGRAMS
RUN apt-get update     && \
    apt-get install -y    \
        git ros-melodic-rqt-* ros-melodic-joy ros-melodic-teleop-twist-joy ros-melodic-teleop-twist-keyboard ros-melodic-laser-proc ros-melodic-rgbd-launch ros-melodic-depthimage-to-laserscan ros-melodic-rosserial-arduino ros-melodic-rosserial-python ros-melodic-rosserial-server ros-melodic-rosserial-client ros-melodic-rosserial-msgs ros-melodic-amcl ros-melodic-map-server ros-melodic-move-base ros-melodic-urdf ros-melodic-xacro ros-melodic-compressed-image-transport ros-melodic-rqt-image-view ros-melodic-gmapping ros-melodic-navigation ros-melodic-interactive-markers ros-melodic-dwa-local-planner git python3-catkin-tools vim ros-melodic-turtlebot3* ros-melodic-octomap ros-melodic-octomap-msgs ros-melodic-ompl ros-melodic-octomap-ros checkinstall ros-melodic-tf2-sensor-msgs && \
    rm -rf /var/lib/apt/lists/*

# CREATE CATKIN WS
RUN mkdir -p /catkin_ws/src/

# DOWNLOAD CODE
WORKDIR /catkin_ws/src

# COPY LOCAL
COPY local_3d_planner /catkin_ws/src/local_3d_planner
COPY msiwr_launcher /catkin_ws/src/msiwr_launcher
RUN git clone https://github.com/OctoMap/octomap_mapping.git

# ROSDEP
WORKDIR /catkin_ws
#RUN /bin/bash -c "source /opt/ros/melodic/setup.bash;apt-get update;rosdep install --from-paths src --ignore-src -r -y"

####### Inne zaleznosci
WORKDIR /catkin_ws/
RUN mkdir thirdparty
WORKDIR /catkin_ws/thirdparty
RUN git clone https://github.com/danfis/libccd
WORKDIR /catkin_ws/thirdparty/libccd
RUN mkdir build
WORKDIR /catkin_ws/thirdparty/libccd/build
RUN cmake -G "Unix Makefiles" -DBUILD_SHARED_LIBS=ON .. && make -j3 && make install

WORKDIR /catkin_ws/thirdparty
RUN git clone https://github.com/flexible-collision-library/fcl
WORKDIR /catkin_ws/thirdparty/fcl
RUN mkdir build
WORKDIR /catkin_ws/thirdparty/fcl/build
RUN cmake ..  -DCMAKE_BUILD_TYPE=Release && make -j3 && make install


# BUILD WORKSPACE
WORKDIR /catkin_ws/
#RUN /bin/bash -c ". /opt/ros/melodic/setup.bash;ldconfig; catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release;catkin build"

# FILL BASHRC
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc
RUN echo "export TURTLEBOT3_MODEL=waffle_pi" >> ~/.bashrc
