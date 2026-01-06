FROM osrf/ros:humble-desktop-full
ENV DEBIAN_FRONTEND=noninteractive

# 1. Install system-level dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git wget curl unzip pkg-config \
    libatlas-base-dev libsuitesparse-dev libglew-dev \
    libyaml-cpp-dev sqlite3 libsqlite3-dev rsync \
    libglm-dev libglfw3-dev libpng-dev libjpeg-dev libeigen3-dev \
    libboost-filesystem-dev libboost-program-options-dev \
    python3-colcon-common-extensions ros-humble-rosbag2-storage-mcap\
    && rm -rf /var/lib/apt/lists/*

ARG CMAKE_INSTALL_PREFIX=/usr/local
ARG NUM_THREADS=4

# 2. Build & Install g2o (Still required as a custom build for vslam)
WORKDIR /tmp
RUN git clone https://github.com/RainerKuemmerle/g2o.git && \
    cd g2o && git checkout 20230223_git && \
    mkdir build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX} \
          -DBUILD_SHARED_LIBS=ON -DG2O_USE_CHOLMOD=OFF -DG2O_USE_CSPARSE=ON \
          -DG2O_USE_OPENGL=OFF -DG2O_BUILD_APPS=OFF .. && \
    make -j${NUM_THREADS} && make install && rm -rf /tmp/*

# 3. Build & Install Iridescence (Core library)
WORKDIR /tmp
RUN git clone https://github.com/koide3/iridescence && \
    cd iridescence && git checkout 085322e0c949f75b67d24d361784e85ad7f197ab && \
    git submodule update --init --recursive && \
    mkdir build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo .. && \
    make -j${NUM_THREADS} && make install && rm -rf /tmp/*

# 4. Build & Install stella_vslam (Core library)
# We clone it here to ensure it is installed system-wide for the examples to find
WORKDIR /opt
RUN git clone --recursive --depth 1 https://github.com/stella-cv/stella_vslam.git && \
    cd stella_vslam && mkdir build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX} .. && \
    make -j${NUM_THREADS} && make install

# 5. Build & Install Iridescence Viewer (Optional but helpful for examples)
WORKDIR /tmp
RUN git clone --recursive https://github.com/stella-cv/iridescence_viewer.git && \
    cd iridescence_viewer && mkdir build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo .. && \
    make -j${NUM_THREADS} && make install && rm -rf /tmp/*


# 6. Build & Install backward-cpp (FIX FOR YOUR ERROR)
WORKDIR /tmp
RUN git clone https://github.com/bombela/backward-cpp.git && \
    cd backward-cpp && git checkout 5ffb2c879ebdbea3bdb8477c671e32b1c984beaa && \
    mkdir build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX} .. && \
    make -j${NUM_THREADS} && make install && rm -rf /tmp/*

# 7. Build the Standalone Examples (For your ./run_video_slam requirement)
WORKDIR /opt
RUN git clone https://github.com/stella-cv/stella_vslam_examples.git && \
    cd stella_vslam_examples && \
    git submodule update --init --recursive && \
    mkdir build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo -DUSE_STACK_TRACE_LOGGER=ON .. && \
    make -j${NUM_THREADS}

# 7. Setup ROS2 Workspace for stella_vslam_ros
WORKDIR /ros2_ws/src
RUN git clone --recursive -b ros2 --depth 1 https://github.com/stella-cv/stella_vslam_ros.git

WORKDIR /ros2_ws
RUN . /opt/ros/humble/setup.sh && \
    rosdep update && \
    rosdep install -y -i --from-paths src --skip-keys="stella_vslam backward google-glog libgoogle-glog-dev" && \
    colcon build --symlink-install

# Final setup
WORKDIR /opt/stella_vslam_examples/build
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

RUN ldconfig
ENTRYPOINT ["/bin/bash"]