FROM osrf/ros:noetic-desktop-full 

# Dependencies we use, catkin tools is very good build system
# https://github.com/ethz-asl/kalibr/wiki/installation
RUN apt-get update && DEBIAN_FRONTEND=noninteractive \
	apt-get install -y \
	git wget autoconf automake nano \
	python3-dev python3-pip python3-scipy python3-matplotlib \
	ipython3 python3-wxgtk4.0 python3-tk python3-igraph \
	libeigen3-dev libboost-all-dev libsuitesparse-dev \
	doxygen \
	libopencv-dev \
	evince \
	libpoco-dev libtbb-dev libblas-dev liblapack-dev libv4l-dev \
	python3-catkin-tools python3-osrf-pycommon

RUN pip3 install pyx

# Create the workspace and build kalibr in it
ENV WORKSPACE /catkin_ws
RUN mkdir -p $WORKSPACE/src && cd $WORKSPACE

# KALIBR
WORKDIR $WORKSPACE
RUN git clone https://github.com/ethz-asl/kalibr $WORKSPACE/src/kalibr
RUN catkin init && catkin config --extend /opt/ros/noetic/ --merge-devel -DCMAKE_BUILD_TYPE=Release -j4 && catkin build -j$(nproc)

# SETUP
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc
