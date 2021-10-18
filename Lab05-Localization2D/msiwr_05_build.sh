xhost +local:root

# BUILD THE IMAGE
ROS_IMAGE="msiwr/image05"
ROS_CONTAINER="MSIwR_05"
docker build -f Dockerfile -t $ROS_IMAGE ./..
