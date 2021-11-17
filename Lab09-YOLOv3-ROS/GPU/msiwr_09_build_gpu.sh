xhost +local:root

# BUILD THE IMAGE
ROS_IMAGE="msiwr/image09gpu"
docker build -f Dockerfile -t $ROS_IMAGE ./..
