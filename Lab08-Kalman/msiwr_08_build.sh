xhost +local:root

# BUILD THE IMAGE
ROS_IMAGE="msiwr/image08"
docker build -f Dockerfile -t $ROS_IMAGE ./..
