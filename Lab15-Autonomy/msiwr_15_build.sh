xhost +local:root

# BUILD THE IMAGE
ROS_IMAGE="msiwr/image15"
docker build -f Dockerfile -t $ROS_IMAGE ./..
