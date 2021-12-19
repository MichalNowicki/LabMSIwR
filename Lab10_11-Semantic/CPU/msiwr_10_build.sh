xhost +local:root

# BUILD THE IMAGE
ROS_IMAGE="msiwr/image10cpu"
docker build -f Dockerfile -t $ROS_IMAGE ./..
