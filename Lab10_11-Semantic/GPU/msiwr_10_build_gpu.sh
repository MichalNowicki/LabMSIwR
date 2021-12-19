xhost +local:root

# BUILD THE IMAGE
ROS_IMAGE="msiwr/image10gpu"
docker build -f Dockerfile -t $ROS_IMAGE ../.
