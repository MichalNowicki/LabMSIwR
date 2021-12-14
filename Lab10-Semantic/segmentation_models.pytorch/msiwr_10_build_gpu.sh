xhost +local:root

# BUILD THE IMAGE
ROS_IMAGE="msiwr/image10gpu"
docker build -f docker/DockerfileGPU -t $ROS_IMAGE .
