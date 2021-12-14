xhost +local:root

# BUILD THE IMAGE
ROS_IMAGE="msiwr/image10cpu"
docker build -f docker/DockerfileCPU -t $ROS_IMAGE .
