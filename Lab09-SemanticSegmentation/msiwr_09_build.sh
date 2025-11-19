xhost +local:root

# BUILD THE IMAGE
ROS_IMAGE="msiwr/image09"
docker build -f Dockerfile -t $ROS_IMAGE . --no-cache 
