xhost +local:root

# BUILD THE IMAGE
ROS_IMAGE="msiwr/image06"
docker build -f Dockerfile -t $ROS_IMAGE . #--no-cache 
