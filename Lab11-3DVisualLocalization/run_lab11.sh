#!/bin/bash

# --- Configuration ---
XAUTH="/tmp/.docker.xauth"
CONTAINER_NAME="MSIwR_11"
IMAGE_NAME="msiwr/image11"

# --- 1. X11 Forwarding Setup ---
echo "Setting up X11 forwarding..."
xhost +local:root > /dev/null 2>&1

# Ensure the XAUTH file exists before xauth tries to use it
touch "$XAUTH"

if [ ! -z "$DISPLAY" ]; then
    # Get the X11 cookie and merge it into our custom XAUTH file
    xauth_list=$(xauth nlist "$DISPLAY" 2>/dev/null | sed -e 's/^..../ffff/')
    if [ ! -z "$xauth_list" ]; then
        echo "$xauth_list" | xauth -f "$XAUTH" nmerge -
    fi
fi

chmod a+r "$XAUTH"

# --- 2. Clean up and Run Container ---
echo "Stopping and removing any existing container named $CONTAINER_NAME..."
docker stop "$CONTAINER_NAME" > /dev/null 2>&1 || true
docker rm "$CONTAINER_NAME" > /dev/null 2>&1 || true

echo "Starting container $CONTAINER_NAME..."

# Added --privileged to match official repo requirements
docker run -it \
    --privileged \
    --ipc=host \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH:$XAUTH" \
    -p 8765:8765 \
    --name="$CONTAINER_NAME" \
    "$IMAGE_NAME" 