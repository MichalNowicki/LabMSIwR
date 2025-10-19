#!/bin/bash

# --- Configuration ---
XAUTH=/tmp/.docker.xauth
CONTAINER_NAME="MSIwR_02"
IMAGE_NAME="osrf/ros:humble-desktop-full"

# --- 1. GPU Option Handling ---
GPU_ARGS=""
if [ "$1" == "--gpu" ] || [ "$1" == "-g" ]; then
    # Check if the nvidia-docker runtime is available
    if docker info | grep -q "nvidia"; then
        GPU_ARGS="--gpus all"
        echo "✅ Running with NVIDIA GPU support."
    else
        echo "⚠️ WARNING: GPU flag provided, but NVIDIA runtime is not available."
        echo "   Falling back to CPU mode."
    fi
elif [ -n "$1" ]; then
    echo "Usage: $0 [-g|--gpu] (to enable NVIDIA GPU)"
    exit 1
else
    echo "🗄️ Running in CPU (non-GPU) mode."
fi

# --- 2. X11 Forwarding Setup ---
xhost +local:root > /dev/null 2>&1

if [ -z "$DISPLAY" ]; then
    echo "⚠️ Warning: DISPLAY environment variable is not set. Graphical applications (like Rviz) may fail."
    touch "$XAUTH" 2>/dev/null || true
else
    xauth_list=$(xauth nlist :0 2>/dev/null | sed -e 's/^..../ffff/')
    
    if [ ! -z "$xauth_list" ]; then
        echo "$xauth_list" | xauth -f "$XAUTH" nmerge -
        echo "Successfully created X authorization file: $XAUTH"
    else
        touch "$XAUTH"
    fi
fi

chmod a+r "$XAUTH" 2>/dev/null || true

# --- 3. Clean up and Run Container ---
docker stop "$CONTAINER_NAME" > /dev/null 2>&1 || true
docker rm "$CONTAINER_NAME" > /dev/null 2>&1 || true

echo "Starting container $CONTAINER_NAME..."

docker run -it \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH:$XAUTH" \
    --privileged \
    $GPU_ARGS \
    --publish 8765:8765 \
    --name="$CONTAINER_NAME" \
    "$IMAGE_NAME" \
    /bin/bash
