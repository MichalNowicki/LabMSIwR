#!/bin/bash

# --- Configuration ---
XAUTH="/tmp/.docker.xauth"
CONTAINER_NAME="MSIwR_05"

# Define the image names for GPU and CPU
GPU_IMAGE_NAME="ultralytics/ultralytics:latest"
CPU_IMAGE_NAME="ultralytics/ultralytics:latest-cpu"

# --- 1. GPU Option Handling (Default GPU Logic) ---
GPU_ARGS=""
NVIDIA_ENV_ARGS=""
IMAGE_NAME=""

# Assume GPU mode by default
echo "Attempting to run with **NVIDIA GPU** support by default..."
IMAGE_NAME="$GPU_IMAGE_NAME"

# Check if the nvidia-docker runtime is available
if docker info | grep -q "nvidia"; then
    GPU_ARGS="--gpus all"
    NVIDIA_ENV_ARGS="--env=NVIDIA_VISIBLE_DEVICES=all --env=NVIDIA_DRIVER_CAPABILITIES=all"
    echo "✅ NVIDIA runtime found. Using GPU image: **$IMAGE_NAME**"
else
    # Fallback to CPU mode
    IMAGE_NAME="$CPU_IMAGE_NAME"
    echo "⚠️ WARNING: NVIDIA runtime not available."
    echo "   Falling back to **CPU** mode, using image: **$IMAGE_NAME**"
fi

if [ -n "$1" ]; then
    echo "Usage: $0 (No arguments needed. Default is GPU with CPU fallback)"
    exit 1
fi

# --- 2. X11 Forwarding Setup ---
# This section prepares X11 forwarding for graphical applications (like Rviz).
xhost +local:root > /dev/null 2>&1

if [ -z "$DISPLAY" ]; then
    echo "⚠️ Warning: DISPLAY environment variable is not set. Graphical applications may fail."
    touch "$XAUTH" 2>/dev/null || true
else
    # Create or update the .docker.xauth file for secure X forwarding
    xauth_list=$(xauth nlist :0 2>/dev/null | sed -e 's/^..../ffff/')
    
    if [ ! -z "$xauth_list" ]; then
        echo "$xauth_list" | xauth -f "$XAUTH" nmerge -
    else
        touch "$XAUTH"
    fi
fi

chmod a+r "$XAUTH" 2>/dev/null || true

# --- 3. Clean up and Run Container ---
# Stop and remove any existing container with the same name
docker stop "$CONTAINER_NAME" > /dev/null 2>&1 || true
docker rm "$CONTAINER_NAME" > /dev/null 2>&1 || true

echo "Starting container **$CONTAINER_NAME**..."

docker run -it \
    --ipc=host \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH:$XAUTH" \
    --privileged \
    --network=host \
    $GPU_ARGS \
    $NVIDIA_ENV_ARGS \
    --name="$CONTAINER_NAME" \
    "$IMAGE_NAME" \
    /bin/bash