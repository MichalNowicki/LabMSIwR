#!/bin/bash

# --- Configuration ---
XAUTH="/tmp/.docker.xauth"
CONTAINER_NAME="MSIwR_06"
IMAGE_NAME="msiwr/image06"

# --- 1. GPU Option Handling (Default GPU Logic) ---
GPU_ARGS=""
NVIDIA_ENV_ARGS=""

# Assume CPU mode initially, then check for GPU support
echo "Checking for NVIDIA GPU support..."

# Check if the nvidia-docker runtime is available (Modern Docker/NVIDIA Container Toolkit)
# The output of 'docker info' usually contains 'Runtimes: nvidia' if configured.
if docker info 2>/dev/null | grep -q "Runtimes:.*nvidia"; then
    GPU_ARGS="--gpus all"
    NVIDIA_ENV_ARGS="--env=NVIDIA_VISIBLE_DEVICES=all --env=NVIDIA_DRIVER_CAPABILITIES=all"
    echo "✅ NVIDIA runtime found. The container will run with **GPU support**."
else
    echo "⚠️ WARNING: NVIDIA runtime not available."
    echo "   Falling back to standard **CPU** mode."
fi

# Exit if any arguments are passed (assuming this script takes none)
if [ -n "$1" ]; then
    echo "Usage: $0 (No arguments needed. GPU support is automatic with CPU fallback)"
    exit 1
fi

# --- 2. X11 Forwarding Setup ---
# This section prepares X11 forwarding for graphical applications.
echo "Setting up X11 forwarding..."
xhost +local:root > /dev/null 2>&1

if [ -z "$DISPLAY" ]; then
    echo "⚠️ Warning: DISPLAY environment variable is not set. Graphical applications may fail."
    # Ensure XAUTH file exists even if DISPLAY is unset, for volume mount to work
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
echo "Stopping and removing any existing container named **$CONTAINER_NAME**..."
docker stop "$CONTAINER_NAME" > /dev/null 2>&1 || true
docker rm "$CONTAINER_NAME" > /dev/null 2>&1 || true

echo "Starting container **$CONTAINER_NAME** with image **$IMAGE_NAME**..."

docker run -it \
    --ipc=host \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH:$XAUTH" \
    -p 8765:8765 \
    $GPU_ARGS \
    $NVIDIA_ENV_ARGS \
    --name="$CONTAINER_NAME" \
    "$IMAGE_NAME" \
    /bin/bash