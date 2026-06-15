#!/bin/bash

# Build the Docker image if it doesn't exist
echo "Building Docker image..."
sudo docker build -t gsolver .

# Set up X11 authentication (Rerun recommended approach)
XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
chmod 777 $XAUTH

echo "Starting Docker container with GPU and X11 support..."

# Check if nvidia-docker runtime is available
if docker info 2>/dev/null | grep -q nvidia; then
    echo "Using NVIDIA runtime for GPU acceleration..."
    sudo docker run --runtime=nvidia --rm --gpus all -it --privileged --network=host \
      -e NVIDIA_DRIVER_CAPABILITIES=all \
      -e DISPLAY=$DISPLAY \
      -v $XSOCK:$XSOCK \
      -v $XAUTH:$XAUTH \
      -e XAUTHORITY=$XAUTH \
      -v "$(pwd)":/workspace \
      -w /workspace \
      gsolver
else
    echo "NVIDIA runtime not available, falling back to regular Docker with X11..."
    # Fallback without NVIDIA runtime
    xhost +local:docker
    sudo docker run -it --rm \
      -e DISPLAY=$DISPLAY \
      -e QT_X11_NO_MITSHM=1 \
      -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
      -v "$(pwd)":/workspace \
      -w /workspace \
      --device /dev/dri \
      --group-add video \
      gsolver
    xhost -local:docker
fi

# Clean up
rm -f $XAUTH 2>/dev/null
