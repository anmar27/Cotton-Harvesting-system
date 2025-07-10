#!/bin/bash

# Path to the configuration file
CONFIG_FILE="config.cfg"

# Load the configuration file if it exists
if [ -f "$CONFIG_FILE" ]; then
    source "$CONFIG_FILE"
else
    echo "Configuration file $CONFIG_FILE not found!"
    exit 1
fi

# Allow overriding variables through script arguments
UBUNTU_VERSION=${1:-$UBUNTU_VERSION}
ROS_DISTRO=${2:-$ROS_DISTRO}
USERNAME=${3:-$DOCKER_USER}
IMAGE_NAME="pmlab_base_u${UBUNTU_VERSION}_${ROS_DISTRO}"

# Variables for forwarding ssh agent into docker container
DOCKER_SSH_AUTH_ARGS=""
if [ ! -z "$SSH_AUTH_SOCK" ]; then
    DOCKER_SSH_AUTH_ARGS="-v $SSH_AUTH_SOCK:/run/host_ssh_auth_sock -e SSH_AUTH_SOCK=/run/host_ssh_auth_sock"
fi

# Settings required for having nvidia GPU acceleration inside the docker
DOCKER_GPU_ARGS="--env DISPLAY --env QT_X11_NO_MITSHM=1 --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw "

# Check if NVIDIA container toolkit or nvidia-docker is installed
dpkg -l | grep nvidia-container-toolkit &> /dev/null
HAS_NVIDIA_TOOLKIT=$?
which nvidia-docker > /dev/null
HAS_NVIDIA_DOCKER=$?

# Determine the Docker command to use based on NVIDIA support
if [ $HAS_NVIDIA_TOOLKIT -eq 0 ]; then
  docker_version=`docker version --format '{{.Client.Version}}' | cut -d. -f1`
  if [ $docker_version -ge 19 ]; then
	  DOCKER_COMMAND="docker run --gpus all"
  else
	  DOCKER_COMMAND="docker run --runtime=nvidia " 
  fi
elif [ $HAS_NVIDIA_DOCKER -eq 0 ]; then
  DOCKER_COMMAND="nvidia-docker run"
else
  echo "Running without nvidia-docker, if you have an NVidia card you may need it"\
  "to have GPU acceleration"
  DOCKER_COMMAND="docker run"
fi

# Set network arguments
DOCKER_NETWORK_ARGS="--net host"
if [[ "$@" == *"--net "* ]]; then
    DOCKER_NETWORK_ARGS=""
fi

# Set PYTHONPATH for ROS Melodic (Python 2.7 support)
DOCKER_ENV=""
if [[ $ROS_DISTRO == melodic ]]; then 
    DOCKER_ENV="-e PYTHONPATH=$PYTHONPATH:/home/$USERNAME/.local/lib/python2.7/site-packages"; 
fi

# Allow connections from any host (for X11 forwarding)
xhost +

# Ensure shared directory exists
if [ ! -d "/home/$USER/shared" ]; then
    mkdir -p "/home/$USER/shared"
fi

$DOCKER_COMMAND -it \
    $DOCKER_GPU_ARGS \
    $DOCKER_SSH_AUTH_ARGS \
    $DOCKER_NETWORK_ARGS \
    $DOCKER_ENV \
    --mount source="/home/$USER/shared",target="/shared",type=bind \
    --device="/dev/snd" \
    --device /dev/bus/usb \
    $IMAGE_NAME \
    bash