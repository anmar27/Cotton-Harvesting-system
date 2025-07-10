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
IMAGE_NAME="pmlab_base_u${UBUNTU_VERSION}_${ROS_DISTRO}"
DOCKER_USER=${4:-$DOCKER_USER}
DOCKER_GID=${5:-$DOCKER_GID}
DOCKER_UID=${4:-$DOCKER_UID}

# Set PYTHON_VERSION based on ROS_DISTRO
if [ "$ROS_DISTRO" = "noetic" ]; then
    PYTHON_VERSION="python3"
else
    PYTHON_VERSION="python"
fi

echo "======================="
echo " Building docker image "
echo " IMAGE_TAG:      ${IMAGE_NAME}"
echo " ROS DISTRO:     ${ROS_DISTRO}"
echo " PYTHON_VERSION: ${PYTHON_VERSION}"
echo " DOCKER_USER:    ${DOCKER_USER}"
echo " DOCKER_GID:     ${DOCKER_GID}"
echo " DOCKER_UID:     ${DOCKER_UID}"

docker build ${DOCKER_FILE_PATH} --build-arg ROS_DISTRO="${ROS_DISTRO}"\
                                 --build-arg UBUNTU_VERSION="${UBUNTU_VERSION}"\
                                 --build-arg PYTHON_VERSION="${PYTHON_VERSION}"\
                                 --build-arg USERNAME="${DOCKER_USER}"\
                                 --build-arg GID="${DOCKER_GID}"\
                                 --build-arg UID="${DOCKER_UID}"\
                                 -f Dockerfile \
                                 -t "${IMAGE_NAME}" .

