# Base image
ARG UBUNTU_VERSION=20.04
FROM ubuntu:${UBUNTU_VERSION}

# Dev container arguments
ARG ROS_DISTRO
ARG PYTHON_VERSION
ARG USERNAME
ARG UID
ARG GID

SHELL ["/bin/bash", "-c"]

# Avoiding tzdata requirement for user interaction during installation
ENV DEBIAN_FRONTEND=noninteractive

# Minimal setup
RUN apt update && \
    apt install -y sudo locales lsb-release mlocate gnupg gedit htop \
    wget unzip vim git mesa-utils && \
    dpkg-reconfigure locales && \
    apt upgrade -y

# Install ROS 
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    apt install curl -y && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    apt update && apt install ros-$ROS_DISTRO-desktop-full -y 
RUN apt install $PYTHON_VERSION-rosdep 
RUN apt install $PYTHON_VERSION-rosinstall $PYTHON_VERSION-rosinstall-generator $PYTHON_VERSION-wstool build-essential -y && \
    rosdep init && \
    rosdep update

# Create new user and home directory
RUN groupadd -f --gid $GID $USERNAME && \
    useradd --uid $UID --gid $GID --create-home $USERNAME && \
    echo $USERNAME:$USERNAME | chpasswd && \
    echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers && \
    adduser --disabled-password  $USERNAME sudo && \
    chown -R $USERNAME:$USERNAME /home/$USERNAME && \
    # Set the ownership of the overlay workspace to the new user
    mkdir -p /home/$USERNAME/iri_lab/iri_ws/src && \
    mkdir -p /home/$USERNAME/iri_lab/labrobotica/drivers && \
    mkdir -p /home/$USERNAME/iri_lab/labrobotica/algorithms && \
    chown -R $UID:$GID /home/$USERNAME/iri_lab/iri_ws

USER $USERNAME

# Install pip
RUN sudo apt install ${PYTHON_VERSION}-pip -y
ENV PATH="/home/$USERNAME/.local/bin:${PATH}"
RUN if [ $ROS_DISTRO != kinetic ] ; then \
    ${PYTHON_VERSION} -m pip install --upgrade pip; fi

# ROS Environment setup
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc && \
    bash -c "source ~/.bashrc"

# Build the ROS workspace
WORKDIR /home/$USERNAME/iri_lab/iri_ws
RUN . /opt/ros/$ROS_DISTRO/setup.bash && catkin_make -DPYTHON_EXECUTABLE=/usr/bin/$PYTHON_VERSION && source devel/setup.bash && \
    echo "source /home/$USERNAME/iri_lab/iri_ws/devel/setup.bash" >> ~/.bashrc

# Copy inputrc script
WORKDIR /home/$USERNAME
COPY inputrc_script ./.inputrc

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=compute,utility,display
