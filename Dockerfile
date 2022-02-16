FROM ubuntu:20.04

RUN DEBIAN_FRONTEND=noninteractive apt-get update && apt-get install -y \
    apt-utils \
    apt-transport-https \
    software-properties-common \
    sudo \
    tzdata \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/* && \
    dpkg-reconfigure -f noninteractive tzdata

ARG UNAME=t265_pointcloud_docker

# Create new user `docker` and disable password and gecos for later
RUN adduser --disabled-password --gecos '' $UNAME
# Add new user docker to sudo group
RUN adduser $UNAME sudo

# Ensure sudo group users are not 
# asked for a password when using 
# sudo command by ammending sudoers file
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

WORKDIR /home/$UNAME/catkin_ws

COPY install_ros_deps.sh /
# The ros_deps script installs ROS Noetic-base, OpenCV4 and other deps
RUN chmod +x /install_ros_deps.sh && \
    DEBIAN_FRONTEND=noninteractive /install_ros_deps.sh

RUN mkdir /src
COPY t265_pointcloud_pub src/t265_pointcloud_pub

RUN chmod +x src/t265_pointcloud_pub/src/pointcloud_node.py

ENV ROS_DISTRO noetic

# Catkin build ros wrapper with args telling the node to run with Python3
RUN . /opt/ros/$ROS_DISTRO/setup.sh &&\
    # cd /home/catkin_ws/ &&\
    # catkin clean -y &&\
    catkin build -DCMAKE_C_COMPILER=gcc-8 -DCMAKE_CXX_COMPILER=g++-8 -DPYTHON_EXECUTABLE=/usr/bin/python3


# USER $UNAME
# ENV USER $UNAME

# # setup entrypoint
COPY ros_entrypoint.sh /

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]