#!/bin/bash

docker run --net=host \
    -it \
    --rm \
    --privileged \
    t265_pointcloud:inspectrone \
    rosrun t265_pointcloud_pub pointcloud_node.py
     
    
    # roscore
#/bin/bash -c "source /home/airsim_ros_wrapper/AirSim/ros/devel/setup.bash && roslaunch airsim_ros_pkgs airsim_nbvp.launch"
#/bin/bash

# USE commands below to mount files from host to container. To be used in docker run command above
# -v $(pwd)/settings.json:/home/airsim_ros_wrapper/Documents/AirSim/settings.json \
# -v $(pwd)/launch_files:/home/airsim_ros_wrapper/launch_files \
# --volume=/dev:/dev \