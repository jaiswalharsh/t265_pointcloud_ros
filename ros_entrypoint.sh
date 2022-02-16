#!/bin/bash
set -e

# setup ros environment
. /home/t265_pointcloud_docker/catkin_ws/devel/setup.bash
exec "$@"
