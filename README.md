# Introduction 
A containerized ROS wrapper to publish poinclouds and disparity based on fisheye images from Intel Realsense T265 camera.

# Getting Started
1.	Build the container by running `./build_t265_pointcloud_image.sh` script
2.	Subscribed topics:
   - `/camera/fisheye1/image_raw`
   - `/camera/fisheye2/image_raw`
3.	Published Topics:
- `/camera/disparity` 
- `/camera/points` 
