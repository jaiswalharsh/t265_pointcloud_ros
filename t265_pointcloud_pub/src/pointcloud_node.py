#!/usr/bin/env python

# ROS Wrapper to publish pointclouds using fisheye camera images from Intel Realsense T265

from __future__ import print_function

# ROS imports
import roslib
import rospy
# ROS Image message
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from sensor_msgs import point_cloud2
import std_msgs.msg
from rospy.numpy_msg import numpy_msg
import message_filters
from t265_pointcloud_pub.msg import Floats
from cv_bridge import CvBridge

# Import OpenCV and numpy
import cv2
import numpy as np
from math import tan, pi
# from matplotlib.cm import get_cmap


frame_data = {"left"  : None,
              "right" : None
              }

bridge = CvBridge()

pub = rospy.Publisher('/camera/disparity', numpy_msg(Floats), queue_size=10)

pub_xyz = rospy.Publisher('/camera/points', PointCloud2, queue_size=10)

pub_undist = rospy.Publisher('/camera/undistorted', Image, queue_size=10)

"""
Returns a numpy array of given shape=row,column from a tuple
"""
def tuple_to_array(tuple,row,column):
    array = np.asarray(tuple)
    array = np.reshape(array,(row,column))
    return array

"""
Returns a ROS PointCloud2 msg given a XYZpointcloud as numpy array
"""
def numpy_to_PointCloud2(array):
    msg = PointCloud2()

    msg.header.stamp = rospy.Time().now()

    msg.header.frame_id = "camera_odom_frame"

    if len(array.shape) == 3:
        msg.height = array.shape[1]
        msg.width = array.shape[0]
    else:
        msg.height = 1
        msg.width = len(array)

    msg.fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1)]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = msg.point_step * array.shape[0]
    # msg.is_dense = int(np.isfinite(array).all())
    msg.is_dense = False
    msg.data = np.asarray(array, np.float32).tostring()
    return msg
    

# Create a callback function for the subscriber.
def image_callback(image_left, image_right, info_left, info_right):

    # print("encoding:",image_left.encoding)
    # Convert left and right camera images from sensor_msgs.Image type to numpy array
    image_left = bridge.imgmsg_to_cv2(image_left, desired_encoding='mono8')
    image_left = np.asarray(image_left)
    image_right = bridge.imgmsg_to_cv2(image_right, desired_encoding='mono8')
    image_right = np.asarray(image_right)
    frame_data["left"] = image_left
    frame_data["right"] = image_right

    # Configure the OpenCV stereo algorithm. See
    # https://docs.opencv.org/3.4/d2/d85/classcv_1_1StereoSGBM.html for a
    # description of the parameters
    window_size = 5 # EB 10
    min_disp = 0
    # must be divisible by 16
    num_disp = 112 - min_disp #EB 224
    max_disp = min_disp + num_disp
    stereo = cv2.StereoSGBM_create(minDisparity = min_disp,
                                    numDisparities = num_disp,
                                    blockSize = 16,
                                    P1 = 8*3*window_size**2,
                                    P2 = 32*3*window_size**2,
                                    disp12MaxDiff = 1,
                                    uniquenessRatio = 10,
                                    speckleWindowSize = 100,
                                    speckleRange = 32)


    # Convert the subscribed Intrinsics from tuples to arrays of desired dimensions
    K_left  = tuple_to_array(info_left.K,3,3) 
    D_left  = tuple_to_array(info_left.D,4,-1)
    K_right  = tuple_to_array(info_right.K,3,3) 
    D_right  = tuple_to_array(info_right.D,4,-1)
    (width, height) = (info_right.width, info_left.height)

    # Get the relative extrinsics between the left and right camera
    # (R, T) = get_extrinsics(streams["left"], streams["right"])
    # TODO: change from harcoded values for each camera to possibly get extrinsics from realsense ROS (?)
    R = np.array([[ 0.99996543, -0.00261991, -0.00788222], [0.00263016,  0.99999571,  0.0012902],[0.0078788,  -0.00131089,  0.99996805]], np.float32)
    T = np.array([-6.37159646e-02,  5.28076089e-06, -2.89689109e-04], np.float32)

    # The code below till calculation of disparity is taken librealsense.
    # Check http://docs.ros.org/en/kinetic/api/librealsense2/html/t265__stereo_8py_source.html for more information

    # We need to determine what focal length our undistorted images should have
    # in order to set up the camera matrices for initUndistortRectifyMap.  We
    # could use stereoRectify, but here we show how to derive these projection
    # matrices from the calibration and a desired height and field of view

    # We calculate the undistorted focal length:
    #
    #         h
    # -----------------
    #  \      |      /
    #    \    | f  /
    #     \   |   /
    #      \ fov /
    #        \|/
    stereo_fov_rad = 90 * (pi/180)  # 90 degree desired fov #EB 120
    stereo_height_px = 300          # 300x300 pixel stereo output #EB 600
    stereo_focal_px = stereo_height_px/2 / tan(stereo_fov_rad/2)

    # We set the left rotation to identity and the right rotation
    # the rotation between the cameras
    R_left = np.eye(3)
    R_right = R

    # The stereo algorithm needs max_disp extra pixels in order to produce valid
    # disparity on the desired output region. This changes the width, but the
    # center of projection should be on the center of the cropped image
    stereo_width_px = stereo_height_px + max_disp
    stereo_size = (stereo_width_px, stereo_height_px)
    stereo_cx = (stereo_height_px - 1)/2 + max_disp
    stereo_cy = (stereo_height_px - 1)/2

    # Construct the left and right projection matrices, the only difference is
    # that the right projection matrix should have a shift along the x axis of
    # baseline*focal_length
    P_left = np.array([[stereo_focal_px, 0, stereo_cx, 0],
                        [0, stereo_focal_px, stereo_cy, 0],
                        [0,               0,         1, 0]])
    P_right = P_left.copy()
    P_right[0][3] = T[0]*stereo_focal_px

    # Construct Q for use with cv2.reprojectImageTo3D. Subtract max_disp from x
    # since we will crop the disparity later
    Q = np.array([[1, 0,       0, -(stereo_cx - max_disp)],
                    [0, 1,       0, -stereo_cy],
                    [0, 0,       0, stereo_focal_px],
                    [0, 0, -1/T[0], 0]])

    # Create an undistortion map for the left and right camera which applies the
    # rectification and undoes the camera distortion. This only has to be done
    # once
    m1type = cv2.CV_32FC1
    (lm1, lm2) = cv2.fisheye.initUndistortRectifyMap(K_left, D_left, R_left, P_left, stereo_size, m1type)
    (rm1, rm2) = cv2.fisheye.initUndistortRectifyMap(K_right, D_right, R_right, P_right, stereo_size, m1type)
    undistort_rectify = {"left"  : (lm1, lm2),
                            "right" : (rm1, rm2)}


    frame_copy = {"left"  : frame_data["left"].copy(),
                    "right" : frame_data["right"].copy()}

    # Undistort and crop the center of the frames
    center_undistorted = {"left" : cv2.remap(src = frame_copy["left"],
                                    map1 = undistort_rectify["left"][0],
                                    map2 = undistort_rectify["left"][1],
                                    interpolation = cv2.INTER_LINEAR),
                            "right" : cv2.remap(src = frame_copy["right"],
                                    map1 = undistort_rectify["right"][0],
                                    map2 = undistort_rectify["right"][1],
                                    interpolation = cv2.INTER_LINEAR)}

    # compute the disparity on the center of the frames and convert it to a pixel disparity (divide by DISP_SCALE=16)
    disparity = stereo.compute(center_undistorted["left"], center_undistorted["right"]).astype(np.float32) / 16.0

    # TEST
    # print(type(center_undistorted["left"]))
    img_msg = bridge.cv2_to_imgmsg(center_undistorted["left"])
    img_msg.header.stamp = rospy.Time().now()
    pub_undist.publish(img_msg)

    # TEST END

    # re-crop just the valid part of the disparity
    disparity = disparity[:,max_disp:]

    xyz = cv2.reprojectImageTo3D(disparity,Q)

    # Publish the disparity to ROS Topic
    # pub.publish(disparity)
    mask = disparity > 2
    out_points = xyz[mask]
    out_points = numpy_to_PointCloud2(out_points)

    pub_xyz.publish(out_points)
    rospy.loginfo_throttle(2, "Publishing points and disparity")

def main_loop():
    # Define your image topic
    # TODO: Find if fisheye1 is left or right camera??
    image_topic_left = "/camera/fisheye1/image_raw"
    image_topic_right = "/camera/fisheye2/image_raw"
    info_topic_left = "/camera/fisheye1/camera_info"
    info_topic_right = "/camera/fisheye2/camera_info"

    # Create a subscriber with appropriate topic, custom message and name of callback function.
    left_image_sub = message_filters.Subscriber(image_topic_left, Image)
    right_image_sub = message_filters.Subscriber(image_topic_right, Image)
    left_camera_info_sub = message_filters.Subscriber(info_topic_left, CameraInfo)
    right_camera_info_sub = message_filters.Subscriber(info_topic_right, CameraInfo)
    ts = message_filters.TimeSynchronizer([left_image_sub, right_image_sub, left_camera_info_sub,right_camera_info_sub], 10)
    ts.registerCallback(image_callback)
    # rospy.Subscriber(image_topic, Image, image_callback)
    # Wait for messages on topic, go to callback function when new messages arrive.
    # Create a publisher for disparity
    rospy.spin()

# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('t265_disparity_node', anonymous = True)
    # Go to the main loop.
    main_loop()