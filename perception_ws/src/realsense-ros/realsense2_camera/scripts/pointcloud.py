#!/usr/bin/env python
import rospy
import os
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
# import pcl

# Initialize global variables
bridge = CvBridge()
color_image = None
depth_image = None
color_camera_info = None
depth_camera_info = None

def color_image_callback(msg):
    global color_image
    color_image = bridge.imgmsg_to_cv2(msg, "bgr8")

def depth_image_callback(msg):
    global depth_image
    depth_image = bridge.imgmsg_to_cv2(msg, "32FC1")

def color_info_callback(msg):
    global color_camera_info
    color_camera_info = msg

def depth_info_callback(msg):
    global depth_camera_info
    depth_camera_info = msg

def save_point_cloud(frame_id):
    # This function needs to be filled with the process of converting the color and depth images
    # into a point cloud and then saving it as a PLY file.
    # This is a placeholder for the conversion and saving process.
    print(f"Saving point cloud for frame {frame_id}")

def listener():
    rospy.init_node('depth_to_ply_listener', anonymous=True)

    rospy.Subscriber("/camera/color/image_raw", Image, color_image_callback)
    rospy.Subscriber("/camera/depth/image_rect_raw", Image, depth_image_callback)
    rospy.Subscriber("/camera/color/camera_info", CameraInfo, color_info_callback)
    rospy.Subscriber("/camera/depth/camera_info", CameraInfo, depth_info_callback)

    # Main loop
    frame_id = 0
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        if color_image is not None and depth_image is not None:
            print("processing the frames %d", frame_id)
            save_point_cloud(frame_id)
            frame_id += 1
        rate.sleep()

if __name__ == '__main__':
    print("The code is running ...")
    listener()
