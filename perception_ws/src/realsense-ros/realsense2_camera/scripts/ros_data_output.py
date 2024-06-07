#!/usr/bin/env python3
import rospy
import cv2
import os
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from scipy.io import savemat

class DataConverter:
    def __init__(self):
        rospy.init_node('data_converter', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        self.pc_sub = rospy.Subscriber("/camera/depth/color/points", PointCloud2, self.pc_callback)
        
        # Ensure the directories exist
        self.image_dir = "image_folder"
        self.pc_dir = "pointcloud_folder"
        if not os.path.exists(self.image_dir):
            os.makedirs(self.image_dir)
        if not os.path.exists(self.pc_dir):
            os.makedirs(self.pc_dir)
        
    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        image_name = "image_{}.png".format(rospy.Time.now())
        cv2.imwrite(os.path.join(self.image_dir, image_name), cv_image)
        print("Saved image to {}".format(os.path.join(self.image_dir, image_name)))
        
    def pc_callback(self, data):
        pc = pc2.read_points(data, skip_nans=True, field_names=("x", "y", "z"))
        points = np.array(list(pc))
        
        pc_name = "pointcloud_{}.mat".format(rospy.Time.now())
        savemat(os.path.join(self.pc_dir, pc_name), {'points': points})
        print("Saved point cloud to {}".format(os.path.join(self.pc_dir, pc_name)))

if __name__ == '__main__':
    try:
        rospy.loginfo("Starting data converter node")
        converter = DataConverter()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Data converter node terminated.")
        pass
