#!/usr/bin/env python

import rosbag
import sensor_msgs.point_cloud2 as pc2
import rospy
import numpy as np
import open3d as o3d
import os
import random

def subsample_pointcloud(point_cloud, percentage):
    num_points = len(point_cloud.points)
    sample_size = int(num_points * percentage)
    indices = random.sample(range(num_points), sample_size)
    return point_cloud.select_by_index(indices)

def apply_filters(point_cloud):
    # Downsample the point cloud
    voxel_down_pcd = point_cloud.voxel_down_sample(voxel_size=0.02)

    # Estimate normals
    voxel_down_pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(
        radius=0.1, max_nn=30))

    # Apply statistical outlier removal
    cl, ind = voxel_down_pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    inlier_cloud = voxel_down_pcd.select_by_index(ind)

    # Apply radius outlier removal
    cl, ind = inlier_cloud.remove_radius_outlier(nb_points=16, radius=0.05)
    filtered_cloud = inlier_cloud.select_by_index(ind)
    
    return filtered_cloud

def extract_pointclouds(bag_file, subsample_percentage):
    output_folder = "pointcloud"
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)
    
    try:
        bag = rosbag.Bag(bag_file)
    except Exception as e:
        rospy.logerr(f"Failed to open bag file: {e}")
        return
    
    index = 0

    try:
        for topic, msg, t in bag.read_messages(topics=['/camera/depth/color/points']):
            print("Processing point cloud...index: ", index)
            # Extract the point cloud data
            points_list = []

            for point in pc2.read_points(msg, skip_nans=True):
                points_list.append([point[0], point[1], point[2]])

            points_array = np.array(points_list)
            
            # Create a point cloud object
            point_cloud = o3d.geometry.PointCloud()
            point_cloud.points = o3d.utility.Vector3dVector(points_array)

            # Subsample the point cloud
            if subsample_percentage < 1.0:
                point_cloud = subsample_pointcloud(point_cloud, subsample_percentage)

            # Apply filters to the point cloud
            point_cloud = apply_filters(point_cloud)

            # Save the point cloud to a file
            output_file = os.path.join(output_folder, f"pointcloud_{index:06d}.ply")
            o3d.io.write_point_cloud(output_file, point_cloud)
            
            index += 1

    except Exception as e:
        rospy.logerr(f"Failed to extract point clouds: {e}")
    finally:
        bag.close()

if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description="Extract point cloud data from a ROS bag file.")
    parser.add_argument('bag_file', help="Path to the ROS bag file.")
    parser.add_argument('subsample_percentage', type=float, help="Subsample percentage of the point cloud (e.g., 0.1 for 10%).")

    args = parser.parse_args()

    rospy.init_node('extract_pointclouds', anonymous=True)
    extract_pointclouds(args.bag_file, args.subsample_percentage)
