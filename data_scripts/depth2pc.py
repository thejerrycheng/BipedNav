import os
import json
import cv2
import numpy as np
from plyfile import PlyData, PlyElement
import sys

def load_intrinsics(json_path):
    with open(json_path, 'r') as f:
        intrinsics = json.load(f)
    
    K = np.array(intrinsics['K']).reshape(3, 3)
    return K

def depth_to_point_cloud(depth, K):
    height, width = depth.shape
    fx, _, cx, _, fy, cy, _, _, _ = K.flatten()

    x = np.linspace(0, width - 1, width)
    y = np.linspace(0, height - 1, height)
    xv, yv = np.meshgrid(x, y)

    zv = depth / 1000.0  # Assuming depth is in millimeters and converting to meters
    xv = (xv - cx) * zv / fx
    yv = (yv - cy) * zv / fy

    points = np.stack((xv, yv, zv), axis=-1).reshape(-1, 3)
    return points

def save_point_cloud(points, output_path):
    vertices = [(point[0], point[1], point[2]) for point in points]
    vertices = np.array(vertices, dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4')])

    el = PlyElement.describe(vertices, 'vertex')
    PlyData([el]).write(output_path)

def main(depth_folder, json_path, output_folder):
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    K = load_intrinsics(json_path)

    for filename in os.listdir(depth_folder):
        if filename.endswith('.png'):
            depth_path = os.path.join(depth_folder, filename)
            depth = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)
            points = depth_to_point_cloud(depth, K)
            output_path = os.path.join(output_folder, filename.replace('.png', '.ply'))
            save_point_cloud(points, output_path)
            print(f'Saved {output_path}')

if __name__ == "__main__":
    if len(sys.argv) != 4:
        print("Usage: python script.py <depth_folder> <json_path> <output_folder>")
        sys.exit(1)
    
    depth_folder = sys.argv[1]
    json_path = sys.argv[2]
    output_folder = sys.argv[3]

    main(depth_folder, json_path, output_folder)
