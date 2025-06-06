#!/usr/bin/env python3
import yaml
import cv2
import numpy as np
import open3d as o3d
import os


script_dir = os.path.dirname(os.path.abspath(__file__))
yaml_path = os.path.join(script_dir, "../maps/nav_carto_3d_cleaned.yaml")

with open(yaml_path, 'r') as f:
    config = yaml.safe_load(f)

img_path = os.path.join(os.path.dirname(yaml_path), config['image'])

resolution = config['resolution']
origin = config['origin']
negate = config['negate']
occ_thresh = config['occupied_thresh']
free_thresh = config['free_thresh']

image = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
if image is None:
    raise FileNotFoundError(f"Failed to read image at: {img_path}")

height, width = image.shape
print(f"Map size: {width} x {height}")

remove_x_min, remove_x_max = 5.0, 6.0
remove_y_min, remove_y_max = -19.0, -8.0
bridge_x_min, bridge_x_max = 5.5, 8.5
bridge_y_min, bridge_y_max = -23.5, -1.8

points = []

for y in range(height):
    for x in range(width):
        pixel = image[y, x]
        wx = origin[0] + x * resolution
        wy = origin[1] + (height - y - 1) * resolution

        if bridge_x_min < wx < bridge_x_max and bridge_y_min < wy < bridge_y_max:
                    continue
        
        if remove_x_min <= wx <= remove_x_max and remove_y_min <= wy <= remove_y_max:
            wz = 0.00
            points.append([wx, wy, wz])
            continue

        if pixel < occ_thresh * 255:
            for z in np.arange(0, 1.0, 0.1):  
                points.append([wx, wy, z])

        elif pixel > free_thresh * 255:
            wz = 0.00
            points.append([wx, wy, wz])


pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(np.array(points))

pcd_save_path = os.path.join(os.path.dirname(yaml_path), "nav_carto_3d_cleaned.pcd")
o3d.io.write_point_cloud(pcd_save_path, pcd)

print(f"Saved as {pcd_save_path}")
