#!/usr/bin/env python3
#coding=utf-8
import rospy
import cv2
import numpy as np
import open3d as o3d
import tf
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from cv_bridge import CvBridge
import sensor_msgs.point_cloud2 as pc2
from tf import transformations
import struct

# Initialize node
rospy.init_node('projection_node', anonymous=True)
bridge = CvBridge()

# Variable storage
latest_image = None
latest_pointcloud = None
latest_header = None
camera_intrinsics = None
distortion_coefficients = None
tf_listener = tf.TransformListener()
min_depth, max_depth = 0, 0

# Camera info callback
def camera_info_callback(msg):
    global camera_intrinsics, distortion_coefficients
    camera_intrinsics = np.array(msg.K).reshape(3, 3)
    distortion_coefficients = np.array(msg.D)

# Image callback
def image_callback(msg):
    global latest_image
    try:
        latest_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except Exception as e:
        rospy.logerr("Failed to convert image: %s" % str(e))

# Point cloud callback
def pointcloud_callback(msg):
    global latest_pointcloud, latest_header, min_depth, max_depth
    try:
        latest_pointcloud = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        distances = np.linalg.norm(latest_pointcloud, axis=1)
        min_depth = np.min(distances)
        max_depth = np.max(distances)
        latest_header = msg.header
    except Exception as e:
        rospy.logerr("Failed to convert point cloud: %s" % str(e))

# Transform point using transformation matrix
def transform_point(point, transform):
    tf_point = np.dot(transform, np.array([point[0], point[1], point[2], 1.0]))
    return tf_point[:3]

def get_color(cur_depth):
    scale = (max_depth - min_depth) / 10
    if cur_depth < min_depth:
        color = (0, 0, 255)  # 蓝色
    elif cur_depth < min_depth + scale:
        green = int((cur_depth - min_depth) / scale * 255)
        color = (0, green, 255)  # 蓝到黄渐变
    elif cur_depth < min_depth + scale * 2:
        red = int((cur_depth - min_depth - scale) / scale * 255)
        color = (0, 255, 255 - red)  # 黄到红渐变
    elif cur_depth < min_depth + scale * 4:
        blue = int((cur_depth - min_depth - scale * 2) / scale * 255)
        color = (blue, 255, 0)  # 红到绿渐变
    elif cur_depth < min_depth + scale * 7:
        green = int((cur_depth - min_depth - scale * 4) / scale * 255)
        color = (255, 255 - green, 0)  # 绿到黄渐变
    elif cur_depth < min_depth + scale * 10:
        blue = int((cur_depth - min_depth - scale * 7) / scale * 255)
        color = (255, 0, blue)  # 黄到蓝渐变
    else:
        color = (255, 0, 255)  # 紫色

    return np.clip(color, 0, 255).astype(np.uint8)  # 确保颜色值在 0~255 并转换为 uint8

# Project point cloud onto image
def project_pointcloud_to_image():
    global min_depth, max_depth
    if latest_image is None or latest_pointcloud is None or camera_intrinsics is None:
        return
    
    try:
        # Get transformation matrix from LiDAR to camera frame
        (trans, rot) = tf_listener.lookupTransform("/front_camera_optical", "/velodyne", rospy.Time(0))
        transform_matrix = np.dot(transformations.translation_matrix(trans), transformations.quaternion_matrix(rot))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.logwarn("Failed to get transformation: %s" % str(e))
        return
    
    projected_image = latest_image.copy()
    depth_map = np.zeros((latest_image.shape[0], latest_image.shape[1]), dtype=np.float32)
    
    for point in latest_pointcloud:
        transformed_point = transform_point(point, transform_matrix)
        x, y, z = transformed_point
        if z <= 0:
            continue
        
        uv = np.dot(camera_intrinsics, np.array([x, y, z]))
        u, v = int(uv[0] / uv[2]), int(uv[1] / uv[2])
        
        if 0 <= u < projected_image.shape[1] and 0 <= v < projected_image.shape[0]:
            color = get_color(z)
            projected_image[v, u] = color

    return projected_image

# ROS publisher for projection image
projection_pub = rospy.Publisher("/projected_image", Image, queue_size=10)

# Subscribe to topics
rospy.Subscriber("/front/camera_info", CameraInfo, camera_info_callback)
rospy.Subscriber("/front/image_raw", Image, image_callback)
rospy.Subscriber("/mid/points", PointCloud2, pointcloud_callback)

rate = rospy.Rate(10)
while not rospy.is_shutdown():
    projected_img = project_pointcloud_to_image()
    if projected_img is not None:
        projection_pub.publish(bridge.cv2_to_imgmsg(projected_img, "bgr8"))
    rate.sleep()
