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
from sensor_msgs.msg import PointField
from tf import transformations
import struct

# Initialize node
# Paint the original point clouds with corresponding color
rospy.init_node('fusion_node', anonymous=True)
bridge = CvBridge()

# Variable storage
latest_image = None
latest_pointcloud = None
latest_header = None
camera_intrinsics = None
distortion_coefficients = None
tf_listener = tf.TransformListener()

# Camera info callback
def camera_info_callback(msg):
    global camera_intrinsics, distortion_coefficients
    camera_intrinsics = np.array(msg.K).reshape(3, 3)
    distortion_coefficients = np.array(msg.D)
    rospy.loginfo("Camera intrinsic matrix loaded: \n%s" % camera_intrinsics)

# Undistort image
def undistort_image(image):
    if camera_intrinsics is not None and distortion_coefficients is not None:
        h, w = image.shape[:2]
        new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(camera_intrinsics, distortion_coefficients, (w, h), 1, (w, h))
        undistorted_image = cv2.undistort(image, camera_intrinsics, distortion_coefficients, None, new_camera_matrix)
        return undistorted_image
    return image

# Image callback
def image_callback(msg):
    global latest_image
    try:
        latest_image = undistort_image(bridge.imgmsg_to_cv2(msg, "bgr8"))
    except Exception as e:
        rospy.logerr("Failed to convert image: %s" % str(e))

# Point cloud callback
def pointcloud_callback(msg):
    global latest_pointcloud, latest_header
    try:
        latest_pointcloud = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        latest_header = msg.header
    except Exception as e:
        rospy.logerr("Failed to convert point cloud: %s" % str(e))

# Transform point using transformation matrix
def transform_point(point, transform):
    tf_point = np.dot(transform, np.array([point[0], point[1], point[2], 1.0]))
    return tf_point[:3]

# Convert RGB to float32 format
def rgb_to_float(r, g, b):
    return struct.unpack('f', struct.pack('I', (r << 16) | (g << 8) | b))[0]

# Subscribe to topics
rospy.Subscriber("/front/camera_info", CameraInfo, camera_info_callback)
rospy.Subscriber("/front/image_raw", Image, image_callback)
rospy.Subscriber("/mid/points", PointCloud2, pointcloud_callback)

# Publisher for fused point cloud
pub = rospy.Publisher("/fused_pointcloud", PointCloud2, queue_size=10)

rate = rospy.Rate(30)
while not rospy.is_shutdown():
    if latest_image is not None and latest_pointcloud is not None and camera_intrinsics is not None:
        try:
            # Transformation path: /velodyne -> /base_link -> /front_camera_optical
            (trans1, rot1) = tf_listener.lookupTransform("/base_link", "/velodyne", rospy.Time(0))
            (trans2, rot2) = tf_listener.lookupTransform("/front_camera_optical", "/base_link", rospy.Time(0))
            
            # Construct transformation matrices using quaternion
            transform1 = np.dot(transformations.translation_matrix(trans1), transformations.quaternion_matrix(rot1))
            transform2 = np.dot(transformations.translation_matrix(trans2), transformations.quaternion_matrix(rot2))
            
            # Final transformation matrix (Note the order!)
            transform_matrix = np.dot(transform2, transform1)
            rospy.loginfo("Combined transformation matrix:\n%s" % transform_matrix)

            points_with_rgb = []
            for point in latest_pointcloud:
                # Transform point
                transformed_point = transform_point(point, transform_matrix)
                x, y, z = transformed_point
                x0, y0, z0 = point

                # Project to image plane
                if z > 0:  # Process points with z > 0 only
                    uv = np.dot(camera_intrinsics, np.array([x, y, z]))
                    u, v = int(uv[0] / uv[2]), int(uv[1] / uv[2])
                else:
                    continue

                # Get RGB information
                if 0 <= u < latest_image.shape[1] and 0 <= v < latest_image.shape[0]:
                    b, g, r = latest_image[v, u]
                    points_with_rgb.append([x0, y0, z0, rgb_to_float(r, g, b)])
                else:
                    points_with_rgb.append([x0, y0, z0, rgb_to_float(255, 255, 255)])

            # Define point cloud fields
            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1)
            ]

            # Publish point cloud
            fused_cloud_msg = pc2.create_cloud(latest_header, fields, points_with_rgb)
            pub.publish(fused_cloud_msg)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn("Failed to transform coordinates: %s" % str(e))
    
    rate.sleep()
