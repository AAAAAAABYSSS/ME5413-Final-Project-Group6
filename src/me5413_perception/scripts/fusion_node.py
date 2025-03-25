#!/usr/bin/env python
# coding: utf-8

import tf
import json
import math
import rospy
import numpy as np
from tf import transformations
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, CameraInfo
from scipy.spatial.transform import Rotation as R

class FusionNode:
    def __init__(self):
        rospy.init_node("fusion_node", anonymous=True)
    
        self.N = rospy.get_param("~num_closest_points", 10)  
        self.merge_distance = rospy.get_param("~merge_distance", 0.4)

        rospy.loginfo(f"[FusionNode] Using {self.N} closest points based on ray direction")

        self.latest_pointcloud = None
        self.camera_intrinsics = None
        self.yolo_targets = []
        self.tf_listener = tf.TransformListener()

        rospy.Subscriber("/front/camera_info", CameraInfo, self.camera_info_callback)
        rospy.Subscriber("/mid/points", PointCloud2, self.pointcloud_callback)
        rospy.Subscriber("/perception/yolo_targets", String, self.yolo_callback)
        rospy.Subscriber("/gazebo/ground_truth/state", Odometry, self.odom_callback)

        self.target_3d_pub = rospy.Publisher("/perception/yolo_targets_3d", String, queue_size=10)

        self.unmatched_points_cache = []
        self.merged_points_cache = []

        self.rate = rospy.Rate(10)

    def camera_info_callback(self, msg):
        self.camera_intrinsics = np.array(msg.K).reshape(3, 3)

    def pointcloud_callback(self, msg):
        try:
            self.latest_pointcloud = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        except Exception as e:
            rospy.logerr("Failed to convert point cloud: %s" % str(e))

    def yolo_callback(self, msg):
        try:
            self.yolo_targets = json.loads(msg.data)
        except Exception as e:
            rospy.logerr(f"Error parsing YOLO target JSON: {e}")
            
    def odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        self.T_base_to_map = np.eye(4)
        self.P_base_to_map = [position.x, position.y, position.z]
        self.R_base_to_map = R.from_quat([orientation.x, orientation.y, orientation.z, orientation.w]).as_matrix()
        self.T_base_to_map[:3, 3] = self.P_base_to_map
        self.T_base_to_map[:3, :3] = self.R_base_to_map
        self.T_map_to_base = np.linalg.inv(self.T_base_to_map)

    def transform_point(self, point, transform):
        point_homog = np.dot(transform, np.array([point[0], point[1], point[2], 1.0]))
        return point_homog[:3]

    def transform_points(self, points, transform):
        transformed = []
        for pt in points:
            pt_homog = np.array([pt[0], pt[1], pt[2], 1.0])
            pt_trans = transform.dot(pt_homog)
            transformed.append(pt_trans[:3])
        return transformed

    def merge_nearby_labels(self, output, distance_threshold):
        merged = []
        used = [False] * len(output)

        for i in range(len(output)):
            if used[i]:
                continue
            curr = output[i]
            curr_pts = curr["points"]
            matched_flag = False

            for j in range(i + 1, len(output)):
                if used[j]:
                    continue
                other = output[j]
                if curr["label"] != other["label"]:
                    continue

                center_i = np.mean([[p["x"], p["y"], p["z"]] for p in curr_pts], axis=0)
                center_j = np.mean([[p["x"], p["y"], p["z"]] for p in other["points"]], axis=0)
                dist = np.linalg.norm(center_i - center_j)

                if dist < distance_threshold:
                    curr_pts += other["points"]
                    used[j] = True
                    matched_flag = True

            merged.append({
                "label": curr["label"],
                "conf": curr["conf"],
                "u_center": curr["u_center"],
                "v_center": curr["v_center"],
                "matched": matched_flag,
                "points": curr_pts
            })

        return merged

    def process_data(self):
        while not rospy.is_shutdown():
            if self.latest_pointcloud and self.camera_intrinsics is not None and self.yolo_targets:
                try:
                    (trans, rot) = self.tf_listener.lookupTransform("/front_camera_optical", "/velodyne", rospy.Time(0))
                    tf_velo_to_cam = np.dot(transformations.translation_matrix(trans), transformations.quaternion_matrix(rot))

                    (trans_base, rot_base) = self.tf_listener.lookupTransform("/base_link", "/velodyne", rospy.Time(0))
                    tf_velo_to_base = np.dot(transformations.translation_matrix(trans_base), transformations.quaternion_matrix(rot_base))

                    fx = self.camera_intrinsics[0, 0]
                    fy = self.camera_intrinsics[1, 1]
                    cx = self.camera_intrinsics[0, 2]
                    cy = self.camera_intrinsics[1, 2]

                    projected_points = []
                    for pt in self.latest_pointcloud:
                        pt_cam = self.transform_point(pt, tf_velo_to_cam)
                        X, Y, Z = pt_cam

                        if Z <= 0:
                            continue
                        u = int((fx * X / Z) + cx)
                        v = int((fy * Y / Z) + cy)
                        projected_points.append({
                            "u": u,
                            "v": v,
                            "point": pt
                        })

                    output = []
                    for target in self.yolo_targets:
                        u_center = int(target["u_center"])
                        v_center = int(target["v_center"])
                        label = target["label"]
                        conf = target["conf"]

                        dists = []
                        for proj in projected_points:
                            du = proj["u"] - u_center
                            dv = proj["v"] - v_center
                            dist = math.sqrt(du**2 + dv**2)
                            dists.append((dist, proj["point"]))

                        dists.sort(key=lambda x: x[0])
                        nearest_points = [pt for _, pt in dists[:self.N]]
                        nearby_points_base = self.transform_points(nearest_points, tf_velo_to_base)
                        nearby_points_map = self.transform_points(nearby_points_base, self.T_base_to_map) 
            
                        output.append({
                            "label": label,
                            "conf": conf,
                            "u_center": u_center,
                            "v_center": v_center,
                            "points": [{"x": p[0], "y": p[1], "z": p[2]} for p in nearby_points_map]
                        })
                    merged_output = self.merge_nearby_labels(output, self.merge_distance)
                    self.target_3d_pub.publish(json.dumps(merged_output))
                   

                except Exception as e:
                    rospy.logwarn(f"[FusionNode] Failed to process: {e}")

            self.rate.sleep()

if __name__ == "__main__":
    try:
        node = FusionNode()
        node.process_data()
    except rospy.ROSInterruptException:
        pass