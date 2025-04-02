#!/usr/bin/env python
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Odometry
import numpy as np
import open3d as o3d
import std_msgs.msg
import os
import json
import tf

from sklearn.linear_model import RANSACRegressor
from scipy.spatial.transform import Rotation as R
from visualization_msgs.msg import Marker, MarkerArray


class PointCloudProcessor:
    def __init__(self):
        rospy.init_node("box_detector", anonymous=True)

        # Initialize storage variables
        self.current_pose = None
        self.all_detections = []
        self.next_unique_id = 0
        self.latest_bbox_markers = None
        self.overall_range = rospy.get_param("~overall_range", [-5, -25])
        self.eligible_x_range = rospy.get_param("~eligible_x_range", [-1.5, 23.5])
        self.eligible_y_range = rospy.get_param("~eligible_y_range", [-23.5, -1.5])
        self.eligible_offset = rospy.get_param("~eligible_offset", 0.1)
        self.roi_x = rospy.get_param("~roi_x", [-0.5, 19.5])
        self.roi_y = rospy.get_param("~roi_y", [-22.5, -1.7])
        self.roi_z = rospy.get_param("~roi_z", [-0.3, 99])
        self.box_x_range = rospy.get_param("~box_x_range", [9.75, 19.5])
        self.bridge_x_range = rospy.get_param("~bridge_x_range", [1.5, 9.75])
        self.goal_box_x_range = rospy.get_param("~goal_box_x_range", [-0.5, 1.5])
        self.eps = rospy.get_param("~eps", [0.25, 0.45, 0.9])
        self.min_points = rospy.get_param("~min_points", [30, 30, 20])
        self.box_criteria_xy = rospy.get_param("~box_criteria_xy", [0.05, 0.81])
        self.box_criteria_xyz = rospy.get_param("~box_criteria_xyz", 0.531441)
        self.box_criteria_z = rospy.get_param("~box_criteria_z", 0.6)
        self.merge_volume_threshold = rospy.get_param("~merge_volume_threshold", 0.05)
        self.max_iterations = rospy.get_param("~max_iterations", 5)
        self.center_dist_threshold = rospy.get_param("~center_dist_threshold", 0.5)
        
        # Set save directory
        self.save_dir = "/home/yuku/1workspace/NUS/ME5413/ME5413-Final-Project-Group6/pcd/"
        # if not os.path.exists(self.save_dir):
        #     os.makedirs(self.save_dir)

        # JSON file path (overwrites each time)
        # self.json_file = os.path.join(self.save_dir, "merged_bboxes.json")

        # Whether to save foreground point cloud
        self.foreground_save = False

        # Subscribe to topics
        rospy.Subscriber("/mid/points", PointCloud2, self.pointcloud_callback)
        # rospy.Subscriber("/gazebo/ground_truth/state", Odometry, self.odom_callback)

        self.marker_pub = rospy.Publisher("/perception/marker/bbox_markers", MarkerArray, queue_size=1)
        
        # Add TF listener
        self.tf_listener = tf.TransformListener()

        rospy.loginfo("Subscribed to /mid/points and /gazebo/ground_truth/state topics")

    # def odom_callback(self, msg):
    #     """ Process odometry messages and update robot pose """
    #     position = msg.pose.pose.position
    #     orientation = msg.pose.pose.orientation

    #     # Compute 4x4 homogeneous transformation matrix (map -> base_link)
    #     translation = np.array([position.x, position.y, position.z])
    #     rotation = R.from_quat(
    #         [orientation.x, orientation.y, orientation.z, orientation.w]
    #     ).as_matrix()

    #     self.current_pose = np.eye(4)
    #     self.current_pose[:3, :3] = rotation
    #     self.current_pose[:3, 3] = translation

    def update_pose_from_tf(self):
        """ Get transform from base_link to map and update current_pose """
        try:
            # (trans, rot) = self.tf_listener.lookupTransform('map', 'base_link', rospy.Time(0))
            (trans, rot) = self.tf_listener.lookupTransform('map', 'velodyne', rospy.Time(0))
            rotation_matrix = R.from_quat(rot).as_matrix()
            translation = np.array(trans)

            T_base_to_map = np.eye(4)
            T_base_to_map[:3, :3] = rotation_matrix
            T_base_to_map[:3, 3] = translation

            self.current_pose = T_base_to_map  # Base to map transform
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("TF lookup failed")

    def pointcloud_callback(self, msg):
        """ Process point cloud data """
        self.update_pose_from_tf()

        if self.current_pose is None:
            rospy.logwarn("Waiting for pose information...")
            return
        
        # During the bridge crossing
        if self.current_pose[0, 3] <= self.bridge_x_range[1] and self.current_pose[0, 3] >= self.bridge_x_range[0]:
            self.marker_pub.publish(self.latest_bbox_markers)
            rospy.logwarn("Crossing bridge, stop detection ...")
            return

        # Read point cloud data
        point_list = [list(point[:3]) for point in pc2.read_points(msg, skip_nans=True)]
        point_cloud = np.array(point_list, dtype=np.float32)
        rospy.logdebug("Received point cloud with %d points" % len(point_cloud))

        # Coordinate transformation (base_link -> map)
        num_points = point_cloud.shape[0]
        ones = np.ones((num_points, 1), dtype=np.float32)
        homogeneous_points = np.hstack((point_cloud, ones))
        transformed_points = (self.current_pose @ homogeneous_points.T).T[:, :3]

        # Preprocessing: Remove out-of-range points
        transformed_points = transformed_points[
            (transformed_points[:, 0] >= self.overall_range[0]) & (transformed_points[:, 1] >= self.overall_range[1])
        ]

        # Detect abnormal point clouds and skip saving
        if (
            np.min(transformed_points[:, 0]) < (self.eligible_x_range[0] - self.eligible_offset)
            or np.max(transformed_points[:, 0]) > (self.eligible_x_range[1] + self.eligible_offset)
            or np.min(transformed_points[:, 1]) < (self.eligible_y_range[0] - self.eligible_offset)
            or np.max(transformed_points[:, 1]) > (self.eligible_y_range[1] + self.eligible_offset)
        ):
            
            # if np.min(transformed_points[:, 0]) < self.eligible_x_range[0]:
            #     rospy.logwarn(f"1 {np.min(transformed_points[:, 0]) - self.eligible_x_range[0]}")
            # if np.max(transformed_points[:, 0]) > self.eligible_x_range[1]:
            #     rospy.logwarn(f"2 {np.max(transformed_points[:, 0]) - self.eligible_x_range[1]}")
            # if np.min(transformed_points[:, 1]) < self.eligible_y_range[0]:
            #     rospy.logwarn(f"3 {np.min(transformed_points[:, 1]) - self.eligible_y_range[0]}")
            # if np.max(transformed_points[:, 1]) > self.eligible_y_range[1]:
            #     rospy.logwarn(f"4 {np.min(transformed_points[:, 1]) - self.eligible_y_range[1]}")

            # Perform clustering
            fg_pcd = o3d.geometry.PointCloud()
            fg_pcd.points = o3d.utility.Vector3dVector(transformed_points)

            if self.foreground_save:
                # Generate PCD file
                timestamp = msg.header.stamp.to_sec()
                spcd_filename = os.path.join(self.save_dir, f"foreground%.6f_error.pcd" % timestamp)
                o3d.io.write_point_cloud(spcd_filename, fg_pcd)

            self.marker_pub.publish(self.latest_bbox_markers)
            rospy.logwarn("Detected abnormal point cloud frame due to rotation, skipped saving")

            return
            # rospy.logwarn("Detected abnormal point cloud frame due to rotation")
            # sample_mask = (transformed_points[:, 0] >= self.ref_x[0]) & (transformed_points[:, 0] < self.ref_x[0]+1) & (transformed_points[:, 1] >= self.ref_y[0]) & (transformed_points[:, 1] < self.ref_y[0]+1)
            # sample_pcd = transformed_points[sample_mask]
            # delta_angle = self.detect_wall_angle_by_ransac(sample_pcd)
            # rospy.logwarn(f"!!!!!{delta_angle}, {np.rad2deg(delta_angle)}")
            # transformed_points = self.correct_pointcloud_rotation(delta_angle, transformed_points)

            # if (
            #     np.min(transformed_points[:, 0]) < self.ref_x[0]
            #     or np.max(transformed_points[:, 0]) > self.ref_x[1]
            #     or np.min(transformed_points[:, 1]) < self.ref_y[0]
            # ):
            #     rospy.logwarn("???")

        # Segment foreground point cloud
        foreground_points = transformed_points[
            (transformed_points[:, 0] > self.roi_x[0])
            & (transformed_points[:, 0] < self.roi_x[1])
            & (transformed_points[:, 1] > self.roi_y[0])
            & (transformed_points[:, 1] < self.roi_y[1])
            & (transformed_points[:, 2] > self.roi_z[0])
            & (transformed_points[:, 2] < self.roi_z[1])
        ]

        if len(foreground_points) == 0:
            rospy.logwarn("No foreground points detected, skipping save")
            return

        # Perform clustering
        fg_pcd = o3d.geometry.PointCloud()
        fg_pcd.points = o3d.utility.Vector3dVector(foreground_points)
        # labels = np.array(fg_pcd.cluster_dbscan(eps=0.25, min_points=30))

        # Perform clustering by y-axis segmentation
        foreground_points = np.array(foreground_points)
        all_labels = -np.ones(len(foreground_points), dtype=int)    # Initialize all label to be -1 (no cluster result)
        label_offset = 0  # make sure the labels of each cluster do not overlap

        # Define segments based on y-axis
        segments = [
            {"x_range": self.box_x_range, "eps": self.eps[0], "min_points": self.min_points[0]},
            {"x_range": self.bridge_x_range, "eps": self.eps[1], "min_points": self.min_points[1]},
            {"x_range": self.goal_box_x_range, "eps": self.eps[2], "min_points": self.min_points[2]},
        ]

        for segment in segments:
            x_min, x_max = segment["x_range"][0], segment["x_range"][1]
            eps = segment["eps"]
            min_points = segment["min_points"]

            # Filter points by x-value
            mask = (foreground_points[:, 0] >= x_min) & (foreground_points[:, 0] < x_max)
            segment_points = foreground_points[mask]

            if len(segment_points) == 0:
                continue

            # Perform clustering
            sub_pcd = o3d.geometry.PointCloud()
            sub_pcd.points = o3d.utility.Vector3dVector(segment_points)
            sub_labels = np.array(sub_pcd.cluster_dbscan(eps=eps, min_points=min_points))

            # Write the sub-cluster label to the original all_labels (record which points in the original array)
            segment_indices = np.where(mask)[0]
            for i, idx in enumerate(segment_indices):
                if sub_labels[i] != -1:
                    all_labels[idx] = sub_labels[i] + label_offset

            # Update label_offset to avoid label duplication
            if sub_labels.max() != -1:
                label_offset += sub_labels.max() + 1


            if self.foreground_save:
                # Generate PCD file
                timestamp = msg.header.stamp.to_sec()
                spcd_filename = os.path.join(self.save_dir, f"foreground%.6f_{x_min}.pcd" % timestamp)
                o3d.io.write_point_cloud(spcd_filename, sub_pcd)

        # Update the labels variable with all_labels
        labels = all_labels

        if self.foreground_save:
            # Generate PCD file
            timestamp = msg.header.stamp.to_sec()
            pcd_filename = os.path.join(self.save_dir, "foreground%.6f.pcd" % timestamp)
            o3d.io.write_point_cloud(pcd_filename, fg_pcd)
            rospy.loginfo("Saved transformed point cloud to %s" % pcd_filename)

        # Process detection results
        new_detections = []
        for label in set(labels):
            if label == -1:
                continue
            cluster = fg_pcd.select_by_index(np.where(labels == label)[0])
            aabb = cluster.get_axis_aligned_bounding_box()
            center = aabb.get_center()
            extent = aabb.get_extent()

            # Classify objects based on y-axis position
            if center[0] >= self.bridge_x_range[0] and center[0] <= self.bridge_x_range[1]:
                obj_type = "bridge"
            # elif extent[0] * extent[1] >= 0.6 * 0.1 and extent[0] * extent[1] * extent[2] <= 0.81 * 0.81 * 0.81 and extent[2] >= 0.6:
            elif ((extent[0] >= self.box_criteria_xy[0] and extent[0] <= self.box_criteria_xy[1]) or (extent[1] >= self.box_criteria_xy[0] and extent[1] <= self.box_criteria_xy[1])) and extent[0] * extent[1] * extent[2] <= self.box_criteria_xyz and extent[2] >= self.box_criteria_z:
                obj_type = "box"
            else:
                obj_type = "unknown"
                
            new_detections.append(
                {   
                    "id": self.next_unique_id,
                    "type": obj_type,
                    "center": center.tolist(),
                    "min_bound": aabb.min_bound.tolist(),
                    "max_bound": aabb.max_bound.tolist(),
                    "extent": extent.tolist(),       
                }
            )
            self.next_unique_id += 1

        # Merge bounding boxes iteratively
        self.all_detections = self.merge_bboxes(self.all_detections + new_detections)

        # # Save to JSON
        # self.save_to_json()

        # Publish visualization MarkerArray
        self.publish_markers()

        rospy.loginfo(f"Current number of objects in all_detections: {len(self.all_detections)}")

    def merge_bboxes(self, bboxes):
        """ Iteratively merge bounding boxes with high IoU """
        threshold = self.merge_volume_threshold
        max_iterations = self.max_iterations

        # Ensure the structure of bboxes is correct
        bboxes = [b for b in bboxes if isinstance(b, dict) and "type" in b and b["type"] in ["box", "bridge"]]

        for iteration in range(max_iterations):
            rospy.loginfo(f"Iteration {iteration + 1} for merging...")

            merged_bboxes = []
            merge_occurred = False
            remaining_bboxes = list(bboxes)  # Copy to avoid modifying the original list

            while remaining_bboxes:
                base_bbox = remaining_bboxes.pop(0)
                to_merge = [base_bbox]
                new_remaining = []

                for bbox in remaining_bboxes:
                    if base_bbox["type"] == bbox["type"]:
                        if bbox["type"] == "box":
                            center_dist = np.linalg.norm(np.array(base_bbox["center"]) - np.array(bbox["center"]))
                            if self.calculate_iou(base_bbox, bbox) > threshold and center_dist <= self.center_dist_threshold:
                                to_merge.append(bbox)
                                merge_occurred = True
                            else:
                                new_remaining.append(bbox)
                        elif bbox["type"] == "bridge":
                            # Directly merge
                            center_dist = np.linalg.norm(np.array(base_bbox["center"]) - np.array(bbox["center"]))
                            if center_dist <= (self.center_dist_threshold + 0.5):   # give more offset
                                to_merge.append(bbox)
                                merge_occurred = True
                    else:
                        new_remaining.append(bbox)

                # Merge all bounding boxes with IoU above the threshold
                merged_bbox = to_merge[0]
                for bbox in to_merge[1:]:
                    merged_bbox = self.merge_two_bboxes(merged_bbox, bbox)

                merged_bboxes.append(merged_bbox)
                remaining_bboxes = new_remaining  # Update unmerged bboxes

            if not merge_occurred:
                rospy.loginfo("No new merges occurred, exiting early.")
                break

            bboxes = merged_bboxes  # Update bboxes for the next iteration

        return bboxes
    
    # IoU calculation function
    def calculate_iou(self, box1, box2):
        # Compute intersection
        inter_min = np.maximum(box1['min_bound'], box2['min_bound'])
        inter_max = np.minimum(box1['max_bound'], box2['max_bound'])
        inter_dim = np.maximum(0, inter_max - inter_min)
        inter_volume = np.prod(inter_dim)
        
        if inter_volume == np.prod(inter_min):
            return 1 + self.merge_volume_threshold

        # Compute union
        vol1 = np.prod(np.array(box1['max_bound']) - np.array(box1['min_bound']))
        vol2 = np.prod(np.array(box2['max_bound']) - np.array(box2['min_bound']))
        union_volume = vol1 + vol2 - inter_volume

        # Compute IoU
        iou = inter_volume / union_volume if union_volume > 0 else 0
        return iou

    def merge_two_bboxes(self, box1, box2):
        """ Merge two bounding boxes """
        if not isinstance(box1, dict) or not isinstance(box2, dict):
            rospy.logwarn("The bounding boxes to be merged are not valid dictionaries, skipping.")
            return box1  # Avoid merging errors, return original bbox

        min_bound = np.minimum(box1["min_bound"], box2["min_bound"]).tolist()
        max_bound = np.maximum(box1["max_bound"], box2["max_bound"]).tolist()
        center = ((np.array(min_bound) + np.array(max_bound)) / 2).tolist()
        extent = (np.array(max_bound) - np.array(min_bound)).tolist()

        return {
            "id": box1["id"], 
            "type": box1["type"],
            "center": center,
            "min_bound": min_bound,
            "max_bound": max_bound,
            "extent": extent
        }
    # def save_to_json(self):
    #     """ Publish markers for visualization """
    #     # self.all_detections.sort(key=lambda d: (d["center"][0], d["center"][1], d["center"][2]))

    #     with open(self.json_file, "w") as f:
    #         json.dump({"bboxes": self.all_detections}, f, indent=4)

    def publish_markers(self):
        marker_array = MarkerArray()
        for detection in self.all_detections:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = detection["type"]
            marker.id = detection["id"]  
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            # Set position and orientation
            marker.pose.position.x = detection["center"][0]
            marker.pose.position.y = detection["center"][1]
            marker.pose.position.z = detection["center"][2]
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0

            # Set size
            marker.scale.x = detection["extent"][0]
            marker.scale.y = detection["extent"][1]
            marker.scale.z = detection["extent"][2]

            if detection["type"] == "box":
                marker.color.r = 1.0 
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 0.5
            elif detection["type"] == "bridge":
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0 
                marker.color.a = 0.5
            else:
                marker.color.r = 0.5
                marker.color.g = 0.5
                marker.color.b = 0.5
                marker.color.a = 0.5
                
            marker.lifetime = rospy.Duration(0.5)  
            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)
        self.latest_bbox_markers = marker_array

    def run(self):
        """ Run ROS listener """
        rospy.spin()


if __name__ == "__main__":
    processor = PointCloudProcessor()
    processor.run()