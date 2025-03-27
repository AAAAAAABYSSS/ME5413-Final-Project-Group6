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

from scipy.spatial.transform import Rotation as R
from visualization_msgs.msg import Marker, MarkerArray


class PointCloudProcessor:
    def __init__(self):
        rospy.init_node("box_detector", anonymous=True)

        # Initialize storage variables
        self.current_pose = None
        self.all_detections = []

        # Set save directory
        self.save_dir = "./pcd_map/"
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)

        # JSON file path (overwrites each time)
        self.json_file = os.path.join(self.save_dir, "merged_bboxes.json")


        # Whether to save foreground point cloud
        self.foreground_save = False

        # Subscribe to topics
        rospy.Subscriber("/mid/points", PointCloud2, self.pointcloud_callback)
        rospy.Subscriber("/gazebo/ground_truth/state", Odometry, self.odom_callback)

        self.marker_pub = rospy.Publisher("/perception/marker/bbox_markers", MarkerArray, queue_size=1)

        rospy.loginfo("Subscribed to /mid/points and /gazebo/ground_truth/state topics")

    def odom_callback(self, msg):
        """ Process odometry messages and update robot pose """
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        # Compute 4x4 homogeneous transformation matrix (map -> base_link)
        translation = np.array([position.x, position.y, position.z])
        rotation = R.from_quat(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        ).as_matrix()

        self.current_pose = np.eye(4)
        self.current_pose[:3, :3] = rotation
        self.current_pose[:3, 3] = translation


    def pointcloud_callback(self, msg):
        """ Process point cloud data """
        if self.current_pose is None:
            rospy.logwarn("Waiting for pose information...")
            return

        # Read point cloud data
        point_list = [list(point[:3]) for point in pc2.read_points(msg, skip_nans=True)]
        point_cloud = np.array(point_list, dtype=np.float32)
        rospy.loginfo("Received point cloud with %d points" % len(point_cloud))

        # Coordinate transformation (base_link -> map)
        num_points = point_cloud.shape[0]
        ones = np.ones((num_points, 1), dtype=np.float32)
        homogeneous_points = np.hstack((point_cloud, ones))
        transformed_points = (self.current_pose @ homogeneous_points.T).T[:, :3]

        # Preprocessing: Remove out-of-range points
        transformed_points = transformed_points[
            (transformed_points[:, 0] <= 25) & (transformed_points[:, 1] >= -5)
        ]

        # Detect abnormal point clouds and skip saving
        if (
            np.min(transformed_points[:, 0]) < 1.5
            or np.max(transformed_points[:, 0]) > 23.5
            or np.min(transformed_points[:, 1]) < -1.5
        ):
            rospy.logwarn("Detected abnormal point cloud frame due to rotation, skipped saving")
            return

        # Segment foreground point cloud
        foreground_points = transformed_points[
            (transformed_points[:, 2] > 2.35)
            & (transformed_points[:, 0] > 1.7)
            & (transformed_points[:, 0] < 23.2)
            & (transformed_points[:, 1] < 20.2)
            # & (transformed_points[:, 1] > 4.9)
            & (transformed_points[:, 1] > -1)
        ]

        if len(foreground_points) == 0:
            rospy.logwarn("No foreground points detected, skipping save")
            return

        # Perform clustering
        fg_pcd = o3d.geometry.PointCloud()
        fg_pcd.points = o3d.utility.Vector3dVector(foreground_points)
        labels = np.array(fg_pcd.cluster_dbscan(eps=0.25, min_points=30))

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
            if center[1] <= 9 and center[1] >= 1.5:
                obj_type = "bridge"
            elif extent[0] * extent[1] >= 0.6 * 0.2:
                obj_type = "box"
            else:
                obj_type = "unknown"
                
            new_detections.append(
                {
                    "type": obj_type,
                    "center": center.tolist(),
                    "min_bound": aabb.min_bound.tolist(),
                    "max_bound": aabb.max_bound.tolist(),
                    "extent": extent.tolist(),       
                }
            )

        # Merge bounding boxes iteratively
        self.all_detections = self.merge_bboxes(self.all_detections + new_detections)

        # Save to JSON
        self.save_to_json()

        # Publish visualization MarkerArray
        self.publish_markers()

        rospy.loginfo(f"Current number of objects in all_detections: {len(self.all_detections)}")

    def merge_bboxes(self, bboxes, threshold=0.05, max_iterations=5):
        """ Iteratively merge bounding boxes with high IoU """
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
                            if self.calculate_iou(base_bbox, bbox) > threshold:
                                to_merge.append(bbox)
                                merge_occurred = True
                            else:
                                new_remaining.append(bbox)
                        elif bbox["type"] == "bridge":
                            # Directly merge
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

        return {"type": box1["type"], "center": center, "min_bound": min_bound, "max_bound": max_bound, "extent": extent}
    
    def save_to_json(self):
        """ Publish markers for visualization """
        with open(self.json_file, "w") as f:
            json.dump({"bboxes": self.all_detections}, f, indent=4)

    def publish_markers(self):
        marker_array = MarkerArray()
        for idx, detection in enumerate(self.all_detections):
            marker = Marker()
            marker.header.frame_id = "map"  
            # marker.header.frame_id = "map" 
            marker.header.stamp = rospy.Time.now()
            marker.ns = detection["type"]
            marker.id = idx
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

    def run(self):
        """ Run ROS listener """
        rospy.spin()


if __name__ == "__main__":
    processor = PointCloudProcessor()
    processor.run()
