#!/usr/bin/env python
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Odometry
import numpy as np
import open3d as o3d  # Used for point cloud processing
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

        # Set save path
        self.save_dir = "pcd_map/"
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)

        # JSON file path (overwritten each time)
        self.json_file = "merged_bboxes.json"

        # Whether to save foreground point cloud
        self.foreground_save = False

        # Subscribe to topics
        rospy.Subscriber("/mid/points", PointCloud2, self.pointcloud_callback)
        rospy.Subscriber("/gazebo/ground_truth/state", Odometry, self.odom_callback)

        self.marker_pub = rospy.Publisher("/bbox_markers", MarkerArray, queue_size=1)

        rospy.loginfo("Subscribed to /mid/points and /gazebo/ground_truth/state topics")

    def odom_callback(self, msg):
        """ Process odometry message and update robot pose """
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

        # Check for abnormal point clouds and avoid saving
        if (
            np.min(transformed_points[:, 0]) < 1.5
            or np.max(transformed_points[:, 0]) > 23.5
            or np.min(transformed_points[:, 1]) < -1.5
        ):
            rospy.logwarn("Detected abnormal point cloud frame due to rotation, skipping save")
            return

        # Segment foreground point cloud
        foreground_points = transformed_points[
            (transformed_points[:, 2] > 2.35)
            & (transformed_points[:, 0] > 1.7)
            & (transformed_points[:, 0] < 23.2)
            & (transformed_points[:, 1] < 20.2)
            & (transformed_points[:, 1] > -1)
        ]

        if len(foreground_points) == 0:
            rospy.logwarn("No foreground points detected, skipping save")
            return

        # Clustering
        fg_pcd = o3d.geometry.PointCloud()
        fg_pcd.points = o3d.utility.Vector3dVector(foreground_points)
        labels = np.array(fg_pcd.cluster_dbscan(eps=0.25, min_points=30))

        # Process detection results
        new_detections = []
        for label in set(labels):
            if label == -1:
                continue
            cluster = fg_pcd.select_by_index(np.where(labels == label)[0])
            aabb = cluster.get_axis_aligned_bounding_box()
            center = aabb.get_center()
            extent = aabb.get_extent()

            # Classify object type based on y-axis position
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

        # Merge detections and save to JSON
        self.all_detections.extend(new_detections)
        self.save_to_json()

        # Publish visualization markers
        self.publish_markers()

        rospy.loginfo(f"Total detected objects: {len(self.all_detections)}")

    def save_to_json(self):
        """ Save merged bbox results to JSON file """
        with open(self.json_file, "w") as f:
            json.dump({"bboxes": self.all_detections}, f, indent=4)

    def publish_markers(self):
        marker_array = MarkerArray()
        for idx, detection in enumerate(self.all_detections):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = detection["type"]
            marker.id = idx
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            # Set position
            marker.pose.position.x = detection["center"][0]
            marker.pose.position.y = detection["center"][1]
            marker.pose.position.z = detection["center"][2]
            marker.pose.orientation.w = 1.0

            # Set size
            marker.scale.x = detection["extent"][0]
            marker.scale.y = detection["extent"][1]
            marker.scale.z = detection["extent"][2]

            # Set color based on type
            if detection["type"] == "box":
                marker.color.r = 1.0  # Red
                marker.color.a = 0.5
            elif detection["type"] == "bridge":
                marker.color.b = 1.0  # Blue
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
