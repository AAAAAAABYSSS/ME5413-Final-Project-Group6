#!/usr/bin/env python
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import numpy as np
import open3d as o3d
import tf

from scipy.spatial.transform import Rotation as R
from visualization_msgs.msg import Marker, MarkerArray


class PointCloudClustering:
    def __init__(self):
        rospy.init_node("target_detector", anonymous=True)

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
        self.qualified_extent = rospy.get_param("~qualified_extent", 0.6)
        self.qualified_center_z = rospy.get_param("~qualified_center_z", [0.25, 0.55])

        # Subscribe to topics
        rospy.Subscriber("/mid/points", PointCloud2, self.pointcloud_callback)

        # Add TF listener
        self.tf_listener = tf.TransformListener()

        # Publish detection results
        self.marker_pub = rospy.Publisher(
            "/perception/marker/bbox_markers_pending", MarkerArray, queue_size=1
        )

        rospy.loginfo(
            "Subscribed to /mid/points and /tf topics, start point cloud detection."
        )

    def update_pose_from_tf(self):
        """Get transform from velodyne to map and update current_pose"""
        try:
            # Velodyne to map transform
            (trans, rot) = self.tf_listener.lookupTransform(
                "map", "velodyne", rospy.Time(0)
            )
            rotation_matrix = R.from_quat(rot).as_matrix()
            translation = np.array(trans)

            T_velodyne_to_map = np.eye(4)
            T_velodyne_to_map[:3, :3] = rotation_matrix
            T_velodyne_to_map[:3, 3] = translation

            self.current_pose = T_velodyne_to_map
        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ):
            rospy.logwarn("TF lookup failed")

    def pointcloud_callback(self, msg):
        """Process and detect point cloud data"""
        self.update_pose_from_tf()

        if self.current_pose is None:
            rospy.logwarn("Waiting for pose information...")
            return

        # During the bridge crossing
        if (
            self.current_pose[0, 3] <= self.bridge_x_range[1]
            and self.current_pose[0, 3] >= self.bridge_x_range[0]
        ):
            self.marker_pub.publish(self.latest_bbox_markers)
            rospy.logwarn("Crossing bridge, stop detection ...")
            return

        # Read point cloud data
        point_list = [list(point[:3]) for point in pc2.read_points(msg, skip_nans=True)]
        point_cloud = np.array(point_list, dtype=np.float32)
        rospy.logdebug("Received point cloud with %d points" % len(point_cloud))

        # Coordinate transformation (velodyne -> map)
        num_points = point_cloud.shape[0]
        ones = np.ones((num_points, 1), dtype=np.float32)
        homogeneous_points = np.hstack((point_cloud, ones))
        transformed_points = (self.current_pose @ homogeneous_points.T).T[:, :3]

        # Preprocessing: Remove out-of-range points
        transformed_points = transformed_points[
            (transformed_points[:, 0] >= self.overall_range[0])
            & (transformed_points[:, 1] >= self.overall_range[1])
        ]

        # Detect abnormal point clouds and skip saving
        if (
            np.min(transformed_points[:, 0])
            < (self.eligible_x_range[0] - self.eligible_offset)
            or np.max(transformed_points[:, 0])
            > (self.eligible_x_range[1] + self.eligible_offset)
            or np.min(transformed_points[:, 1])
            < (self.eligible_y_range[0] - self.eligible_offset)
            or np.max(transformed_points[:, 1])
            > (self.eligible_y_range[1] + self.eligible_offset)
        ):
            if np.min(transformed_points[:, 0]) < self.eligible_x_range[0]:
                rospy.logdebug(
                    f"1 {np.min(transformed_points[:, 0]) - self.eligible_x_range[0]}"
                )
            if np.max(transformed_points[:, 0]) > self.eligible_x_range[1]:
                rospy.logdebug(
                    f"2 {np.max(transformed_points[:, 0]) - self.eligible_x_range[1]}"
                )
            if np.min(transformed_points[:, 1]) < self.eligible_y_range[0]:
                rospy.logdebug(
                    f"3 {np.min(transformed_points[:, 1]) - self.eligible_y_range[0]}"
                )
            if np.max(transformed_points[:, 1]) > self.eligible_y_range[1]:
                rospy.logdebug(
                    f"4 {np.min(transformed_points[:, 1]) - self.eligible_y_range[1]}"
                )

            rospy.logwarn("Abnormal point cloud, skip ...")
            return

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
            rospy.logwarn("No foreground points detected, skip")
            return

        # Perform clustering
        fg_pcd = o3d.geometry.PointCloud()
        fg_pcd.points = o3d.utility.Vector3dVector(foreground_points)

        # Perform clustering by y-axis segmentation
        foreground_points = np.array(foreground_points)
        # Initialize all label to be -1 (no cluster result)
        all_labels = -np.ones(len(foreground_points), dtype=int)
        # Make sure the labels of each cluster do not overlap
        label_offset = 0

        # Define segments based on x-axis
        segments = [
            {
                "x_range": self.box_x_range,
                "eps": self.eps[0],
                "min_points": self.min_points[0],
            },
            {
                "x_range": self.bridge_x_range,
                "eps": self.eps[1],
                "min_points": self.min_points[1],
            },
            {
                "x_range": self.goal_box_x_range,
                "eps": self.eps[2],
                "min_points": self.min_points[2],
            },
        ]

        for segment in segments:
            x_min, x_max = segment["x_range"][0], segment["x_range"][1]
            eps = segment["eps"]
            min_points = segment["min_points"]

            # Filter points by x-value
            mask = (foreground_points[:, 0] >= x_min) & (
                foreground_points[:, 0] < x_max
            )
            segment_points = foreground_points[mask]

            if len(segment_points) == 0:
                continue

            # Perform clustering
            sub_pcd = o3d.geometry.PointCloud()
            sub_pcd.points = o3d.utility.Vector3dVector(segment_points)
            sub_labels = np.array(
                sub_pcd.cluster_dbscan(eps=eps, min_points=min_points)
            )

            # Write the sub-cluster label to the original all_labels (record which points in the original array)
            segment_indices = np.where(mask)[0]
            for i, idx in enumerate(segment_indices):
                if sub_labels[i] != -1:
                    all_labels[idx] = sub_labels[i] + label_offset

            # Update label_offset to avoid label duplication
            if sub_labels.max() != -1:
                label_offset += sub_labels.max() + 1

        # Update the labels variable with all_labels
        labels = all_labels

        # Process detection results
        new_detections = []
        for label in set(labels):
            if label == -1:
                continue
            cluster = fg_pcd.select_by_index(np.where(labels == label)[0])
            aabb = cluster.get_axis_aligned_bounding_box()
            center = aabb.get_center()
            extent = aabb.get_extent()

            # Classify objects based on x-axis position
            if (
                center[0] >= self.bridge_x_range[0]
                and center[0] <= self.bridge_x_range[1]
                and center[1] <= self.roi_y[1] - self.eligible_offset
            ):
                obj_type = "bridge"
            # Classify objects based on extent and exclude wall errors
            elif (center[0] >= self.bridge_x_range[1]) or (
                (center[0] <= self.bridge_x_range[0])
                and (center[1] <= self.overall_range[0])
            ):
                if (
                    (
                        (
                            extent[0] >= self.box_criteria_xy[0]
                            and extent[1] >= self.qualified_extent
                            and extent[1] <= self.box_criteria_xy[1]
                        )
                        or (
                            extent[1] >= self.box_criteria_xy[0]
                            and extent[0] >= self.qualified_extent
                            and extent[0] <= self.box_criteria_xy[1]
                        )
                    )
                    and extent[0] * extent[1] * extent[2] <= self.box_criteria_xyz
                    and extent[2] >= self.qualified_extent
                    and center[2] >= self.qualified_center_z[0]
                    and center[2] <= self.qualified_center_z[1]
                ):
                    obj_type = "box"
                else:
                    obj_type = "unknown"
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
        self.all_detections = new_detections

        # # Publish visualization MarkerArray
        self.publish_markers()

        rospy.logdebug(f"[Before merge] Detect: {len(self.all_detections)} bboxes.")

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
        """Run ROS listener and publish detection results"""
        rospy.spin()


if __name__ == "__main__":
    processor = PointCloudClustering()
    processor.run()
