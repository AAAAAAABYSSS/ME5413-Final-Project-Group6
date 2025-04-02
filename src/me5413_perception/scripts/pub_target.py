#!/usr/bin/env python
import rospy
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import open3d as o3d
import std_msgs.msg
import os
import json
import tf

from sklearn.linear_model import RANSACRegressor
from scipy.spatial.transform import Rotation as R
from visualization_msgs.msg import Marker, MarkerArray


class ClusterProcessor:
    def __init__(self):
        rospy.init_node("pub_target", anonymous=True)

        # Initialize storage variables
        self.current_pose = None
        self.all_bbox = []
        self.next_unique_id = 0
        self.latest_bbox_markers = None

        self.merge_volume_threshold = rospy.get_param("~merge_volume_threshold", 0.05)
        self.max_iterations = rospy.get_param("~max_iterations", 5)
        self.center_dist_threshold = rospy.get_param("~center_dist_threshold", 0.5)
        self.max_extent = rospy.get_param("~max_extent", 0.81)
        self.qualified_extent = rospy.get_param("~qualified_extent", 0.65)
        self.delete_radius = rospy.get_param("~delete_radius", 0.56569)

        # Subscribe to topics
        rospy.Subscriber(
            "/perception/marker/bbox_markers_pending", MarkerArray, self.bbox_callback
        )

        # Publish processed detection results
        self.marker_pub = rospy.Publisher(
            "/perception/marker/bbox_markers", MarkerArray, queue_size=1
        )

        rospy.loginfo("Subscribed to /perception/marker/bbox_markers_pending topic.")

    def bbox_callback(self, msg):
        """Get bbox info and process it"""
        bboxes = []
        for marker in msg.markers:
            # Load marker msg with default quat = [0, 0, 0, 1]
            id = marker.id
            type = marker.ns
            center = np.array(
                [
                    marker.pose.position.x,
                    marker.pose.position.y,
                    marker.pose.position.z,
                ]
            )
            extent = np.array(
                [
                    marker.scale.x,
                    marker.scale.y,
                    marker.scale.z,
                ]
            )
            max_bound, min_bound = self.cal_max_min_bound(center, extent)

            bboxes.append(
                {
                    "id": id,
                    "type": type,
                    "center": center.tolist(),
                    "min_bound": min_bound.tolist(),
                    "max_bound": max_bound.tolist(),
                    "extent": extent.tolist(),
                }
            )

        # Merge bounding boxes iteratively
        self.all_bbox = self.merge_bboxes(self.all_bbox + bboxes)

        # Delete error boxes
        rospy.logdebug(f"before: {len(self.all_bbox)}")
        self.all_bbox = self.delete_bboxes(self.all_bbox)
        rospy.logdebug(f"after: {len(self.all_bbox)}")

        # Publish visualization MarkerArray
        self.publish_markers()

        rospy.loginfo(f"Detect {len(self.all_bbox)} boxes.")

    def cal_max_min_bound(self, center, extent):
        half_extent = extent / 2.0
        max_bound = center + half_extent
        min_bound = center - half_extent
        return max_bound, min_bound

    def merge_bboxes(self, bboxes):
        """Iteratively merge bounding boxes with high IoU"""
        threshold = self.merge_volume_threshold
        max_iterations = self.max_iterations

        # Ensure the structure of bboxes is correct
        bboxes = [
            b
            for b in bboxes
            if isinstance(b, dict) and "type" in b and b["type"] in ["box", "bridge"]
        ]

        for iteration in range(max_iterations):
            rospy.logdebug(f"Iteration {iteration + 1} for merging...")

            merged_bboxes = []
            merge_occurred = False
            # Copy to avoid modifying the original list
            remaining_bboxes = list(bboxes)

            while remaining_bboxes:
                base_bbox = remaining_bboxes.pop(0)
                to_merge = [base_bbox]
                new_remaining = []

                for bbox in remaining_bboxes:
                    if base_bbox["type"] == bbox["type"]:
                        if bbox["type"] == "box":
                            center_dist = np.linalg.norm(
                                np.array(base_bbox["center"]) - np.array(bbox["center"])
                            )
                            if (
                                self.calculate_iou(base_bbox, bbox) > threshold
                                and center_dist <= self.center_dist_threshold
                            ):
                                to_merge.append(bbox)
                                merge_occurred = True
                            else:
                                new_remaining.append(bbox)
                        elif bbox["type"] == "bridge":
                            # Eliminate wall interference and directly merge other bboxes
                            center_dist = np.linalg.norm(
                                np.array(base_bbox["center"]) - np.array(bbox["center"])
                            )
                            if center_dist <= (
                                self.center_dist_threshold + 0.5
                            ):  # give more offset
                                to_merge.append(bbox)
                                merge_occurred = True
                    else:
                        new_remaining.append(bbox)

                # Merge all bounding boxes with IoU above the threshold
                merged_bbox = to_merge[0]
                for bbox in to_merge[1:]:
                    merged_bbox = self.merge_two_bboxes(merged_bbox, bbox)

                merged_bboxes.append(merged_bbox)
                # Update unmerged bboxes
                remaining_bboxes = new_remaining

            if not merge_occurred:
                rospy.logdebug("No new merges occurred, exiting early.")
                break

            # Update bboxes for the next iteration
            bboxes = merged_bboxes

        return bboxes

    def calculate_iou(self, box1, box2):
        """IoU calculation function"""
        # Compute intersection
        inter_min = np.maximum(box1["min_bound"], box2["min_bound"])
        inter_max = np.minimum(box1["max_bound"], box2["max_bound"])
        inter_dim = np.maximum(0, inter_max - inter_min)
        inter_volume = np.prod(inter_dim)

        # If it is a fully inclusive relationship, it will be returned directly without judging IOU
        if inter_volume == np.prod(inter_min):
            return 1 + self.merge_volume_threshold

        # Compute union
        vol1 = np.prod(np.array(box1["max_bound"]) - np.array(box1["min_bound"]))
        vol2 = np.prod(np.array(box2["max_bound"]) - np.array(box2["min_bound"]))
        union_volume = vol1 + vol2 - inter_volume

        # Compute IoU
        iou = inter_volume / union_volume if union_volume > 0 else 0
        return iou

    def merge_two_bboxes(self, box1, box2):
        """Merge two bounding boxes"""
        # Avoid merging errors, return original bbox
        if not isinstance(box1, dict) or not isinstance(box2, dict):
            rospy.logwarn(
                "The bounding boxes to be merged are not valid dictionaries, skipping."
            )
            return box1

        min_bound = np.minimum(box1["min_bound"], box2["min_bound"]).tolist()
        max_bound = np.maximum(box1["max_bound"], box2["max_bound"]).tolist()
        center = ((np.array(min_bound) + np.array(max_bound)) / 2).tolist()
        extent = (np.array(max_bound) - np.array(min_bound)).tolist()

        # Avoid merging error caused by distortion
        if any(extent) > self.max_extent and box1["type"] == "box":
            return {
                "id": box1["id"],
                "type": box1["type"],
                "center": box1["center"],
                "min_bound": box1["min_bound"],
                "max_bound": box1["max_bound"],
                "extent": box1["extent"],
            }

        return {
            "id": box1["id"],
            "type": box1["type"],
            "center": center,
            "min_bound": min_bound,
            "max_bound": max_bound,
            "extent": extent,
        }

    def delete_bboxes(self, bboxes):
        # Ensure the structure of bboxes is correct (only for box type)
        box_bboxes = [
            b
            for b in bboxes
            if isinstance(b, dict) and "type" in b and b["type"] in ["box"]
        ]
        bridge_bboxes = [
            b
            for b in bboxes
            if isinstance(b, dict) and "type" in b and b["type"] in ["bridge"]
        ]
        qualified_bboxes = [
            bbox
            for bbox in box_bboxes
            if all(e >= self.qualified_extent for e in bbox["extent"])
        ]
        unqualified_bboxes = [
            bbox
            for bbox in box_bboxes
            if not all(e >= self.qualified_extent for e in bbox["extent"])
        ]

        # No quilified bbox, do not delete anything
        if not qualified_bboxes:
            return bboxes

        while unqualified_bboxes:
            bbox = unqualified_bboxes.pop(0)
            for base_bbox in qualified_bboxes:
                center_dist = np.linalg.norm(
                    np.array(base_bbox["center"]) - np.array(bbox["center"])
                )
                # If there is a qualified box very close, ignore this small bbox
                if center_dist <= self.delete_radius:
                    break
            else:
                # No qualified box is close enough → keep this bbox (regarded as the new target)
                qualified_bboxes.append(bbox)

        return bridge_bboxes + qualified_bboxes

    def publish_markers(self):
        marker_array = MarkerArray()
        for detection in self.all_bbox:
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
        """Run ROS listener"""
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.publish_markers()
            rate.sleep()


if __name__ == "__main__":
    processor = ClusterProcessor()
    processor.run()
