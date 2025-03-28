#!/usr/bin/env python
import tf
import json
import rospy
import numpy as np
<<<<<<< Updated upstream
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import MarkerArray, Marker
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from scipy.spatial.transform import Rotation as R
=======
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray, Marker
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import String
import json
import os
import tf
>>>>>>> Stashed changes

class FurthestBoxNavigator:
    def __init__(self):
        rospy.init_node("furthest_box_navigator", anonymous=True)
<<<<<<< Updated upstream

        self.overlap_threshold = rospy.get_param("~overlap_threshold", 0.6)
        self.bridge_width = rospy.get_param("~bridge_width", 4.0)
        self.box_size = rospy.get_param("~box_big_size", 0.8)
        self.yaw_bin_size = rospy.get_param("~yaw_bin_size", 30)
        self.box_half = self.box_size / 2.0

        self.tf_listener = tf.TransformListener()
        self.current_pose = np.eye(4)
        self.reach_goal = True
        self.last_goal_position = None
        self.last_bbox_msg = None
        self.tracked_boxes = []
        self.finished_directions = set()

        self.fusion_info = {}

        rospy.Subscriber("/move_base/status", Bool, self.status_callback)
        rospy.Subscriber("/gazebo/ground_truth/state", Odometry, self.odom_callback)
        rospy.Subscriber("/perception/marker/bbox_markers_visualized", MarkerArray, self.bbox_callback)
        rospy.Subscriber("/perception/fusion_box_labels", String, self.fusion_info_callback)

        rospy.Subscriber("/one_rot/finish_status", Bool, self.rot_status_callback)
        rospy.Subscriber("/one_rot/end_status", Bool, self.rot_end_callback)

        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        self.marker_pub = rospy.Publisher("/perception/marker/nav_goal_marker", Marker, queue_size=1)
        self.unfinished_pub = rospy.Publisher("/one_rot/unfinished_boxes", String, queue_size=1)

        rospy.loginfo("FurthestBoxNavigator Initialization completed")
        rospy.spin()

    def fusion_info_callback(self, msg):
        try:
            self.fusion_info = json.loads(msg.data)
        except Exception as e:
            rospy.logwarn(f"[Fusion Info] Failed to parse: {e}")
=======

        self.overlap_threshold = rospy.get_param("~overlap_threshold", 0.6)
        self.bridge_width = rospy.get_param("~bridge_width", 4.0)
        self.box_size = rospy.get_param("~box_big_size", 0.8)
        self.box_half = self.box_size / 2.0

        self.tf_listener = tf.TransformListener()
        self.current_pose = np.eye(4)  # 4x4 transformation matrix from map to base_link
        self.reach_goal = True  # Assume robot starts at the target
        self.last_goal_position = None
        self.last_bbox_msg = None
        self.yolo_targets = []
        self.tracked_boxes = []

        rospy.Subscriber("/gazebo/ground_truth/state", Odometry, self.odom_callback)
        rospy.Subscriber("/bbox_markers", MarkerArray, self.bbox_callback)
        rospy.Subscriber("/move_base/status", Bool, self.status_callback)
        rospy.Subscriber("/perception/yolo_targets_3d", String, self.yolo_callback)


        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        self.marker_pub = rospy.Publisher("/nav_goal_marker", Marker, queue_size=1)
        # self.bbox_marker_pub = rospy.Publisher("/bbox_markers_tracked", MarkerArray, queue_size=1)


        rospy.loginfo("FurthestBoxNavigator Initialization completed")
        rospy.spin()
>>>>>>> Stashed changes

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        translation = np.array([position.x, position.y, position.z])
<<<<<<< Updated upstream
        rotation = R.from_quat([orientation.x, orientation.y, orientation.z, orientation.w]).as_matrix()
=======
        rotation = R.from_quat([
            orientation.x, orientation.y, orientation.z, orientation.w
        ]).as_matrix()
>>>>>>> Stashed changes

        self.current_pose = np.eye(4)
        self.current_pose[:3, :3] = rotation
        self.current_pose[:3, 3] = translation

<<<<<<< Updated upstream
    def bbox_callback(self, msg):
        self.last_bbox_msg = msg
        print(self.last_bbox_msg)
        # self.last_bbox = MarkerArray()
        # label_set = set()
        # bridge_found = False

        # for marker in msg.markers:
        #     marker_id = str(marker.id)
        #     if marker.ns != "box":
        #         if marker.ns == "bridge":
        #             bridge_found = True
        #         continue

        #     if marker_id in self.fusion_info:
        #         info = self.fusion_info[marker_id]
        #         if info.get("matched", True):
        #             for h in info.get("history", []):
        #                 label_set.add(h["label"])
        #         else:
        #             self.last_bbox.markers.append(marker)
        #     else:
        #         self.last_bbox.markers.append(marker)
        #     self.last_bbox_msg = self.last_bbox
        #     print('ababaabababbabab阿爸爸不', self.last_bbox_msg)
        # self.try_publish_bridgehead_if_ready(msg, label_set, bridge_found)

    def try_publish_bridgehead_if_ready(self, msg, label_set, bridge_found):
        if len(msg.markers) == 10 and len(label_set) >= 4 and bridge_found:
            rospy.loginfo("[Navigator] All boxes matched and 4 labels found, bridge exists. Publishing bridge goal")
            for marker in msg.markers:
                if marker.ns == "bridge":
                    pos = marker.pose.position
                    self.publish_goal([pos.x, pos.y + self.box_half, pos.z])
                    self.publish_arrow_marker([pos.x, pos.y, pos.z])
                    break

    def status_callback(self, msg):
        self.reach_goal = msg.data
        if self.reach_goal:
            rospy.loginfo("Target reached, computing new target...")
            if self.last_bbox_msg:
                self.find_and_publish_new_goal()
        else:
            if self.last_goal_position is not None:
                rospy.loginfo("Continue current target...")
                self.publish_goal(self.last_goal_position)
                self.publish_arrow_marker(self.last_goal_position)

    def find_and_publish_new_goal(self):
        max_distance = -1
        furthest_box_position = None

        for box in self.tracked_boxes:
            if not box["matched"]:
                marker = box["marker"]
                if marker.ns != "box" or marker.pose.position.y <= 9:
                    continue
                box_pos = np.array([
                    marker.pose.position.x,
                    marker.pose.position.y,
                    marker.pose.position.z
                ])
                robot_pos = self.current_pose[:3, 3]
                distance = np.linalg.norm(box_pos - robot_pos)

                if distance > max_distance:
                    max_distance = distance
                    furthest_box_position = box_pos
=======
    def yolo_callback(self, msg):
        try:
            self.yolo_targets = json.loads(msg.data)
            rospy.loginfo(f"[YoloCallback] get {len(self.yolo_targets)} target")
        except Exception as e:
            rospy.logwarn(f"[YoloCallback] Failed: {e}")
            self.yolo_targets = []

    def point_in_box(self, point, box_center):
        return all(abs(point[i] - box_center[i]) <= self.box_half for i in range(3))

    def bbox_callback(self, msg):
        if not self.tracked_boxes:
            for marker in msg.markers:
                if marker.ns != "box":
                    continue

                if any(box["id"] == marker.id for box in self.tracked_boxes):
                    continue

                center = [
                    marker.pose.position.x,
                    marker.pose.position.y,
                    marker.pose.position.z
                ]
                self.tracked_boxes.append({
                    "id": marker.id,
                    "center": center,
                    "matched": False,
                    "labels": [],
                    "label": [],
                    "marker": marker
                })
        else:
            for box in self.tracked_boxes:

                box_center = np.array(box["center"])
                for target in self.yolo_targets:
                    label = target["label"]
                    conf = target["conf"]
                    points = target["points"]

                    count_in = sum(
                        1 for p in points
                        if self.point_in_box(np.array([p["x"], p["y"], p["z"]]), box_center)
                    )
                    ratio = count_in / float(len(points))
                    if ratio >= self.overlap_threshold:
                        box["labels"].append({"label": label, "conf": conf})
                        box["matched"] = True

                marker = box["marker"]

        self.publish_active_boxes()
        self.try_publish_bridgehead_if_ready(msg)

    def try_publish_bridgehead_if_ready(self, msg):
        # Step 1: Check if all tracked boxes are matched and all box IDs match the current MarkerArray
        msg_ids = {marker.id for marker in msg.markers if marker.ns == "box"}
        tracked_ids = {box["id"] for box in self.tracked_boxes}
        all_matched = all(box["matched"] for box in self.tracked_boxes)

        # Step 2: Check if there are any YOLO targets not matched to any existing box
        unmatched_labels = set()
        tracked_centers = [np.array(box["center"]) for box in self.tracked_boxes]

        for target in self.yolo_targets:
            label = target["label"]
            points = target["points"]

            all_outside = True
            for p in points:
                pt = np.array([p["x"], p["y"], p["z"]])
                if any(self.point_in_box(pt, center) for center in tracked_centers):
                    all_outside = False
                    break

            if all_outside:
                unmatched_labels.add(label)

        # Step 3: Proceed only if all boxes are matched and no unmatched targets remain
        if msg_ids == tracked_ids and all_matched and not unmatched_labels:
            rospy.loginfo("[Navigator] All boxes matched and no unmatched YOLO targets remain. Proceeding to compute the most probable label and publish the bridge goal.")

            # Step 4: Compute average confidence for each label across all matched boxes
            label_dict = {}
            for box in self.tracked_boxes:
                for l in box["labels"]:
                    label = l["label"]
                    conf = l["conf"]
                    if label not in label_dict:
                        label_dict[label] = []
                    label_dict[label].append(conf)

            if not label_dict:
                rospy.logwarn("[Navigator] No label data available for confidence computation.")
                return

            label_avg_conf = {k: sum(v) / len(v) for k, v in label_dict.items()}
            best_label = max(label_avg_conf.items(), key=lambda x: x[1])[0]
            rospy.loginfo(f"[Navigator] Final selected label: {best_label}")

            # Step 5: Assign the selected label to all tracked boxes
            for box in self.tracked_boxes:
                box["label"] = best_label

            # Step 6: Locate the bridge marker and publish its position as the next goal
            for marker in msg.markers:
                if marker.ns == "bridge":
                    position = marker.pose.position
                    rospy.loginfo(f"[Navigator] Publishing bridge goal at: ({position.x:.2f}, {position.y:.2f}, {position.z:.2f})")
                    self.publish_goal([position.x, position.y + self.box_half, position.z])
                    self.publish_arrow_marker([position.x, position.y, position.z])
                    break
        else:
            if unmatched_labels:
                rospy.logwarn(f"[Navigator] Unmatched YOLO labels found: {list(unmatched_labels)}")
            if not all_matched:
                rospy.loginfo("[Navigator] Some boxes are still unmatched. Waiting for further recognition...")

    def publish_active_boxes(self):
        marker_array = MarkerArray()
        for box in self.tracked_boxes:
            if not box["matched"]:
                marker_array.markers.append(box["marker"])
        self.last_bbox_msg = marker_array  
        print('self.last_bbox_msg',self.last_bbox_msg, 'self.tracked_boxes',self.tracked_boxes)
        # self.bbox_marker_pub.publish(marker_array)

    def status_callback(self, msg):
        """Publish new target based on status"""
        self.reach_goal = msg.data

        if self.reach_goal:
            rospy.loginfo("Target reached, computing new target...")
            if self.last_bbox_msg:
                self.find_and_publish_new_goal(self.last_bbox_msg)
        else:
            if self.last_goal_position is not None:
                rospy.loginfo("Target not reached, continue publishing current target...")
                self.publish_goal(self.last_goal_position)
                self.publish_arrow_marker(self.last_goal_position)

    def find_and_publish_new_goal(self, msg):
        """Compute the furthest box and publish the target"""
        max_distance = -1
        furthest_box_position = None

        for marker in msg.markers:
            if marker.ns != "box":
                continue
            if marker.pose.position.y <= 9:
                continue  #  Ignore the opposite side of the bridge

            box_pos = np.array([
                marker.pose.position.x,
                marker.pose.position.y,
                marker.pose.position.z
            ])

            robot_pos = self.current_pose[:3, 3]
            distance = np.linalg.norm(box_pos - robot_pos)

            if distance > max_distance:
                max_distance = distance
                furthest_box_position = box_pos
>>>>>>> Stashed changes

        if furthest_box_position is not None:
            self.last_goal_position = furthest_box_position
            self.publish_goal(furthest_box_position)
            self.publish_arrow_marker(furthest_box_position)
<<<<<<< Updated upstream

    def rot_end_callback(self, msg):
        if msg.data:  
            rospy.loginfo("[Rotation] One rotation finished. Computing new goal...")
            if self.last_bbox_msg:
                self.find_and_publish_new_goal()

    def rot_status_callback(self, msg):
        if not msg.data:
            return  

        yaw = np.arctan2(self.current_pose[1, 0], self.current_pose[0, 0]) * 180.0 / np.pi
        yaw = (yaw + 360) % 360  
        yaw_bin = int(yaw // self.yaw_bin_size) * self.yaw_bin_size

        self.finished_directions.add(yaw_bin)
        rospy.loginfo(f"[Rotation] Finished direction: {yaw_bin}°")

        unfinished_bins = set()
        for box in self.tracked_boxes:
            if not box["matched"]:
                box_vec = np.array(box["center"]) - self.current_pose[:3, 3]
                box_yaw = np.arctan2(box_vec[1], box_vec[0]) * 180.0 / np.pi
                box_yaw = (box_yaw + 360) % 360
                box_bin = int(box_yaw // self.yaw_bin_size) * self.yaw_bin_size
                if box_bin not in self.finished_directions:
                    unfinished_bins.add(box_bin)

        self.unfinished_pub.publish(json.dumps(sorted(list(unfinished_bins))))
=======
            
>>>>>>> Stashed changes

    def publish_goal(self, position):
        goal_msg = PoseStamped()
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.header.frame_id = "map"
        goal_msg.pose.position.x = position[0]
        goal_msg.pose.position.y = position[1]
        goal_msg.pose.position.z = position[2]
<<<<<<< Updated upstream
        goal_msg.pose.orientation.w = 1.0
        self.goal_pub.publish(goal_msg)
        rospy.loginfo(f"Target point: x={position[0]:.2f}, y={position[1]:.2f}, z={position[2]:.2f}")
=======

        goal_msg.pose.orientation.x = 0.0
        goal_msg.pose.orientation.y = 0.0
        goal_msg.pose.orientation.z = 0.0
        goal_msg.pose.orientation.w = 1.0

        self.goal_pub.publish(goal_msg)
        rospy.loginfo(f"Target point: ={position[0]:.2f}, y={position[1]:.2f}, z={position[2]:.2f}")
>>>>>>> Stashed changes

    def publish_arrow_marker(self, position):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "goal_arrow"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
<<<<<<< Updated upstream
        start_point = self.current_pose[:3, 3]
        marker.points = [Point(x=start_point[0], y=start_point[1], z=start_point[2]),
                         Point(x=position[0], y=position[1], z=position[2])]
        marker.scale.x = 0.1
        marker.scale.y = 0.2
        marker.scale.z = 0.2
=======

        from geometry_msgs.msg import Point
        start_point = self.current_pose[:3, 3]
        end_point = position
        marker.points = [Point(x=start_point[0], y=start_point[1], z=start_point[2]),
                         Point(x=end_point[0], y=end_point[1], z=end_point[2])]

        marker.scale.x = 0.1
        marker.scale.y = 0.2
        marker.scale.z = 0.2

>>>>>>> Stashed changes
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
<<<<<<< Updated upstream
=======

>>>>>>> Stashed changes
        marker.lifetime = rospy.Duration(1.0)
        self.marker_pub.publish(marker)

if __name__ == "__main__":
    try:
        FurthestBoxNavigator()
    except rospy.ROSInterruptException:
        pass
