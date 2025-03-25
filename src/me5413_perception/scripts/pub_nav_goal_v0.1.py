#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import MarkerArray, Marker
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from scipy.spatial.transform import Rotation as R
import json
import tf

class FurthestBoxNavigator:
    def __init__(self):
        rospy.init_node("furthest_box_navigator", anonymous=True)

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
        self.yolo_targets = []
        self.tracked_boxes = []
        self.unfinished_directions = []
        self.finished_directions = set()

        rospy.Subscriber("/gazebo/ground_truth/state", Odometry, self.odom_callback)
        rospy.Subscriber("/bbox_markers", MarkerArray, self.bbox_callback)
        rospy.Subscriber("/move_base/status", Bool, self.status_callback)
        rospy.Subscriber("/perception/yolo_targets_3d", String, self.yolo_callback)
        rospy.Subscriber("/one_rot/finish_status", Bool, self.rot_status_callback)
        rospy.Subscriber("/one_rot/end_status", Bool, self.rot_end_callback)

        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        self.marker_pub = rospy.Publisher("/nav_goal_marker", Marker, queue_size=1)
        self.unfinished_pub = rospy.Publisher("/one_rot/unfinished_boxes", String, queue_size=1)

        rospy.loginfo("FurthestBoxNavigator Initialization completed")
        rospy.spin()

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        translation = np.array([position.x, position.y, position.z])
        rotation = R.from_quat([orientation.x, orientation.y, orientation.z, orientation.w]).as_matrix()

        self.current_pose = np.eye(4)
        self.current_pose[:3, :3] = rotation
        self.current_pose[:3, 3] = translation

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
                center = [marker.pose.position.x, marker.pose.position.y, marker.pose.position.z]
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
                    count_in = sum(1 for p in points if self.point_in_box(np.array([p["x"], p["y"], p["z"]]), box_center))
                    ratio = count_in / float(len(points))
                    if ratio >= self.overlap_threshold:
                        box["labels"].append({"label": label, "conf": conf})
                        box["matched"] = True
        self.publish_active_boxes()
        self.try_publish_bridgehead_if_ready(msg)

    def try_publish_bridgehead_if_ready(self, msg):
        msg_ids = {marker.id for marker in msg.markers if marker.ns == "box"}
        tracked_ids = {box["id"] for box in self.tracked_boxes}
        all_matched = all(box["matched"] for box in self.tracked_boxes)

        unmatched_labels = set()
        tracked_centers = [np.array(box["center"]) for box in self.tracked_boxes]
        for target in self.yolo_targets:
            label = target["label"]
            points = target["points"]
            if all(not self.point_in_box(np.array([p["x"], p["y"], p["z"]]), center) for center in tracked_centers for p in points):
                unmatched_labels.add(label)

        if msg_ids == tracked_ids and all_matched and not unmatched_labels:
            rospy.loginfo("[Navigator] All matched. Publishing bridge goal")
            label_dict = {}
            for box in self.tracked_boxes:
                for l in box["labels"]:
                    label = l["label"]
                    conf = l["conf"]
                    label_dict.setdefault(label, []).append(conf)
            if not label_dict:
                return
            best_label = max(label_dict.items(), key=lambda x: sum(x[1])/len(x[1]))[0]
            for box in self.tracked_boxes:
                box["label"] = best_label
            for marker in msg.markers:
                if marker.ns == "bridge":
                    pos = marker.pose.position
                    self.publish_goal([pos.x, pos.y + self.box_half, pos.z])
                    self.publish_arrow_marker([pos.x, pos.y, pos.z])
                    break

    def publish_active_boxes(self):
        marker_array = MarkerArray()
        for box in self.tracked_boxes:
            if not box["matched"]:
                marker_array.markers.append(box["marker"])
        self.last_bbox_msg = marker_array
     
    def end_status_callback(self, msg):
        if msg.data and self.last_bbox_msg:
            self.find_and_publish_new_goal(self.last_bbox_msg)

    def status_callback(self, msg):
        self.reach_goal = msg.data
        if self.reach_goal:
            rospy.loginfo("Target reached, computing new target...")
            if self.last_bbox_msg:
                self.find_and_publish_new_goal(self.last_bbox_msg)
        else:
            if self.last_goal_position is not None:
                rospy.loginfo("Continue current target...")
                self.publish_goal(self.last_goal_position)
                self.publish_arrow_marker(self.last_goal_position)

    def find_and_publish_new_goal(self, msg):
        max_distance = -1
        furthest_box_position = None
        for marker in msg.markers:
            if marker.ns != "box" or marker.pose.position.y <= 9:
                continue
            box_pos = np.array([marker.pose.position.x, marker.pose.position.y, marker.pose.position.z])
            robot_pos = self.current_pose[:3, 3]
            distance = np.linalg.norm(box_pos - robot_pos)
            if distance > max_distance:
                max_distance = distance
                furthest_box_position = box_pos
        if furthest_box_position is not None:
            self.last_goal_position = furthest_box_position
            self.publish_goal(furthest_box_position)
            self.publish_arrow_marker(furthest_box_position)
    
    def rot_end_callback(self, msg):
        if msg.data:  
            rospy.loginfo("[Rotation] One rotation finished. Computing new goal...")
            if self.last_bbox_msg:
                self.find_and_publish_new_goal(self.last_bbox_msg)


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

    def publish_goal(self, position):
        goal_msg = PoseStamped()
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.header.frame_id = "map"
        goal_msg.pose.position.x = position[0]
        goal_msg.pose.position.y = position[1]
        goal_msg.pose.position.z = position[2]
        goal_msg.pose.orientation.w = 1.0
        self.goal_pub.publish(goal_msg)
        rospy.loginfo(f"Target point: x={position[0]:.2f}, y={position[1]:.2f}, z={position[2]:.2f}")

    def publish_arrow_marker(self, position):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "goal_arrow"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        start_point = self.current_pose[:3, 3]
        marker.points = [Point(x=start_point[0], y=start_point[1], z=start_point[2]),
                         Point(x=position[0], y=position[1], z=position[2])]
        marker.scale.x = 0.1
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.lifetime = rospy.Duration(1.0)
        self.marker_pub.publish(marker)

if __name__ == "__main__":
    try:
        FurthestBoxNavigator()
    except rospy.ROSInterruptException:
        pass
