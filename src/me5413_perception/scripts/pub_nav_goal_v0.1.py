#!/usr/bin/env python
import tf
import json
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import MarkerArray, Marker
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from scipy.spatial.transform import Rotation as R

class FurthestBoxNavigator:
    def __init__(self):
        rospy.init_node("furthest_box_navigator", anonymous=True)

        self.overlap_threshold = rospy.get_param("~overlap_threshold", 0.6)
        self.bridge_length = rospy.get_param("~bridge_length", 4.0)
        self.box_size = rospy.get_param("~box_big_size", 0.8)
        self.yaw_bin_size = rospy.get_param("~yaw_bin_size", 30)
        self.box_half = self.box_size / 2.0

        self.tf_listener = tf.TransformListener()
        self.current_pose = np.eye(4)
        self.reach_goal = True
        self.last_goal_position = None
        self.last_bbox_msg = None
        self.finished_directions = set()

        self.fusion_info = {}

        self.bridge_ready = False
        self.bridge_position = None


        rospy.Subscriber("/move_base/status", Bool, self.status_callback)
        rospy.Subscriber("/gazebo/ground_truth/state", Odometry, self.odom_callback)
        rospy.Subscriber("/perception/marker/bbox_markers_fusion", MarkerArray, self.bbox_callback)
        rospy.Subscriber("/perception/fusion_box_labels", String, self.fusion_info_callback)
        rospy.Subscriber("/navigation/fallback_recover", Bool, self.recover_callback)

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

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        translation = np.array([position.x, position.y, position.z])
        rotation = R.from_quat([orientation.x, orientation.y, orientation.z, orientation.w]).as_matrix()

        self.current_pose = np.eye(4)
        self.current_pose[:3, :3] = rotation
        self.current_pose[:3, 3] = translation

    def bbox_callback(self, msg):
<<<<<<< HEAD
        self.last_bbox = MarkerArray()
        label_set = set()
        bridge_found = False
        
        for marker in msg.markers:
            marker_id = str(marker.id)
            if marker.ns != "box":
=======
        # self.last_bbox_msg = msg
        # print(self.last_bbox_msg)
        self.last_bbox = MarkerArray()
        label_set = set()
        bridge_found = False

        for marker in msg.markers:
            marker_id = str(marker.id)
            if marker.ns != "box":
                if marker.ns == "bridge":
                    bridge_found = True
                continue

            if marker_id in self.fusion_info:
                info = self.fusion_info[marker_id]
                if info.get("matched", True):
                    for h in info.get("history", []):
                        label_set.add(h["label"])
                else:
                    self.last_bbox.markers.append(marker)
            else:
                self.last_bbox.markers.append(marker)
            self.last_bbox_msg = self.last_bbox
        self.try_publish_bridgehead_if_ready(msg, label_set, bridge_found)

    def try_publish_bridgehead_if_ready(self, msg, label_set, bridge_found):
        if len(msg.markers) == 10 and len(label_set) >= 4 and bridge_found:
            rospy.loginfo("[Navigator] All boxes matched and 4 labels found, bridge exists. Publishing bridge goal")
            for marker in msg.markers:
>>>>>>> origin/main
                if marker.ns == "bridge":
                    bridge_found = True
                continue

            if marker_id in self.fusion_info:
                info = self.fusion_info[marker_id]
                if info.get("matched", True):
                    for h in info.get("history", []):
                        label_set.add(h["label"])
                else:
                    self.last_bbox.markers.append(marker)
            else:
                self.last_bbox.markers.append(marker)
        self.last_bbox_msg = self.last_bbox
        self.try_publish_bridgehead_if_ready(msg, label_set, bridge_found)
    
    def recover_callback(self, msg):
        if msg.data:
            rospy.loginfo("[Navigator] Recovering from fallback...")
            self.reach_goal = True  # 
            if self.last_bbox_msg:
<<<<<<< HEAD
                self.find_and_publish_new_goal(self.last_bbox_msg)


    def try_publish_bridgehead_if_ready(self, msg, label_set, bridge_found):
        box_count = 0
        bridge_count = 0
        all_boxes_in_front = True
        bridge_position = None

        for marker in msg.markers:
            if marker.ns == "box":
                box_count += 1
                if marker.pose.position.y > 9:
                    all_boxes_in_front = False
            elif marker.ns == "bridge":
                bridge_count += 1
                bridge_position = marker.pose.position

        if (box_count >= 10 and bridge_count == 1 and
            len(label_set) >= 4 and all_boxes_in_front and bridge_position and bridge_position.y > 9):
            
            rospy.loginfo("[Navigator] All boxes matched and 4 labels found, bridge exists.")
            self.bridge_ready = True
            self.bridge_position = [bridge_position.x, bridge_position.y + self.bridge_length/2, bridge_position.z]

    def status_callback(self, msg):
        self.reach_goal = msg.data
        if self.reach_goal:
            rospy.loginfo("Target reached, computing new target...")

            if self.bridge_ready:
                rospy.loginfo("[Navigator] Publishing bridge goal...")
                self.last_goal_position = self.bridge_position
                self.publish_goal(self.bridge_position)
                self.publish_arrow_marker(self.bridge_position)
                self.bridge_ready = False
            elif self.last_bbox_msg and self.last_bbox_msg.markers:
                self.find_and_publish_new_goal(self.last_bbox_msg)

            if self.last_bbox_msg and not self.last_bbox_msg.markers and not self.bridge_ready:
                rospy.logwarn("No unmatched boxes. Triggering fallback...")
                fallback_pub = rospy.Publisher("/navigation/fallback_trigger", Bool, queue_size=1, latch=True)
                rospy.sleep(0.5)
                fallback_pub.publish(Bool(data=True))

=======
                self.find_and_publish_new_goal()
>>>>>>> origin/main
        else:
            if self.last_goal_position is not None:
                rospy.loginfo("Continue current target...")
                self.publish_goal(self.last_goal_position)
                self.publish_arrow_marker(self.last_goal_position)
<<<<<<< HEAD
            


    def find_and_publish_new_goal(self, msg):
        max_distance = -1
        furthest_box_position = None

        for marker in msg.markers:
            if marker.ns != "box":
                continue
            if marker.pose.position.y <= 9:
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
>>>>>>> origin/main

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
        if self.last_bbox_msg:
            for marker in self.last_bbox_msg.markers:
                if marker.ns != "box":
                    continue

                marker_id = str(marker.id)
                matched = self.fusion_info.get(marker_id, {}).get("matched", True)
                if matched:
                    continue

                box_pos = np.array([
                    marker.pose.position.x,
                    marker.pose.position.y,
                    marker.pose.position.z
                ])
                box_vec = box_pos - self.current_pose[:3, 3]
                box_yaw = np.arctan2(box_vec[1], box_vec[0]) * 180.0 / np.pi
                box_yaw = (box_yaw + 360) % 360
                box_bin = int(box_yaw // self.yaw_bin_size) * self.yaw_bin_size

                if box_bin not in self.finished_directions:
                    unfinished_bins.add(box_bin)

        self.unfinished_pub.publish(json.dumps(sorted(list(unfinished_bins))))
<<<<<<< HEAD
        
=======

>>>>>>> origin/main
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
