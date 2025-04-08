#!/usr/bin/env python3
import tf
import json
import rospy
import numpy as np
from scipy.ndimage import label
from std_msgs.msg import Bool, String
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import MarkerArray, Marker

class BridgeCrossingManager:
    def __init__(self, navigator):
        self.navigator = navigator
        self.crossed = False

    def execute_crossing(self):
        rospy.loginfo("[BridgeCrossing] Preparing to cross bridge...")
        rospy.sleep(0.5)
        self.navigator.open_bridge_pub.publish(Bool(data=True))
        rospy.loginfo("[BridgeCrossing] Bridge opening command sent.")
        rospy.sleep(0.5)
        self.navigator.publish_goal(self.navigator.post_bridge_position, yaw_deg=180.0)
        self.navigator.publish_arrow_marker(self.navigator.post_bridge_position)
        self.crossed = True

    def after_crossing_logic(self):
        rospy.loginfo("[BridgeCrossing] Executing post-bridge exploration...")
        self.navigator.update_pose_from_tf()

        if self.navigator.target_box_found(post_bridge=True):
            rospy.loginfo("[BridgeCrossing] Target box detected post-bridge, navigating to it.")
            self.navigator.publish_goal(self.navigator.target_box_position)
            return

        self.navigator.perform_rotation_scan()

        if self.navigator.target_box_found(post_bridge=True):
            rospy.loginfo("[BridgeCrossing] Target box found after rotation scan.")
            self.navigator.publish_goal(self.navigator.target_box_position)
            return

        rospy.logwarn("[BridgeCrossing] No target box found after full scan.")
        rarest_box = self.navigator.find_rarest_postbridge_box_by_prebridge_label_stats()
        if rarest_box:
            rospy.loginfo(f"[BridgeCrossing] Navigating to rarest post-bridge box: {rarest_box['id']} ({rarest_box['best_label']})")
            self.navigator.publish_goal(rarest_box["position"])
            self.navigator.publish_arrow_marker(rarest_box["position"])
            self.navigator.last_goal_position = rarest_box["position"]
        else:
            rospy.logwarn("[BridgeCrossing] No rarest box available to navigate to.")


class FurthestBoxNavigator:
    def __init__(self):
        rospy.init_node("furthest_box_navigator", anonymous=True)

        # Params
        self.overlap_threshold = rospy.get_param("~overlap_threshold", 0.6)
        self.bridge_length = rospy.get_param("~bridge_length", 4.0)
        self.box_size = rospy.get_param("~box_big_size", 0.8)
        self.yaw_bin_size = rospy.get_param("~yaw_bin_size", 30)
        self.fallback_grid_resolution = rospy.get_param("~fallback_grid_resolution", 0.2)
        self.x_range = rospy.get_param("~x_range", [11.0, 19.0])
        self.y_range = rospy.get_param("~y_range", [-22.0, -2.0])
        self.bridge_pos = rospy.get_param("~bridge_pos", 9.0)

        # State
        self.tf_listener = tf.TransformListener()
        self.current_pose = np.eye(4)

        self.begin_nav= False

        self.bridge_ready = False
        self.bridge_goal_sent = False
        self.bridge_found = False
        self.bridge_position = None
        self.post_bridge_position = None

        self.target_box_position = None
        self.last_goal_position = None
        self.last_bbox_msg = None
        self.current_box = MarkerArray()

        self.fusion_info = {}
        self.has_received_bbox = False

        self.in_fallback_mode = False
        self.reach_goal = True
        self.crossing_manager = BridgeCrossingManager(self)
        self.has_crossed_bridge = False
        self.label_set = set()

        # ROS Comm
        rospy.Subscriber("/move_status", Bool, self.status_callback)
        # rospy.Subscriber("/move_status", Bool, self.status_callback)
        rospy.Subscriber("/perception/marker/bbox_markers_fusion", MarkerArray, self.bbox_callback)
        rospy.Subscriber("/perception/fusion_box_labels", String, self.fusion_info_callback)
        rospy.Subscriber("/one_rot/end_status", Bool, self.rot_end_callback)


        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        self.marker_pub = rospy.Publisher("/perception/marker/nav_goal_marker", Marker, queue_size=1)
        self.open_bridge_pub = rospy.Publisher("/cmd_open_bridge", Bool, queue_size=1)

        rospy.Timer(rospy.Duration(1.0), self.update_pose_from_tf)
        rospy.Timer(rospy.Duration(1.0), self.periodic_bridge_check)
        self.begin_nav_publish_goal()
        rospy.loginfo("FurthestBoxNavigator Initialization completed")
        rospy.spin()

    def periodic_bridge_check(self, event):
        if self.try_publish_bridge_goal():
            
            rospy.loginfo("[BridgeCheck] Periodically published bridge goal.")

    def find_rarest_postbridge_box_by_prebridge_label_stats(self):
        if not self.last_bbox_msg or not self.fusion_info:
            return None
        
        bridge_front_label_count = {}
        postbridge_boxes = []

        for marker in self.last_bbox_msg.markers:
            if marker.ns != "box":
                continue

            marker_id = str(marker.id)
            info = self.fusion_info.get(marker_id, {})
            if not info.get("matched", False):
                continue

            labels = {}
            for h in info.get("history", []):
                label = h.get("label")
                conf = h.get("conf")
                if label:
                    if label not in labels:
                        labels[label] = []
                    labels[label].append(conf)

            if not labels:
                continue

            best_label, best_conf = max(
                ((l, sum(c) / len(c)) for l, c in labels.items()), key=lambda x: x[1]
            )

            box_info = {
                "id": marker_id,
                "position": [marker.pose.position.x, marker.pose.position.y, marker.pose.position.z],
                "best_label": best_label,
                "avg_conf": best_conf
            }

            if marker.pose.position.x > 5.0:
                bridge_front_label_count[best_label] = bridge_front_label_count.get(best_label, 0) + 1
            else:
                postbridge_boxes.append(box_info)

        if len(postbridge_boxes) < 4:
            rospy.logwarn("[RareLabel] Less than 4 post-bridge boxes available.")
            return None

        def get_label_count(label):
            return bridge_front_label_count.get(label, 0)

        rarest_box = min(postbridge_boxes, key=lambda b: get_label_count(b["best_label"]))
        rospy.loginfo(f"[RareLabel] Rarest post-bridge box is {rarest_box['id']} with label '{rarest_box['best_label']}' (avg conf: {rarest_box['avg_conf']:.2f}), seen {get_label_count(rarest_box['best_label'])} times before bridge.")
        return rarest_box

    def wait_for_goal_or_detection(self):
        rospy.sleep(3.0)

    def perform_rotation_scan(self):
        rospy.loginfo("[Scan] Moving to predefined scan positions for rotation scan...")

        scan_positions = [[4, -8, 0], [4, -16, 0]]
        for idx, position in enumerate(scan_positions):
            rospy.loginfo(f"[Scan] Navigating to scan position {idx+1}: {position}")
            self.publish_goal(position, yaw_deg=180.0)
            self.wait_for_goal_or_detection()

            if self.target_box_found(post_bridge=True):
                rospy.loginfo("[Scan] Target found before starting rotation at this point.")
                return

        rospy.logwarn("[Scan] No target found after full rotation scan at both positions.")

    def status_callback(self, msg):
        self.publish_next_goal = msg.data
        
        if self.try_publish_bridge_goal():
            return

        if self.publish_next_goal and self.begin_nav:
            rospy.loginfo("Target reached, computing new target...")
            if self.last_bbox_msg and any(m.ns == "box" for m in self.last_bbox_msg.markers):
                self.exit_fallback_mode()
                self.last_goal_position = self.find_and_publish_new_goal(self.last_bbox_msg)
            else:
                fallback_goal = self.find_fallback_goal()
                if fallback_goal and self.begin_nav:
                    self.enter_fallback_mode()
                    self.last_goal_position = fallback_goal
                    rospy.loginfo(f"[Navigator-status] No target found, using fallback goal.{self.begin_nav}")
                    self.publish_goal(fallback_goal)
                    self.publish_arrow_marker(fallback_goal)
        else:
            if self.last_goal_position is not None and self.begin_nav:
                rospy.loginfo("[Navigator-status] No target found, using last goal position.")
                # self.publish_goal(self.last_goal_position)
                self.publish_arrow_marker(self.last_goal_position)

        if self.reach_goal and self.bridge_goal_sent and not self.has_crossed_bridge:
            rospy.loginfo("[Navigator] Reached bridge goal. Starting crossing...")
            self.crossing_manager.execute_crossing()
            self.has_crossed_bridge = True
            self.crossing_manager.after_crossing_logic()

    def begin_nav_publish_goal(self):
        if self.begin_nav:
            rospy.loginfo("[Navigator] Navigation already started.")
            return 
        if not self.begin_nav:
            fallback_goal = self.find_fallback_goal()
            if fallback_goal:
                rospy.loginfo("[Navigator-begin] Starting navigation in fallback mode.")
                self.enter_fallback_mode()
                self.last_goal_position = fallback_goal
                self.publish_goal(fallback_goal)
                self.publish_arrow_marker(fallback_goal)
                self.begin_nav = True
            return

    def bbox_callback(self, msg):

        self.try_bridge_ready()
        self.try_publish_bridge_goal()
        self.has_received_bbox = True
        self.last_bbox = MarkerArray()
        unmatched_box_count = 0 
        label_set = set()
        self.current_box = msg

        for marker in msg.markers:
            marker_id = str(marker.id)
            if marker.ns != "box":
                if marker.ns == "bridge":
                    self.bridge_found = True
                continue

            marker_id = str(marker.id)
            is_unmatched = True

            if marker_id in self.fusion_info:
                info = self.fusion_info[marker_id]
                if marker.pose.position.x > 5.0:
                    is_unmatched = not info.get("matched", False) 
                    if info.get("matched", True):
                        for h in info.get("history", []):
                            label_set.add(h["label"])
                    else:
                        self.last_bbox.markers.append(marker)
                    
                    if is_unmatched:
                        self.last_bbox.markers.append(marker)
                        unmatched_box_count += 1

            else:
                self.last_bbox.markers.append(marker)
        self.label_set = label_set
        self.last_bbox_msg = self.last_bbox
        # self.try_bridge_ready()
        rospy.loginfo(f"[BBoxCallback] Unmatched boxes received: {unmatched_box_count}")

        if self.in_fallback_mode and any(m.ns == "box" for m in self.last_bbox.markers):
            rospy.loginfo("[BBoxCallback] New unmatched boxes detected during fallback. Switching back to box navigation.")
            self.exit_fallback_mode()
            self.find_and_publish_new_goal(self.last_bbox)
        
        if self.bridge_ready and self.in_fallback_mode and self.begin_nav:
            rospy.loginfo("[BBoxCallback] Bridge now ready during fallback. Switching to bridge goal.")
            self.exit_fallback_mode()
            self.last_goal_position = self.bridge_position
            rospy.loginfo(f"[BBoxCallback-pubgoal] Bridge position: {self.bridge_position}")
            self.publish_goal(self.bridge_position)
            self.publish_arrow_marker(self.bridge_position)
            return

    def try_bridge_ready(self):
        box_count = sum(1 for m in self.current_box.markers if m.ns == "box" and m.pose.position.x > self.bridge_pos)
        bridge_marker = next((m for m in self.current_box.markers if m.ns == "bridge"), None)
        any_unmatched = any(not self.fusion_info.get(str(m.id), {}).get("matched", True)
                            for m in self.current_box.markers if m.ns == "box")

        if not any_unmatched and box_count >= 10 and len(self.label_set) >= 4 and bridge_marker:
            rospy.loginfo("[BridgeCheck] Bridge ready conditions met.")
            self.bridge_ready = True
            self.bridge_position = [bridge_marker.pose.position.x + self.bridge_length / 2 - 0.5,
                                    bridge_marker.pose.position.y, 0.0]
            self.post_bridge_position = [self.bridge_position[0] - self.bridge_length / 2,
                                         self.bridge_position[1], 0.0]
            rospy.loginfo(f"[BridgeCheck] box_count={box_count}, label_count={len(self.label_set)}, bridge_marker={bridge_marker is not None}, any_unmatched={any_unmatched}")

    def try_publish_bridge_goal(self):
        # self.try_bridge_ready()
        if self.bridge_ready and self.bridge_position and not self.bridge_goal_sent and self.begin_nav:
            rospy.loginfo("[BridgeGoal] Publishing bridge goal.")
            self.exit_fallback_mode()
            self.last_goal_position = self.bridge_position
            rospy.loginfo(f"[BridgeGoal-pubgoal] Bridge position: {self.bridge_position}")
            self.publish_goal(self.bridge_position, yaw_deg=180.0)
            self.publish_arrow_marker(self.bridge_position, yaw_deg=180.0)
            self.bridge_goal_sent = True
            return True
        return False

    def find_and_publish_new_goal(self, msg):
        if self.try_publish_bridge_goal():
            return
        # if self.bridge_ready and self.bridge_position and not self.bridge_goal_sent:
        #     rospy.loginfo("[Navigator] Conditions met, jumping to bridge goal!")
        #     self.last_goal_position = self.bridge_position
        #     self.publish_goal(self.bridge_position)
        #     self.publish_arrow_marker(self.bridge_position)
        #     self.bridge_goal_sent = True
        #     return self.last_goal_position
        
        max_distance = -1
        furthest_box_position = None
        trans, rot = self.tf_listener.lookupTransform("map", "base_link", rospy.Time(0))
        T = np.eye(4)
        T[:3, :3] = R.from_quat(rot).as_matrix()
        T[:3, 3] = trans
        self.current_pose = T
        
        for marker in msg.markers:
            if marker.ns != "box":
                continue
            if marker.pose.position.x <= self.bridge_pos:
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

        if furthest_box_position is not None and self.begin_nav:
            self.last_goal_position = furthest_box_position
            rospy.loginfo(f"[Navigator-pubgoal] Furthest box position: {robot_pos} to {furthest_box_position}")
            self.publish_goal(furthest_box_position)
            self.publish_arrow_marker(furthest_box_position)
            return furthest_box_position
        
    def find_fallback_goal(self):
        x_vals = np.arange(self.x_range[0], self.x_range[1], self.fallback_grid_resolution)
        y_vals = np.arange(self.y_range[0], self.y_range[1], self.fallback_grid_resolution)
        X, Y = np.meshgrid(x_vals, y_vals)
        mask = np.ones_like(X, dtype=bool)

        for marker in self.current_box.markers:
            if marker.ns != "box":
                continue
            bx, by = marker.pose.position.x, marker.pose.position.y
            mask &= ~((X >= bx - self.box_size/2) & (X <= bx + self.box_size/2) &
                      (Y >= by - self.box_size/2) & (Y <= by + self.box_size/2))

        labeled, num = label(mask)
        if num == 0:
            rospy.logwarn("[Fallback] No valid free space found.")
            return None

        largest = max(range(1, num + 1), key=lambda i: np.sum(labeled == i))
        indices = np.argwhere(labeled == largest)
        cy, cx = np.mean(indices, axis=0)
        center = [x_vals[int(cx)], y_vals[int(cy)], 0.0]
        rospy.loginfo(f"[Fallback] Selected fallback goal at {center}")
        return center

    def fusion_info_callback(self, msg):
        try:
            self.fusion_info = json.loads(msg.data)
        except Exception as e:
            rospy.logwarn(f"[FusionInfo] Failed to parse: {e}")

    def update_pose_from_tf(self, event):
        try:
            trans, rot = self.tf_listener.lookupTransform("map", "base_link", rospy.Time(0))
            T = np.eye(4)
            T[:3, :3] = R.from_quat(rot).as_matrix()
            T[:3, 3] = trans
            self.current_pose = T
        except Exception:
            rospy.logwarn("[TF] Failed to update pose")

    def publish_goal(self, position, yaw_deg=0.0):

        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.pose.position.x = position[0]
        msg.pose.position.y = position[1]
        msg.pose.position.z = position[2]

        q = R.from_euler('z', np.deg2rad(yaw_deg)).as_quat()
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]

        self.goal_pub.publish(msg)
        rospy.loginfo(f"[Goal] Published goal: {position} yaw={yaw_deg:.1f}")


    def publish_arrow_marker(self, position):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "goal_arrow"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        start = self.current_pose[:3, 3]
        marker.points = [Point(x=start[0], y=start[1], z=start[2]),
                         Point(x=position[0], y=position[1], z=position[2])]
        marker.scale.x = 0.1
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        self.marker_pub.publish(marker)

    def exit_fallback_mode(self):
        if self.in_fallback_mode:
            rospy.loginfo("[Navigator] Exiting fallback mode.")
        self.in_fallback_mode = False

    def enter_fallback_mode(self):
        if not self.in_fallback_mode:
            rospy.logwarn("[Navigator] Entering fallback mode.")
        self.in_fallback_mode = True

    def rot_end_callback(self, msg):
        if msg.data:  
            if self.try_publish_bridge_goal():
                return
            rospy.loginfo("[Rotation] One rotation finished. Computing new goal...")
            if self.last_bbox_msg:
                self.find_and_publish_new_goal(self.last_bbox_msg)

if __name__ == "__main__":
    try:
        FurthestBoxNavigator()
    except rospy.ROSInterruptException:
        pass
