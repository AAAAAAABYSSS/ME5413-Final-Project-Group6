#!/usr/bin/env python3
import tf
import json
import rospy
import numpy as np
from scipy.ndimage import label
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import MarkerArray, Marker

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

        self.fallback_grid_resolution = rospy.get_param("~fallback_grid_resolution", 0.2)
        self.x_range = rospy.get_param("~x_range", [11.0, 19.0])
        self.y_range = rospy.get_param("~y_range", [-22.0, -2.0])
        self.bridge_pos = rospy.get_param("~bridge_pos", 9.0)


        self.reach_goal = True
        self.last_goal_position = None
        self.last_bbox_msg = None
        self.finished_directions = set()

        self.fusion_info = {}

        self.bridge_ready = False
        self.bridge_position = None

        self.in_fallback_mode = False
        self.has_received_bbox = False
        rospy.Subscriber("/move_base/status", Bool, self.status_callback)
        # rospy.Subscriber("/gazebo/ground_truth/state", Odometry, self.odom_callback)
        rospy.Subscriber("/perception/marker/bbox_markers_fusion", MarkerArray, self.bbox_callback)
        rospy.Subscriber("/perception/fusion_box_labels", String, self.fusion_info_callback)

        rospy.Subscriber("/one_rot/finish_status", Bool, self.rot_status_callback)
        rospy.Subscriber("/one_rot/end_status", Bool, self.rot_end_callback)

        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        self.marker_pub = rospy.Publisher("/perception/marker/nav_goal_marker", Marker, queue_size=1)
        self.unfinished_pub = rospy.Publisher("/one_rot/unfinished_boxes", String, queue_size=1)

        rospy.Timer(rospy.Duration(1.0), self.main_loop)
        rospy.loginfo(f"FurthestBoxNavigator Initialization completed")
        rospy.spin()

    def enter_fallback_mode(self):
        self.in_fallback_mode = True
        rospy.logwarn("[Navigator] Entering fallback mode...")

    def exit_fallback_mode(self):
        self.in_fallback_mode = False
        rospy.loginfo("[Navigator] Recovered from fallback, returning to normal navigation.")

    def main_loop(self, event):
        self.update_pose_from_tf()
        if not self.reach_goal:
            return

        # 0：Waiting for the position of bboxes
        if not self.has_received_bbox:
            rospy.loginfo_throttle(5.0, "[Main Loop] Awaiting first bbox detection... Standing by.")
            return

        # 1：Brige
        if self.bridge_ready:
            if not np.allclose(self.last_goal_position, self.bridge_position, atol=0.1):
                rospy.loginfo("[Main Loop] Bridge is ready, switching goal to bridge.")
                self.exit_fallback_mode()
                self.last_goal_position = self.bridge_position
                self.publish_goal(self.bridge_position)
                self.publish_arrow_marker(self.bridge_position)
            return

        # 2：Unmatched box
        if self.last_bbox_msg and any(m.ns == "box" for m in self.last_bbox_msg.markers):
            if self.in_fallback_mode:
                self.exit_fallback_mode()
            rospy.loginfo("[Main Loop] Navigating to furthest unmatched box.")
            self.find_and_publish_new_goal(self.last_bbox_msg)
            return

        # 3：Fallback
        if self.last_bbox_msg and not any(m.ns == "box" for m in self.last_bbox_msg.markers):
            if self.bridge_ready:
                rospy.loginfo("[Main Loop] All boxes matched, waiting for bridge goal.")
                return
            else:
                rospy.logwarn("[Main Loop] All boxes matched, bridge not ready or not enough box. Entering fallback.")
                if not self.in_fallback_mode:
                    self.enter_fallback_mode()
                fallback_goal = self.find_fallback_goal()
                if fallback_goal:
                    self.last_goal_position = fallback_goal
                    self.publish_goal(fallback_goal)
                    self.publish_arrow_marker(fallback_goal)
                return

    def find_fallback_goal(self):
        label_set = set()
        box_count = 0
        for marker_id, info in self.fusion_info.items():
            matched = info.get("matched", False)
            if matched:
                for h in info.get("history", []):
                    label_set.add(h["label"])
                box_count += 1

        if box_count >= 10 and len(label_set) >= 4 and not self.bridge_ready:
            rospy.loginfo("[Fallback] Box and label threshold met, approaching center to wait for bridge...")
            return [10.5, -11.0 , self.current_pose[2, 3]]
    
        x_vals = np.arange(self.x_range[0], self.x_range[1], self.fallback_grid_resolution)
        y_vals = np.arange(self.y_range[0], self.y_range[1], self.fallback_grid_resolution)
        X, Y = np.meshgrid(x_vals, y_vals)
        mask = np.ones_like(X, dtype=bool)

        for marker in (self.last_bbox_msg.markers if self.last_bbox_msg else []):
            if marker.ns != "box":
                continue
            bx, by = marker.pose.position.x, marker.pose.position.y
            xmin = bx - self.box_size / 2
            xmax = bx + self.box_size / 2
            ymin = by - self.box_size / 2
            ymax = by + self.box_size / 2
            occupied = (X >= xmin) & (X <= xmax) & (Y >= ymin) & (Y <= ymax)
            mask[occupied] = False

        # free_points = np.column_stack((X[mask], Y[mask]))
        # if len(free_points) == 0:
        #     return None

        # robot_pos = self.current_pose[:3, 3]
        # dists = np.linalg.norm(free_points - robot_pos[:2], axis=1)
        # fallback_point = free_points[np.argmax(dists)]

        # return [fallback_point[0], fallback_point[1], robot_pos[2]]
        labeled_mask, num_features = label(mask)

        max_area = 0
        max_label = -1
        for i in range(1, num_features + 1):
            area = np.sum(labeled_mask == i)
            if area > max_area:
                max_area = area
                max_label = i

        region_indices = np.argwhere(labeled_mask == max_label)
        if len(region_indices) == 0:
            rospy.logwarn("[Fallback] No valid connected free space found.")
            return None

        center_index = np.mean(region_indices, axis=0)  # [row_idx, col_idx]
        center_y = y_vals[int(round(center_index[0]))]
        center_x = x_vals[int(round(center_index[1]))]
        robot_z = self.current_pose[2, 3]

        rospy.loginfo(f"[Fallback] Chosen center of largest free region at x={center_x:.2f}, y={center_y:.2f}")
        return [center_x, center_y, 0.0]

    def fusion_info_callback(self, msg):
        try:
            self.fusion_info = json.loads(msg.data)
        except Exception as e:
            rospy.logwarn(f"[Fusion Info] Failed to parse: {e}")

    # def odom_callback(self, msg):
    #     position = msg.pose.pose.position
    #     orientation = msg.pose.pose.orientation

    #     translation = np.array([position.x, position.y, position.z])
    #     rotation = R.from_quat([orientation.x, orientation.y, orientation.z, orientation.w]).as_matrix()

    #     self.current_pose = np.eye(4)
    #     self.current_pose[:3, :3] = rotation
    #     self.current_pose[:3, 3] = translation

    def update_pose_from_tf(self):
        """Get transform from Baselink to map and update current_pose"""
        try:
            # Baselink to map transform
            (trans, rot) = self.tf_listener.lookupTransform(
                "map", "base_link", rospy.Time(0)
            )
            rotation_matrix = R.from_quat(rot).as_matrix()
            translation = np.array(trans)

            T_base_to_map = np.eye(4)
            T_base_to_map[:3, :3] = rotation_matrix
            T_base_to_map[:3, 3] = translation

            self.current_pose = T_base_to_map
            # rospy.loginfo(f"The current postion is {self.current_pose}")

        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ):
            rospy.logwarn("TF lookup failed")

    def bbox_callback(self, msg):
        self.has_received_bbox = True
        self.last_bbox = MarkerArray()
        unmatched_box_count = 0 
        label_set = set()
        bridge_found = False
        
        for marker in msg.markers:
            marker_id = str(marker.id)
            if marker.ns != "box":
                if marker.ns == "bridge":
                    bridge_found = True
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
        self.last_bbox_msg = self.last_bbox
        self.try_publish_bridgehead_if_ready(msg, label_set, bridge_found)
        rospy.loginfo(f"[BBoxCallback] Unmatched boxes received: {unmatched_box_count}")

        if self.in_fallback_mode and any(m.ns == "box" for m in self.last_bbox.markers):
            rospy.loginfo("[BBoxCallback] New unmatched boxes detected during fallback. Switching back to box navigation.")
            self.exit_fallback_mode()
            self.find_and_publish_new_goal(self.last_bbox)
        
        if self.bridge_ready and self.in_fallback_mode:
            rospy.loginfo("[BBoxCallback] Bridge now ready during fallback. Switching to bridge goal.")
            self.exit_fallback_mode()
            self.last_goal_position = self.bridge_position
            self.publish_goal(self.bridge_position)
            self.publish_arrow_marker(self.bridge_position)
            return
        
    
    def try_publish_bridgehead_if_ready(self, msg, label_set, bridge_found):
        box_count = 0
        bridge_count = 0
        bridge_position = None
        any_unmatched = False

        for marker in msg.markers:
            if marker.ns == "box":
                if marker.pose.position.y > 9:
                    box_count += 1
                    
                marker_id = str(marker.id)
                matched = self.fusion_info.get(marker_id, {}).get("matched", True)
                if not matched:
                    any_unmatched = True

            elif marker.ns == "bridge":
                bridge_count += 1
                bridge_position = marker.pose.position

        if (not any_unmatched and box_count >= 10 and bridge_count == 1 and
            len(label_set) >= 4 and bridge_position):
            rospy.loginfo(f"[BridgeCheck] box_count={box_count}, bridge_count={bridge_count}, "
              f"label_set={label_set} "
              f"bridge_position_y={(bridge_position.y if bridge_position else None)}")
            
            rospy.loginfo("[Navigator] All boxes matched and 4 labels found, bridge exists.")
            self.bridge_ready = True
            self.bridge_position = [bridge_position.x, bridge_position.y + self.bridge_length/2, bridge_position.z]

    def status_callback(self, msg):
        self.reach_goal = msg.data
        if self.reach_goal:
            rospy.loginfo("Target reached, computing new target...")

            if self.bridge_ready:
                self.exit_fallback_mode()
                rospy.loginfo("[Navigator] Publishing bridge goal...")
                self.last_goal_position = self.bridge_position
                self.publish_goal(self.bridge_position)
                self.publish_arrow_marker(self.bridge_position)
                self.bridge_ready = False
            elif self.last_bbox_msg and any(m.ns == "box" for m in self.last_bbox_msg.markers):
                self.exit_fallback_mode()
                self.find_and_publish_new_goal(self.last_bbox_msg)

            else:
                fallback_goal = self.find_fallback_goal()
                if fallback_goal:
                    rospy.loginfo("[Navigator] Continuing fallback...")
                    self.enter_fallback_mode()
                    self.last_goal_position = fallback_goal
                    self.publish_goal(fallback_goal)
                    self.publish_arrow_marker(fallback_goal)
        else:
            if self.last_goal_position is not None:
                rospy.loginfo("Continue current target...")
                self.publish_goal(self.last_goal_position)
                self.publish_arrow_marker(self.last_goal_position)
            

    def find_and_publish_new_goal(self, msg):
        if self.bridge_ready and self.bridge_position:
            rospy.loginfo("[Navigator] Conditions met, jumping to bridge goal!")
            self.last_goal_position = self.bridge_position
            self.publish_goal(self.bridge_position)
            self.publish_arrow_marker(self.bridge_position)
            return
        
        max_distance = -1
        furthest_box_position = None

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
