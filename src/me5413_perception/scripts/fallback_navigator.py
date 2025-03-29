#!/usr/bin/env python3
import tf
import rospy
import json
import numpy as np
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped, Point
from scipy.spatial.transform import Rotation as R
from visualization_msgs.msg import Marker


class FallbackNavigator:
    def __init__(self):
        rospy.init_node("fallback_navigator", anonymous=True)

        self.inflate_ratio = rospy.get_param("~inflate_ratio", 1.2)
        self.max_inflate_ratio = rospy.get_param("~max_inflate_ratio", 2.5)
        self.inflate_step = rospy.get_param("~inflate_step", 0.2)
        self.box_size = rospy.get_param("~box_size", 0.8)
        self.x_range = rospy.get_param("~x_range", [2.0, 22.0])
        self.y_range = rospy.get_param("~y_range", [11.0, 19.0])
        self.wait_time = rospy.get_param("~wait_time", 0.5)
        self.grid_resolution = rospy.get_param("~grid_resolution", 0.2)

        self.enable_fallback = False
        self.status_sub = rospy.Subscriber("/move_base/status", Bool, self.status_callback)
        self.odom_sub = rospy.Subscriber("/gazebo/ground_truth/state", Odometry, self.odom_callback)
        
        self.fallback_trigger_sub = rospy.Subscriber("/navigation/fallback_trigger", Bool, self.trigger_callback)

        self.cluster_sub = rospy.Subscriber("/perception/fusion_box_labels", String, self.cluster_callback)
        
        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        self.marker_pub = rospy.Publisher("/perception/marker/nav_goal_marker", Marker, queue_size=1)

        self.tf_listener = tf.TransformListener()
        self.current_pose = np.eye(4)

        self.current_boxes = []
        self.tried_goals = []
        self.goal_active = False
        self.waiting = False
        self.last_goal = None

        rospy.Timer(rospy.Duration(1.0), self.main_loop)
        rospy.loginfo("[FallbackNavigator] Node started.")
        rospy.spin()

    def trigger_callback(self, msg):
        self.enable_fallback = msg.data
        if self.enable_fallback:
            rospy.logwarn("[FallbackNavigator] Fallback mode ENABLED by main navigator.")
    
    def odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        translation = np.array([position.x, position.y, position.z])
        rotation = R.from_quat([orientation.x, orientation.y, orientation.z, orientation.w]).as_matrix()

        self.current_pose = np.eye(4)
        self.current_pose[:3, :3] = rotation
        self.current_pose[:3, 3] = translation

    def cluster_callback(self, msg):
        try:
            fusion_dict = json.loads(msg.data)
            self.current_boxes = []

            for marker_id, info in fusion_dict.items():
                if not info.get("matched", True):
                    center = self.get_center_from_id(marker_id)
                    if center:
                        self.current_boxes.append({
                            "x": center[0],
                            "y": center[1],
                            "size": self.box_size
                        })
        except Exception as e:
            rospy.logwarn("[FallbackNavigator] Failed to parse cluster info: %s" % e)

    def get_center_from_id(self, marker_id):
        try:
            marker_id = int(marker_id)
            col = marker_id % 10
            row = marker_id // 10
            x = self.x_range[0] + col * self.box_size
            y = self.y_range[0] + row * self.box_size
            return [x, y]
        except:
            return None

    def status_callback(self, msg):
        if self.goal_active and msg.data:
            rospy.loginfo("[FallbackNavigator] Reached fallback goal.")
            self.goal_active = False
            self.waiting = True
            rospy.Timer(rospy.Duration(self.wait_time), self.resume_after_wait, oneshot=True)

    def resume_after_wait(self, event):
        rospy.loginfo("[FallbackNavigator] Resuming fallback scan...")
        self.waiting = False

    def main_loop(self, event):
        if not self.enable_fallback:
            return
        if self.goal_active or self.waiting:
            return

        rospy.loginfo("[FallbackNavigator] Attempting to find fallback goal with grid analysis...")
        fallback_goal = self.find_fallback_goal()
        if fallback_goal is not None:
            self.publish_goal(fallback_goal)
            self.publish_arrow_marker(fallback_goal)
            # self.publish_markers(inflate, [x, y])
            self.goal_active = True
            self.last_goal = fallback_goal
        else:
            rospy.logwarn("[FallbackNavigator] No fallback goal found after grid analysis.")

    def find_fallback_goal(self):
        x_vals = np.arange(self.x_range[0], self.x_range[1], self.grid_resolution)
        y_vals = np.arange(self.y_range[0], self.y_range[1], self.grid_resolution)
        X, Y = np.meshgrid(x_vals, y_vals)
        mask = np.ones_like(X, dtype=bool)

        for box in self.current_boxes:
            bx, by = box["x"], box["y"]
            xmin = bx - self.box_size / 2 * self.inflate_ratio
            xmax = bx + self.box_size / 2 * self.inflate_ratio
            ymin = by - self.box_size / 2 * self.inflate_ratio
            ymax = by + self.box_size / 2 * self.inflate_ratio
            occupied = (X >= xmin) & (X <= xmax) & (Y >= ymin) & (Y <= ymax)
            mask[occupied] = False

        free_points = np.column_stack((X[mask], Y[mask]))
        if len(free_points) == 0:
            return None

        if len(self.current_boxes) > 0:
            box_centers = np.array([[b["x"], b["y"]] for b in self.current_boxes])
            dists = np.min(np.linalg.norm(free_points[:, None, :] - box_centers[None, :, :], axis=2), axis=1)
            fallback_point = free_points[np.argmax(dists)]
        else:
            fallback_point = free_points[len(free_points) // 2]

        return [fallback_point[0], fallback_point[1], 0.0]

    def publish_goal(self, position):
        goal_msg = PoseStamped()
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.header.frame_id = "map"
        goal_msg.pose.position.x = position[0]
        goal_msg.pose.position.y = position[1]
        goal_msg.pose.position.z = position[2]
        goal_msg.pose.orientation.w = 1.0
        self.goal_pub.publish(goal_msg)
        rospy.loginfo(f"[FallbackNavigator] Target point: x={position[0]:.2f}, y={position[1]:.2f}, z={position[2]:.2f}")

    def publish_arrow_marker(self, position):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "goal_arrow"
        marker.id = 10001  
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        start_point = self.current_pose[:3, 3]
        marker.points = [
            Point(x=start_point[0], y=start_point[1], z=start_point[2]),
            Point(x=position[0], y=position[1], z=position[2])
        ]
        marker.scale.x = 0.1
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.lifetime = rospy.Duration(1.0)
        self.marker_pub.publish(marker)

        # def publish_markers(self, ratio, goal):
        #     marker_array = MarkerArray()
        #     now = rospy.Time.now()
        #     for i, box in enumerate(self.current_boxes):
        #         marker = Marker()
        #         marker.header.frame_id = "map"
        #         marker.header.stamp = now
        #         marker.ns = "inflated_box"
        #         marker.id = i
        #         marker.type = Marker.CUBE
        #         marker.action = Marker.ADD
        #         marker.pose.position.x = box["x"]
        #         marker.pose.position.y = box["y"]
        #         marker.pose.position.z = 0.1
        #         marker.scale.x = marker.scale.y = box["size"] * ratio
        #         marker.scale.z = 0.2
        #         marker.color.r = 1.0
        #         marker.color.g = 1.0
        #         marker.color.b = 0.0
        #         marker.color.a = 0.3
        #         marker.lifetime = rospy.Duration(1.0)
        #         marker_array.markers.append(marker)

        #     goal_marker = Marker()
        #     goal_marker.header.frame_id = "map"
        #     goal_marker.header.stamp = now
        #     goal_marker.ns = "fallback_goal"
        #     goal_marker.id = 9999
        #     goal_marker.type = Marker.SPHERE
        #     goal_marker.action = Marker.ADD
        #     goal_marker.pose.position.x = goal[0]
        #     goal_marker.pose.position.y = goal[1]
        #     goal_marker.pose.position.z = 0.2
        #     goal_marker.scale.x = goal_marker.scale.y = goal_marker.scale.z = 0.3
        #     goal_marker.color.r = 0.0
        #     goal_marker.color.g = 0.5
        #     goal_marker.color.b = 1.0
        #     goal_marker.color.a = 0.8
        #     goal_marker.lifetime = rospy.Duration(1.0)
        #     marker_array.markers.append(goal_marker)

        #     self.marker_pub.publish(marker_array)


if __name__ == '__main__':
    try:
        FallbackNavigator()
    except rospy.ROSInterruptException:
        pass
