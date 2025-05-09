#!/usr/bin/env python3
import rospy
import tf
import json
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped, Twist, Point
from visualization_msgs.msg import MarkerArray, Marker
from move_base_msgs.msg import MoveBaseAction
from transitions import Machine
import numpy as np
from scipy.spatial.transform import Rotation as R
import actionlib


class AdaptiveNavigatorFSM(Machine):
    def __init__(self, robot):
        self.robot = robot
        states = [
            'idle',
            'waiting_for_bbox',
            'navigating_to_goal',
            'fallback_navigation',
        ]

        transitions = [
            {'trigger': 'start', 'source': 'idle', 'dest': 'waiting_for_bbox'},
            {'trigger': 'got_valid_bbox', 'source': 'waiting_for_bbox', 'dest': 'navigating_to_goal'},
            {'trigger': 'no_bbox_available', 'source': 'waiting_for_bbox', 'dest': 'fallback_navigation'},
            {'trigger': 'new_boxes_during_fallback', 'source': 'fallback_navigation', 'dest': 'navigating_to_goal'},
            {'trigger': 'reset', 'source': '*', 'dest': 'idle'}
        ]

        super().__init__(model=self, states=states, transitions=transitions, initial='idle', auto_transitions=False)

        for state in states:
            method = f'on_enter_{state}'
            if not hasattr(self, method):
                setattr(self, method, self.default_on_enter)

    def default_on_enter(self):
        rospy.loginfo(f"[FSM] Entered state: {self.state}")

    def on_enter_navigating_to_goal(self):
        rospy.loginfo("[FSM] Navigating to goal box.")
        if self.robot.last_bbox_msg:
            self.robot.find_and_publish_goal(self.robot.last_bbox_msg)

    def on_enter_fallback_navigation(self):
        rospy.loginfo("[FSM] No box found. Executing fallback navigation.")
        fallback = self.robot.fallback_goal()
        if fallback:
            self.robot.publish_goal(fallback)
            self.robot.publish_arrow_marker(fallback)


class AdaptiveNavigator:
    def __init__(self):
        rospy.init_node("adaptive_navigator")

        self.tf_listener = tf.TransformListener()
        self.current_pose = np.eye(4)
        self.last_bbox_msg = None
        self.label_set = set()
        self.box_size = 0.8

        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        self.marker_pub = rospy.Publisher("/perception/marker/nav_goal_marker", Marker, queue_size=1)

        rospy.Subscriber("/perception/marker/bbox_markers_fusion", MarkerArray, self.bbox_callback)
        rospy.Subscriber("/perception/fusion_box_labels", String, self.fusion_info_callback)

        self.fusion_info = {}
        self.state_machine = AdaptiveNavigatorFSM(self)
        self.state_machine.start()

        rospy.Timer(rospy.Duration(1.0), self.update_pose_from_tf)
        rospy.loginfo("[AdaptiveNavigator] Initialized.")
        rospy.spin()

    def update_pose_from_tf(self, _):
        try:
            trans, rot = self.tf_listener.lookupTransform("map", "base_link", rospy.Time(0))
            T = np.eye(4)
            T[:3, :3] = R.from_quat(rot).as_matrix()
            T[:3, 3] = trans
            self.current_pose = T
        except Exception:
            rospy.logwarn("[TF] Failed to update pose")

    def bbox_callback(self, msg):
        self.last_bbox_msg = msg
        unmatched = [m for m in msg.markers if m.ns == "box"]
        rospy.loginfo(f"[BBoxCallback] Received {len(unmatched)} boxes")

        if self.state_machine.state == "waiting_for_bbox":
            if unmatched:
                self.state_machine.got_valid_bbox()
            else:
                self.state_machine.no_bbox_available()
        elif self.state_machine.state == "fallback_navigation" and unmatched:
            self.state_machine.new_boxes_during_fallback()

    def fusion_info_callback(self, msg):
        try:
            self.fusion_info = json.loads(msg.data)
        except Exception as e:
            rospy.logwarn(f"[FusionInfo] JSON parse error: {e}")

    def find_and_publish_goal(self, msg):
        max_dist, target = -1, None
        for m in msg.markers:
            if m.ns != "box":
                continue
            pos = np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z])
            dist = np.linalg.norm(self.current_pose[:3, 3][:2] - pos[:2])
            if dist > max_dist:
                max_dist, target = dist, pos

            box_pos_target = target.copy()
            # Plan B
            if target[1] > -4.0:
                box_pos_target[1] = target[1] - 0.8
            elif target[1] < -4.0:
                box_pos_target[1] = target[1] + 0.8
        if box_pos_target is not None:
            self.publish_goal(box_pos_target)
            self.publish_arrow_marker(box_pos_target)

    def fallback_goal(self):
        return [14.0, -12.0, 0.0]  # Simple placeholder fallback

    def publish_goal(self, pos, yaw=0.0):
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = pos
        q = R.from_euler('z', np.deg2rad(yaw)).as_quat()
        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = q
        self.goal_pub.publish(msg)
        rospy.loginfo(f"[Goal] Published goal at {pos}")

    def publish_arrow_marker(self, pos):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "nav_arrow"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.points = [Point(*self.current_pose[:3, 3]), Point(*pos)]
        marker.scale.x, marker.scale.y, marker.scale.z = 0.1, 0.2, 0.2
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.0, 1.0, 0.0, 1.0
        self.marker_pub.publish(marker)


if __name__ == '__main__':
    try:
        AdaptiveNavigator()
    except rospy.ROSInterruptException:
        pass