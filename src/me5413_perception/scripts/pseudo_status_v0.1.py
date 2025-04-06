#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Always publish /move_base/status
import tf
import rospy
import numpy as np
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R

class PseudoStatus:
    def __init__(self):
        rospy.init_node("pseudo_status", anonymous=True)

        self.min_dist = rospy.get_param("~min_dist", 1.8)
        self.tf_listener = tf.TransformListener()
        self.current_pose = np.eye(4)
        self.goal_marker = None  # Store nav_goal_marker
        self.goal_received = False  # Whether a goal has been received

        # Subscriptions
        rospy.Subscriber("/perception/marker/nav_goal_marker", Marker, self.goal_callback)
        # rospy.Subscriber("/gazebo/ground_truth/state", Odometry, self.odom_callback)

        # Publish simulated move_base/status
        self.status_pub = rospy.Publisher("/move_base/status", Bool, queue_size=1)


        rospy.loginfo("PseudoStatus started.")

        self.rate = rospy.Rate(10)  # 10Hz
        # self.update_pose_from_tf()
        
        rospy.Timer(rospy.Duration(1.0), self.run_loop)
    def goal_callback(self, marker):
        self.goal_marker = marker
        self.goal_received = True

    # def odom_callback(self, msg):
    #     pos = msg.pose.pose.position
    #     self.current_pose = np.array([pos.x, pos.y, pos.z])

    def update_pose_from_tf(self):
        """Get transform from base_link to map and update current_pose"""
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
            rospy.loginfo(f"The current postion is {self.current_pose}")

        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ):
            rospy.logwarn("TF lookup failed")

    def run_loop(self, event=None): 
        self.update_pose_from_tf()
        # while not rospy.is_shutdown():
        if not self.goal_received:
            # No goal received, continuously publish True
            self.status_pub.publish(Bool(data=True))
            rospy.loginfo_throttle(5, "No goal yet, publishing True")
        else:
            if self.current_pose is not None:
                pt_goal = np.array([
                    self.goal_marker.points[1].x,
                    self.goal_marker.points[1].y,
                    self.goal_marker.points[1].z
                ])
                dist = np.linalg.norm(self.current_pose[:3, 3] - pt_goal)
                rospy.loginfo_throttle(1, f"Distance to goal: {dist:.2f} {pt_goal} {self.current_pose[:3, 3]}")

                if dist <= self.min_dist:
                    self.status_pub.publish(Bool(data=True))
                    rospy.loginfo("Goal reached: True")
                else:
                    self.status_pub.publish(Bool(data=False))
                    rospy.loginfo("Goal not reached: False")
            else:
                rospy.logwarn_throttle(5, "Waiting for odometry...")

            self.rate.sleep()

if __name__ == "__main__":
    try:
        PseudoStatus()
        rospy.spin()  
    except rospy.ROSInterruptException:
        pass
