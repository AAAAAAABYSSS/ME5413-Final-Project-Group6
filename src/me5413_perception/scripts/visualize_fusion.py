#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import json
import numpy as np
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from collections import defaultdict
from scipy.spatial.transform import Rotation as R
from visualization_msgs.msg import MarkerArray, Marker

class FusionVisualizer:
    def __init__(self):
        rospy.init_node("visualize_fusion", anonymous=False)

        self.fusion_info = {}  # marker_id -> list of {label, conf}
        rospy.Subscriber("/gazebo/ground_truth/state", Odometry, self.odom_callback)
        self.sub_info = rospy.Subscriber("/perception/fusion_box_labels", String, self.fusion_info_callback)
        self.sub_markers = rospy.Subscriber("/perception/marker/bbox_markers_fusion", MarkerArray, self.marker_callback)
        
        self.pub_visual = rospy.Publisher("/perception/marker/bbox_markers_visualized", MarkerArray, queue_size=1)

    def fusion_info_callback(self, msg):
        try:
            self.fusion_info = json.loads(msg.data)
        except Exception as e:
            rospy.logwarn(f"[FusionVisualizer] Failed to parse fusion info: {e}")

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        T_map2base = np.eye(4)
        T_map2base[:3, 3] = [position.x, position.y, position.z]
        rot_matrix = R.from_quat([orientation.x, orientation.y, orientation.z, orientation.w]).as_matrix()
        T_map2base[:3, :3] = rot_matrix

        self.current_pose_inv = np.linalg.inv(T_map2base)

    def marker_callback(self, msg):
        if self.current_pose_inv is None:
            rospy.logwarn("[FusionVisualizer] Waiting for odom data...")
            return
        visual_out = MarkerArray()

        for marker in msg.markers:
            marker_id = str(marker.id)
            new_marker = Marker()
            new_marker.header.frame_id = "base_link"  
            new_marker.header.stamp = rospy.Time.now()
            new_marker.ns = "visualized"
            new_marker.id = marker.id
            new_marker.type = Marker.CUBE
            new_marker.action = Marker.ADD

            center_point = np.array([
                marker.pose.position.x,
                marker.pose.position.y,
                marker.pose.position.z,
                1.0
            ])

            transformed_center = self.current_pose_inv @ center_point

            q_marker = [
                marker.pose.orientation.x,
                marker.pose.orientation.y,
                marker.pose.orientation.z,
                marker.pose.orientation.w
            ]

            rot_marker = R.from_quat(q_marker)

            rot_map2base_mat = self.current_pose_inv[:3, :3]  
            rot_map2base = R.from_matrix(rot_map2base_mat)  
            rot_transformed = rot_map2base * rot_marker
            q_new = rot_transformed.as_quat()

            new_marker.pose.position.x = transformed_center[0]
            new_marker.pose.position.y = transformed_center[1]
            new_marker.pose.position.z = transformed_center[2]

            new_marker.pose.orientation.x = q_new[0]
            new_marker.pose.orientation.y = q_new[1]
            new_marker.pose.orientation.z = q_new[2]
            new_marker.pose.orientation.w = q_new[3]

            new_marker.scale.x = marker.scale.x 
            new_marker.scale.y = marker.scale.y 
            new_marker.scale.z = marker.scale.z 

            # new_marker.color.r = marker.color.r
            # new_marker.color.g = marker.color.g
            # new_marker.color.b = marker.color.b
            # new_marker.color.a = marker.color.a

            new_marker.lifetime = rospy.Duration(0.5)

            label_stats = defaultdict(list)
            best_label = None
            best_conf = 0.0

            if marker_id in self.fusion_info:
                info = self.fusion_info[marker_id]

                if info.get("matched", False):
                    new_marker.color.r = 0.0
                    new_marker.color.g = 1.0
                    new_marker.color.b = 0.0
                    new_marker.color.a = 0.5
                else:
                    new_marker.color.r = 1.0
                    new_marker.color.g = 0.0
                    new_marker.color.b = 0.0
                    new_marker.color.a = 0.

                if "history" in info:
                    for entry in info["history"]:
                        label_stats[entry["label"]].append(entry["conf"])

                    if label_stats:
                        best_label, best_conf = max(
                            ((lbl, sum(confs)/len(confs)) for lbl, confs in label_stats.items()),
                            key=lambda x: x[1]
                        )

            if best_label is not None:
                new_marker.text = f"{best_label}: {round(best_conf, 2)}"
            else:
                new_marker.text = ""


            # new_marker = Marker()
            # new_marker.header = marker.header
            # new_marker.ns = "visualized"
            # new_marker.id = marker.id
            # new_marker.type = Marker.CUBE
            # new_marker.action = Marker.ADD
            # new_marker.pose = marker.pose
            # new_marker.scale = marker.scale
            # new_marker.color = marker.color
            # new_marker.lifetime = rospy.Duration(1.0)

            visual_out.markers.append(new_marker)

        self.pub_visual.publish(visual_out)


if __name__ == '__main__':
    try:
        FusionVisualizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
