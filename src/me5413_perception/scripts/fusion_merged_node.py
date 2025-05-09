#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import json
import math
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import String
from std_msgs.msg import Header
import threading
from collections import defaultdict


class FusionMergedNode:
    def __init__(self):
        rospy.init_node("fusion_merged_node", anonymous=False)

        self.box_big_size = rospy.get_param("~box_big_size", 1.0)
        self.max_history = rospy.get_param("~max_history", 30)
        self.unmatch_max = rospy.get_param("~unmatch_max", math.inf)
        self.point_inside_ratio_threshold = rospy.get_param("~point_inside_ratio_threshold", 0.6)

        self.yolo_targets = []
        self.history_labels = defaultdict(list)  # key: marker_id, value: list of {"label": str, "conf": float}
        
        self.match_state = {} 
        self.lock = threading.Lock()

        self.sub_yolo = rospy.Subscriber("/perception/yolo_targets_3d", String, self.yolo_callback)
        self.sub_bbox = rospy.Subscriber("/perception/marker/bbox_markers", MarkerArray, self.bbox_callback)

        self.pub_marker_fusion = rospy.Publisher("/perception/marker/bbox_markers_fusion", MarkerArray, queue_size=1)
        self.pub_fusion_info = rospy.Publisher("/perception/fusion_box_labels", String, queue_size=1)  
    
    def yolo_callback(self, msg):
        try:
            with self.lock:
                self.yolo_targets = json.loads(msg.data)
        except Exception as e:
            rospy.logwarn(f"[FusionMergedNode] YOLO targets parse failed: {e}")

    def bbox_callback(self, msg):
        # rospy.loginfo(f"[FusionMergedNode] Received {len(msg.markers)} markers")
        with self.lock:
            marker_out = MarkerArray()
            box_info_output = {}

            for marker in msg.markers:
                if marker.ns == "box":
                    center = marker.pose.position
                    extent = marker.scale

                    size_x = extent.x * self.box_big_size / 2.0
                    size_y = extent.y * self.box_big_size / 2.0
                    size_z = extent.z * self.box_big_size / 2.0

                    min_bound = [center.x - size_x, center.y - size_y, center.z - size_z]
                    max_bound = [center.x + size_x, center.y + size_y, center.z + size_z]
                    
                    matched = False
                    for target in self.yolo_targets:
                        total_pts = len(target["points_map"])
                        if total_pts == 0:
                            continue

                        inside = sum(
                            1 for pt in target["points_map"]
                            if min_bound[0] <= pt["x"] <= max_bound[0] and
                            min_bound[1] <= pt["y"] <= max_bound[1] and
                            min_bound[2] <= pt["z"] <= max_bound[2]
                        )

                        if inside / total_pts >= self.point_inside_ratio_threshold:
                            self.history_labels[marker.id].append({
                                "label": target["label"],
                                "conf": target["conf"],
                            })
                            matched = True


                    if len(self.history_labels[marker.id]) > self.max_history:
                        self.history_labels[marker.id] = self.history_labels[marker.id][-self.max_history:]
                    
                    if marker.id not in self.match_state:
                        self.match_state[marker.id] = False

                    if matched:
                        self.match_state[marker.id] = True  

                    final_matched = self.match_state[marker.id]  

                    marker_copy = Marker()
                    marker_copy.header = Header(frame_id="map", stamp=rospy.Time.now())
                    marker_copy.ns = marker.ns
                    marker_copy.id = marker.id
                    marker_copy.type = Marker.CUBE
                    marker_copy.action = Marker.ADD
                    marker_copy.pose = marker.pose
                    marker_copy.scale = marker.scale
                    marker_copy.lifetime = rospy.Duration(1.0)
                    marker_out.markers.append(marker_copy)

                    box_info_output[str(marker.id)] = {
                        "type": "box",
                        "matched": final_matched,
                        "history": self.history_labels[marker.id]
                    }
                elif marker.ns == "bridge":
                    marker_copy = Marker()
                    marker_copy.header = Header(frame_id="map", stamp=rospy.Time.now())
                    marker_copy.ns = marker.ns
                    marker_copy.id = marker.id
                    marker_copy.type = Marker.CUBE
                    marker_copy.action = Marker.ADD
                    marker_copy.pose = marker.pose
                    marker_copy.scale = marker.scale
                    marker_copy.lifetime = rospy.Duration(1.0)
                    marker_out.markers.append(marker_copy)
                    box_info_output[str(marker.id)] = {
                        "type": "bridge",
                        "matched": None,
                        "history": ""
                    }
            self.pub_marker_fusion.publish(marker_out)
            
            self.pub_fusion_info.publish(json.dumps(box_info_output))



if __name__ == '__main__':
    try:
        FusionMergedNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
