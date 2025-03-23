import rospy
import json
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
from std_msgs.msg import String

class YOLORVizMarker:
    def __init__(self):
        rospy.init_node("yolo_rviz_marker", anonymous=True)
        self.marker_pub = rospy.Publisher("/yolo_markers", MarkerArray, queue_size=10)
        self.yolo_targets_3d = []
        rospy.Subscriber("/yolo_targets_3d", String, self.yolo_callback)
        self.rate = rospy.Rate(10)

    def yolo_callback(self, msg):
        try:
            self.yolo_targets_3d = json.loads(msg.data)  # Parse JSON data
        except Exception as e:
            rospy.logerr(f"Error parsing YOLO 3D target JSON: {e}")

    def filter_close_points(self, targets, min_distance=0.2):
        """
        Filter out target points that are too close to each other
        :param targets: Original YOLO target points list
        :param min_distance: Minimum distance threshold, points closer than this will be merged
        :return: Filtered target points list
        """
        filtered_targets = []
        
        for target in targets:
            x, y, z = target["X"], target["Y"], target["Z"]
            
            too_close = any(
                np.linalg.norm(np.array([x, y, z]) - np.array([t["X"], t["Y"], t["Z"]])) < min_distance
                for t in filtered_targets
            )

            if not too_close:
                filtered_targets.append(target)

        rospy.loginfo(f"Filtered target count: {len(filtered_targets)} (Original: {len(targets)})")
        return filtered_targets

    def create_markers(self):
        marker_array = MarkerArray()
        marker_id = 0

        filtered_targets = self.filter_close_points(self.yolo_targets_3d)

        for target in filtered_targets:
            x, y, z = target["X"], target["Y"], target["Z"]
            label = target["label"]
            conf = target["conf"]

            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "yolo_markers"
            marker.id = marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = z
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)
            marker.lifetime = rospy.Duration(1.0)
            marker_array.markers.append(marker)
            marker_id += 1

            text_marker = Marker()
            text_marker.header.frame_id = "base_link"
            text_marker.header.stamp = rospy.Time.now()
            text_marker.ns = "yolo_labels"
            text_marker.id = marker_id
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = x
            text_marker.pose.position.y = y
            text_marker.pose.position.z = z + 0.2
            text_marker.scale.z = 0.2
            text_marker.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
            text_marker.text = f"Label: {label}\nConf: {conf:.2f}"
            text_marker.lifetime = rospy.Duration(1.0)
            marker_array.markers.append(text_marker)
            marker_id += 1

        return marker_array

    def publish_markers(self):
        while not rospy.is_shutdown():
            self.marker_pub.publish(self.create_markers())
            self.rate.sleep()

if __name__ == "__main__":
    try:
        marker_publisher = YOLORVizMarker()
        marker_publisher.publish_markers()
    except rospy.ROSInterruptException:
        pass