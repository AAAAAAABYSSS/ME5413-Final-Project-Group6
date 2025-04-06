#!/usr/bin/env python
import tf
import rospy
import numpy as np
from visualization_msgs.msg import MarkerArray, Marker
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R

class BBoxTransformer:
    def __init__(self):
        rospy.init_node("bbox_transformer", anonymous=True)

        self.current_pose_inv = None  
        rospy.Subscriber("/perception/marker/nav_goal_marker", Marker, self.arrow_callback)

        self.arrow_pub = rospy.Publisher("/perception/marker/arrow_marker_baselink", Marker, queue_size=1)
        self.tf_listener = tf.TransformListener()
        self.current_pose = np.eye(4)
        rospy.loginfo("BBoxTransformer started.")


    def update_pose_from_tf(self):
        """Get transform from map to base and update current_pose"""
        try:
            # Velodyne to map transform
            (trans, rot) = self.tf_listener.lookupTransform(
                "base_link", "map", rospy.Time(0)
            )
            rotation_matrix = R.from_quat(rot).as_matrix()
            translation = np.array(trans)

            T_map_to_base = np.eye(4)
            T_map_to_base[:3, :3] = rotation_matrix
            T_map_to_base[:3, 3] = translation

            self.current_pose_inv = T_map_to_base
        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ):
            rospy.logwarn("TF lookup failed")

    def arrow_callback(self, marker):
        self.update_pose_from_tf()
        if self.current_pose_inv is None:
            rospy.logwarn("Waiting for the data from odom...")
            return

        transformed_marker = Marker()
        transformed_marker.header.frame_id = "base_link"
        transformed_marker.header.stamp = rospy.Time.now()
        transformed_marker.ns = marker.ns
        transformed_marker.id = marker.id
        transformed_marker.type = marker.type
        transformed_marker.action = Marker.ADD

        # Transformed between points[0] and points[1]
        from geometry_msgs.msg import Point
        transformed_points = []
        for pt in marker.points:
            pt_hom = np.array([pt.x, pt.y, pt.z, 1.0])
            pt_trans = self.current_pose_inv @ pt_hom
            transformed_points.append(Point(x=pt_trans[0], y=pt_trans[1], z=pt_trans[2]))
        transformed_marker.points = transformed_points

        transformed_marker.scale = marker.scale
        transformed_marker.color = marker.color
        transformed_marker.lifetime = rospy.Duration(1.0)

        self.arrow_pub.publish(transformed_marker)


if __name__ == "__main__":
    try:
        BBoxTransformer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
