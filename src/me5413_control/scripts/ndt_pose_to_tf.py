#!/usr/bin/env python3
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
import rospy, tf2_ros

def callback(msg):
    t = TransformStamped()
    t.header.stamp = msg.header.stamp
    t.header.frame_id = "map"  
    t.child_frame_id = "odom"
    t.transform.translation = msg.pose.pose.position
    t.transform.rotation = msg.pose.pose.orientation
    br.sendTransform(t)

rospy.init_node("ndt_pose_to_tf")
br = tf2_ros.TransformBroadcaster()
rospy.Subscriber("/ndt/pose_converted", PoseWithCovarianceStamped, callback)
rospy.spin()
