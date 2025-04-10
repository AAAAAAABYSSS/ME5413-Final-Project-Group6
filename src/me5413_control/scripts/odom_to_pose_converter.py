#!/usr/bin/env python3
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import rospy

def callback(msg):
    out = PoseWithCovarianceStamped()
    out.header = msg.header
    out.pose = msg.pose
    pub.publish(out)

rospy.init_node("odom_to_pose_converter")
pub = rospy.Publisher("/ndt/pose_converted", PoseWithCovarianceStamped, queue_size=10)
rospy.Subscriber("/ndt/pose", Odometry, callback)
rospy.spin()
