import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

# def receive_the_location(msg):
#         """Callback function to receive location and print it"""
#         rospy.loginfo("Received PoseWithCovarianceStamped:\n Position: x=%.2f, y=%.2f, z=%.2f\n Orientation: x=%.2f, y=%.2f, z=%.2f, w=%.2f",
#                       msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z,
#                       msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)

# if __name__ == '__main__':
#     rospy.init_node('subscribe_location')
#     rospy.loginfo("Subscribe location node started")

#     # Subscriber to receive PoseWithCovarianceStamped location
#     rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, receive_the_location)

#     rospy.spin()

class SubscribeLocation:
    def __init__(self):
        rospy.init_node('subscribe_location')
        rospy.loginfo("Subscribe location node started")

        # Subscriber to receive PoseWithCovarianceStamped location
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.receive_the_location)

        rospy.spin()
    
    def receive_the_location(self, msg):
        """Callback function to receive location and print it"""
        rospy.loginfo("Received PoseWithCovarianceStamped:\n Position: x=%.2f, y=%.2f, z=%.2f\n Orientation: x=%.2f, y=%.2f, z=%.2f, w=%.2f",
                      msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z,
                      msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
    
    def keep_run(self):
        rospy.spin()

if __name__ == '__main__':
    sub_loc = SubscribeLocation()
    sub_loc.keep_run()