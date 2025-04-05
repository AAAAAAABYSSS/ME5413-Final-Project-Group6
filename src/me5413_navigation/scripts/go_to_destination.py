#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionGoal

class GoalPublisher:
    def __init__(self):
        """Initialize the Goal Publisher Node."""
        rospy.init_node('go_to_destination', anonymous=True)
        rospy.loginfo("Go to destination node started")
        
        # Subscriber to receive PoseStamped goal
        self.subscriber = rospy.Subscriber('/nav_goal', PoseStamped, self.receive_the_goal)
        
        # Publisher to send goal to move_base
        self.publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        
        # Variable to store goal
        self.goal = None
        
        self.rate = rospy.Rate(1)  # 1 Hz loop rate
    
    def receive_the_goal(self, msg):
        """Callback function to receive goal and store it."""
        rospy.loginfo("Received PoseStamped:\n Position: x=%.2f, y=%.2f, z=%.2f\n Orientation: x=%.2f, y=%.2f, z=%.2f, w=%.2f",
                      msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
                      msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
        
        msg.header.frame_id = 'map'  # Ensure frame is correct
        self.goal = msg  # Store the received goal
    
    def run(self):
        """Main loop to publish the goal."""
        while not rospy.is_shutdown():
            if self.goal is not None:
                rospy.loginfo("Publishing the goal to move_base")
                self.publisher.publish(self.goal)
                self.goal = None  # Reset after publishing to avoid re-sending
            self.rate.sleep()

if __name__ == '__main__':
    node = GoalPublisher()
    node.run()