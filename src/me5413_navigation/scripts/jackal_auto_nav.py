#!/usr/bin/env python

import rospy
import tf
import math
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, Twist
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Bool


class NavigationSuccessChecker:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node("jackal_auto_rot_nav")

        # Initialize the variables
        self.previous_position = None
        self.previous_orientation = None
        self.similarity_count = 0
        self.unsimilarity_count = 0
        self.rot_end_status = False
        self.goal = []

        # Set the thresholds
        self.position_threshold = 0.08 #0.01  # position difference threshold in meters
        self.orientation_threshold = 0.01  # orientation difference threshold in radians
        self.max_similarity_count = (
            30  # Number of similar frames required to trigger success
        )
        self.max_unsimilarity_count = (
            100  # Number of unsimilar frames required to trigger unsuccess
        )
        self.min_ego_goal_dist = (
            2.0  # Number of similar frames required to trigger success
        )

        # Publisher for cmd_vel to control the robot's movement
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.rot_end_pub = rospy.Publisher("/one_rot/end_status", Bool, queue_size=10)
        # self.reach_goal_pub = rospy.Publisher('/move_status', Bool, queue_size=10)

        # Subscribe to the tf topic to get the robot's pose
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_callback)
        self.listener = tf.TransformListener()

        rospy.loginfo("Navigation Success Checker initialized.")

    def goal_callback(self, msg):
        try:
            x = msg.pose.position.x
            y = msg.pose.position.y
            z = msg.pose.position.z
            self.goal = [x, y, z]
            rospy.logdebug(f"Goal received: {x:.2f}, {y:.2f}, {z:.2f}")
        except AttributeError as e:
            rospy.logwarn(f"Malformed goal message received: {e}")

    def check_navigation_success(self):
        # Continuously check for the robot's position and orientation
        try:
            # Get the current pose of the robot from tf
            (trans, rot) = self.listener.lookupTransform(
                "map", "base_link", rospy.Time(0)
            )

            # Do not rotate 360 in an L-shaped corridor
            if (trans[0] <= 23.5 and trans[1] >= -1.5) or (
                trans[0] >= 20.5 and trans[1] <= -1.5
            ):
                # self.reach_goal_pub.publish(Bool(data=False))
                rospy.logdebug("In an L-shaped corridor.")

            else:
                if self.goal:
                    dist_ego_goal = math.sqrt(
                        (trans[0] - self.goal[0]) ** 2 + (trans[1] - self.goal[1]) ** 2
                    )
                    if (
                        self.previous_position is not None
                        and self.previous_orientation is not None
                    ):
                        # Calculate the Euclidean distance between the current and previous position
                        position_diff = math.sqrt(
                            (trans[0] - self.previous_position[0]) ** 2
                            + (trans[1] - self.previous_position[1]) ** 2
                        )
                        # Convert quaternion to euler angles to calculate the orientation difference
                        current_euler = euler_from_quaternion(rot)
                        previous_euler = euler_from_quaternion(
                            self.previous_orientation
                        )
                        orientation_diff = abs(
                            current_euler[2] - previous_euler[2]
                        )  # Only check the Z-axis orientation
                        if dist_ego_goal < self.min_ego_goal_dist:
                            self.cancel_navigation_goal()
                            self.rotate_360()
                            self.similarity_count = 0
                            self.unsimilarity_count = 0
                        # Check if the position and orientation differences are below the thresholds
                        if (
                            position_diff < self.position_threshold
                            and orientation_diff < self.orientation_threshold
                            
                        ):
                            self.similarity_count += 1
                            self.unsimilarity_count = 0
                        else:
                            # Reset the similarity count if the movement is not similar enough
                            self.similarity_count = 0
                            self.unsimilarity_count += 1
                            # Target not reached, ensure flag is False

                        # If the required number of similar frames is met, declare navigation success
                        rospy.logdebug(f"Similarity count: {self.similarity_count}")
                        if self.similarity_count >= self.max_similarity_count:
                            rospy.loginfo("Navigation Success: Goal reached!")
                            # self.reach_goal_pub.publish(Bool(data=True))
                            self.cancel_navigation_goal()
                            # Start rotating 360 degrees after goal is reached
                            self.rotate_360()
                            # Reset the counter after success
                            self.similarity_count = 0
                            self.unsimilarity_count = 0
                        elif (
                            (self.unsimilarity_count >= self.max_unsimilarity_count)
                            and (dist_ego_goal <= self.min_ego_goal_dist)
                        ) or (
                            self.unsimilarity_count >= self.max_unsimilarity_count * 1.2
                        ):
                            # self.reach_goal_pub.publish(Bool(data=False))
                            rospy.loginfo(
                                "Navigation Unsuccess: Failed to reach the target point, abandoned the target!"
                            )
                            self.cancel_navigation_goal()
                            self.rotate_360()
                            self.similarity_count = 0
                            self.unsimilarity_count = 0

            # Update the previous position and orientation
            self.previous_position = trans
            self.previous_orientation = rot

            # Always publish the current rot_end_status
            self.rot_end_pub.publish(Bool(data=self.rot_end_status))
            self.rot_end_status = False  # Reset to False after one-time True publish
            # self.reach_goal_pub.publish(Bool(data=False))

        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ):
            rospy.logwarn("TF exception, unable to get the current pose.")

    def rotate_360(self):
        rospy.loginfo("Starting 360-degree rotation...")

        rotate_cmd = Twist()
        rotate_cmd.angular.z = 0.7  # 0.5  # Positive means counter-clockwise rotation

        self.rot_end_status = False  # Reset before rotation
        self.rot_end_pub.publish(Bool(data=False))  # Actively publish reset
        # self.reach_goal_pub.publish(Bool(data=False))

        # Get initial yaw angle
        (_, rot) = self.listener.lookupTransform("map", "base_link", rospy.Time(0))
        last_yaw = euler_from_quaternion(rot)[2]

        total_rotation = 0.0

        rate = rospy.Rate(20)  # Higher rate for smoother calculation
        while not rospy.is_shutdown():
            try:
                (_, rot) = self.listener.lookupTransform(
                    "map", "base_link", rospy.Time(0)
                )
                current_yaw = euler_from_quaternion(rot)[2]

                # Calculate delta yaw and normalize to [-pi, pi]
                delta_yaw = current_yaw - last_yaw
                if delta_yaw > math.pi:
                    delta_yaw -= 2 * math.pi
                elif delta_yaw < -math.pi:
                    delta_yaw += 2 * math.pi

                total_rotation += abs(delta_yaw)
                last_yaw = current_yaw

                rospy.logdebug(
                    f"Accumulated rotation: {math.degrees(total_rotation):.2f} degrees"
                )

                if total_rotation >= 2 * math.pi - 0.1:  # Allow some margin (≈6.18 rad)
                    rospy.loginfo("Rotation complete.")
                    self.cmd_vel_pub.publish(Twist())  # Stop the robot
                    # Set rotation end status to True and publish it
                    self.rot_end_status = True
                    self.rot_end_pub.publish(Bool(data=True))
                    break
                else:
                    self.cmd_vel_pub.publish(rotate_cmd)

                rate.sleep()
                # self.reach_goal_pub.publish(Bool(data=False))

            except (
                tf.LookupException,
                tf.ConnectivityException,
                tf.ExtrapolationException,
            ):
                rospy.logwarn("TF exception during rotation.")
                self.cmd_vel_pub.publish(rotate_cmd)
                rate.sleep()

    def cancel_navigation_goal(self):
        # Create a SimpleActionClient and connect to the ActionServer of move_base
        client = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        # Waiting for connection
        # rospy.loginfo("Waiting for move_base action server to start...")
        client.wait_for_server()

        # Publish a request to cancel all navigation target
        rospy.loginfo("Canceling current navigation goal for rotation.")
        client.cancel_all_goals()


if __name__ == "__main__":
    try:
        # Create the NavigationSuccessChecker object
        checker = NavigationSuccessChecker()

        # Run the check_navigation_success function at a fixed rate
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            checker.check_navigation_success()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass