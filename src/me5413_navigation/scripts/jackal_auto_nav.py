#!/usr/bin/env python
import rospy
import tf
import math
import actionlib
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult
from std_msgs.msg import Bool, Float32MultiArray
from actionlib_msgs.msg import GoalStatus

class JackalAutoRotNav:
    def __init__(self):
        rospy.init_node('jackal_auto_rot_nav')

        self.pause_time = rospy.get_param('~pause_duration', 1.0)
        self.goal_sent = False
        self.awaiting_rotation = False

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=20)
        self.finish_pub = rospy.Publisher('/one_rot/finish_status', Bool, queue_size=5)
        self.end_pub = rospy.Publisher('/one_rot/end_status', Bool, queue_size=5)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=5)

        rospy.Subscriber('/one_rot/unfinished_boxes', Float32MultiArray, self.unfinished_callback)
        rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.nav_result_callback)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)

        self.listener = tf.TransformListener()
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("📦 等待 move_base server...")
        self.move_base_client.wait_for_server()
        rospy.loginfo("✅ move_base 已连接")

        rospy.sleep(2.0)  # 延时等待 TF 和初始定位稳定
        self.send_initial_goal()

    def send_initial_goal(self):
        goal = PoseStamped()
        goal.header.frame_id = "map"
        while rospy.Time.now().to_sec() == 0:
            rospy.loginfo("⏳ 等待仿真时间启动...")
            rospy.sleep(0.1)
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = 21.16
        goal.pose.position.y = -22.14
        goal.pose.orientation.z = 0.98
        goal.pose.orientation.w = 0.20

        rospy.loginfo("🚀 发布初始导航目标到 /move_base_simple/goal")
        self.goal_pub.publish(goal)

    def goal_callback(self, msg):
        rospy.loginfo("🎯 接收到目标点（来自 /move_base_simple/goal）")
        goal = MoveBaseGoal()
        goal.target_pose.header = msg.header
        goal.target_pose.pose = msg.pose
        self.move_base_client.send_goal(goal)
        self.goal_sent = True

    def nav_result_callback(self, msg):
        rospy.loginfo("📬 move_base 结果: 状态 %s", msg.status.status)
        if msg.status.status == GoalStatus.SUCCEEDED:
            rospy.loginfo("✅ 导航完成，开始旋转")
            self.goal_sent = False
            self.rotate_full_circle()

    def rotate_full_circle(self):
        rospy.loginfo("🔁 开始原地旋转一圈")
        twist = Twist()
        twist.angular.z = 0.5
        rotate_time = 2 * math.pi / abs(twist.angular.z)
        rate = rospy.Rate(10)
        start_time = rospy.Time.now()
        while rospy.Time.now() - start_time < rospy.Duration(rotate_time):
            self.cmd_vel_pub.publish(twist)
            rate.sleep()
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(0.5)
        rospy.loginfo("✅ 原地旋转完成，发布 /one_rot/finish_status")
        self.finish_pub.publish(Bool(data=True))
        self.awaiting_rotation = True

    def unfinished_callback(self, msg):
        if not self.awaiting_rotation:
            return
        rospy.loginfo("📐 补旋角度：%s", str(msg.data))
        for angle in msg.data:
            self.rotate_to_angle(angle)
            rospy.sleep(self.pause_time)
        rospy.loginfo("✅ 补角完成，发布 /one_rot/end_status")
        self.end_pub.publish(Bool(data=True))
        self.awaiting_rotation = False

    def rotate_to_angle(self, yaw_target_deg):
        target_yaw_rad = math.radians(yaw_target_deg)
        tolerance = 0.05
        rate = rospy.Rate(10)
        twist = Twist()

        while not rospy.is_shutdown():
            try:
                self.listener.waitForTransform("/map", "/base_link", rospy.Time(0), rospy.Duration(1.0))
                (_, rot) = self.listener.lookupTransform('/map', '/base_link', rospy.Time(0))
                _, _, current_yaw = euler_from_quaternion(rot)
                yaw_error = self.normalize_angle(target_yaw_rad - current_yaw)
                if abs(yaw_error) < tolerance:
                    break
                twist.angular.z = 0.4 if yaw_error > 0 else -0.4
                self.cmd_vel_pub.publish(twist)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logwarn("⚠️ TF 错误：%s", str(e))
                break
            rate.sleep()
        self.cmd_vel_pub.publish(Twist())

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

if __name__ == '__main__':
    try:
        JackalAutoRotNav()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass