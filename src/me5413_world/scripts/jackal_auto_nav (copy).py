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

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.finish_pub = rospy.Publisher('/one_rot/finish_status', Bool, queue_size=1)
        self.end_pub = rospy.Publisher('/one_rot/end_status', Bool, queue_size=1)

        rospy.Subscriber('/one_rot/unfinished_boxes', Float32MultiArray, self.unfinished_callback)
        rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.nav_result_callback)

        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("📦 等待 move_base server...")
        self.move_base_client.wait_for_server()
        rospy.loginfo("✅ move_base 已连接")
        rospy.sleep(1.0)
        self.send_initial_goal()

    def send_initial_goal(self):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        
        while rospy.Time.now().to_sec() == 0:
            rospy.loginfo("⏳ Waiting for simulated time to start...")
            rospy.sleep(0.1)

        # ✅ 修复：使用当前仿真时间，或 rospy.Time(0)
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = 21.16
        goal.target_pose.pose.position.y = -22.14
        goal.target_pose.pose.orientation.z = 0.98
        goal.target_pose.pose.orientation.w = 0.20

        rospy.loginfo("🚀 发送导航目标")
        self.move_base_client.send_goal(goal)
        rospy.loginfo("🛰️ 已发出导航目标，等待反馈...")

        rate = rospy.Rate(1)
        timeout = rospy.Time.now() + rospy.Duration(60)  # 最多等 60 秒
        while rospy.Time.now() < timeout and not rospy.is_shutdown():
            state = self.move_base_client.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("✅ move_base 成功导航到目标点")
                self.rotate_full_circle()
                return
            elif state == GoalStatus.ABORTED:
                rospy.logwarn("❌ move_base 导航失败（路径规划失败？）")
                return
            elif state == GoalStatus.ACTIVE:
                rospy.loginfo("🚕 正在导航中...")
            else:
                rospy.loginfo("🟡 当前状态: %s", state)
            rate.sleep()

            rospy.logwarn("⚠️ move_base 超时未完成导航")
            # 添加 30 秒内等待结果
            if self.move_base_client.wait_for_result(rospy.Duration(800)):
                result = self.move_base_client.get_result()
                rospy.loginfo("✅ move_base 执行完毕，结果: %s", result)
            else:
                rospy.logwarn("⚠️ move_base 在 30 秒内未返回结果，目标可能未被接受")
            rate = rospy.Rate(1)
            while not rospy.is_shutdown():
                state = self.move_base_client.get_state()
                if state == GoalStatus.ACTIVE:
                    rospy.loginfo("🚕 正在导航中...")
                elif state == GoalStatus.SUCCEEDED:
                    rospy.loginfo("✅ 导航完成，开始旋转")
                    self.goal_sent = False
                    self.rotate_full_circle()
                    break
                elif state in [GoalStatus.ABORTED, GoalStatus.REJECTED]:
                    rospy.logwarn("❌ 导航失败或被拒绝")
                    break
                rate.sleep()

    def nav_result_callback(self, msg):
        rospy.loginfo("📬 move_base 结果: 状态 %s", msg.status.status)
        if msg.status.status == 3:  # SUCCEEDED
            self.goal_sent = False
            self.rotate_full_circle()

    def rotate_full_circle(self):
        rospy.loginfo("🔁 开始原地旋转一圈")
        twist = Twist()
        twist.angular.z = 0.5
        rotate_time = 2 * math.pi / abs(twist.angular.z)
        t_end = rospy.Time.now() + rospy.Duration(rotate_time)
        rate = rospy.Rate(10)
        while rospy.Time.now() < t_end and not rospy.is_shutdown():
            self.cmd_vel_pub.publish(twist)
            rate.sleep()
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        rospy.sleep(0.5)
        rospy.loginfo("✅ 原地旋转完成，发布 /one_rot/finish_status")
        self.finish_pub.publish(Bool(data=True))
        self.awaiting_rotation = True

    def unfinished_callback(self, msg):
        if not self.awaiting_rotation:
            return
        rospy.loginfo("📐 未完成角度：%s", str(msg.data))
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
        listener = tf.TransformListener()
        while not rospy.is_shutdown():
            try:
                (_, rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
                _, _, current_yaw = euler_from_quaternion(rot)
                yaw_error = self.normalize_angle(target_yaw_rad - current_yaw)
                if abs(yaw_error) < tolerance:
                    break
                twist.angular.z = 0.4 if yaw_error > 0 else -0.4
                self.cmd_vel_pub.publish(twist)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            rate.sleep()
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

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
