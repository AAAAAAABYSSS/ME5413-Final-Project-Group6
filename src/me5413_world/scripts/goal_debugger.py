#!/usr/bin/env python
import rospy
import actionlib
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatusArray

class SimpleGoalSender:
    def __init__(self):
        rospy.init_node('debug_move_base_goal')
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("⏳ 等待 move_base 服务器...")
        self.client.wait_for_server()
        rospy.loginfo("✅ 已连接 move_base")

        rospy.Subscriber('/move_base/status', GoalStatusArray, self.status_callback)

        self.send_goal()

    def send_goal(self):
        # 确保仿真时间已经开始
        while rospy.Time.now().to_sec() == 0:
            rospy.loginfo("🕒 等待仿真时间启动 /clock")
            rospy.sleep(0.1)

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = 4.0
        goal.target_pose.pose.position.y = 0.0
        goal.target_pose.pose.orientation.z = -0.13
        goal.target_pose.pose.orientation.w = 0.99

        rospy.loginfo("🚀 正在发送导航目标到 move_base")
        self.client.send_goal(goal)

        # 可选：等待结果（最多 60s）
        self.client.wait_for_result(rospy.Duration(60.0))
        result_state = self.client.get_state()
        rospy.loginfo("📬 move_base 最终状态码: %d", result_state)

    def status_callback(self, msg):
        if not msg.status_list:
            rospy.logwarn("⚠️ 当前无活动目标")
            return
        latest = msg.status_list[-1]
        rospy.loginfo("📡 move_base 状态: %d -> %s", latest.status, latest.text)

if __name__ == '__main__':
    try:
        SimpleGoalSender()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

