#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <gazebo_msgs/ModelStates.h>
#include <fstream>
#include <cmath>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <ros/package.h>


// std::ofstream logfile;

geometry_msgs::Pose goal_pose;
geometry_msgs::Pose current_pose;
bool goal_received = false;
bool goal_evaluated = false;
std::string robot_name = "jackal";  // 根据你的模型名称调整

double computePositionError(const geometry_msgs::Pose& a, const geometry_msgs::Pose& b)
{
    return std::hypot(a.position.x - b.position.x, a.position.y - b.position.y);
}

double computeYaw(const geometry_msgs::Pose& p)
{
    tf2::Quaternion q(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    return yaw;
}

void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    goal_pose = msg->pose;
    goal_received = true;
    goal_evaluated = false;
    ROS_INFO("Received new goal.");
}

void gtCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    if (!goal_received || goal_evaluated) return;

    int idx = -1;
    for (int i = 0; i < msg->name.size(); ++i)
    {
        if (msg->name[i] == robot_name)
        {
            idx = i;
            break;
        }
    }
    if (idx == -1) return;

    current_pose = msg->pose[idx];
    double pos_err = computePositionError(goal_pose, current_pose);

    if (pos_err < 0.2)  // 误差阈值
    {
        double yaw_goal = computeYaw(goal_pose);
        double yaw_actual = computeYaw(current_pose);
        double heading_err = std::abs(yaw_goal - yaw_actual) * 180 / M_PI;

        std::string pkg_path = ros::package::getPath("me5413_evaluation"); // replace with your package name
        std::string log_path = pkg_path + "/logs/final_goal_error.txt";

        std::ofstream out(log_path);
        if (!out.is_open()) {
            ROS_ERROR("Failed to open log file: %s", log_path.c_str());
            return;
        }

        // std::ofstream out(log_path);
        out << "目标位置: " << goal_pose.position.x << ", " << goal_pose.position.y << "\n";
        out << "实际位置: " << current_pose.position.x << ", " << current_pose.position.y << "\n";
        out << "位置误差: " << pos_err << " m\n";
        out << "朝向误差: " << heading_err << " deg\n";
        out.close();

        ROS_INFO("导航完成，误差保存成功！");
        goal_evaluated = true;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "goal_vs_final_evaluator_node");
    ros::NodeHandle nh;

    ros::Subscriber sub_goal = nh.subscribe("/move_base_simple/goal", 10, goalCallback);
    ros::Subscriber sub_gt = nh.subscribe("/gazebo/ground_truth/state", 10, gtCallback);

    ros::spin();
    return 0;
}

