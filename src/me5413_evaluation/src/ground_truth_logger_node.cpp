#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <fstream>
#include <tf2/LinearMath/Quaternion.h>
#include <ros/package.h>
#include <nav_msgs/Odometry.h>


std::ofstream logfile;
// std::string robot_name = "jackal";  // 替换为你的机器人模型名

void callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // int index = -1;
    // for (size_t i = 0; i < msg->name.size(); ++i)
    // {
    //     if (msg->name[i] == robot_name)
    //     {
    //         index = i;
    //         break;
    //     }
    // }
    // if (index == -1) return;

    double t = msg->header.stamp.toSec();
    const auto& pose = msg->pose.pose;
    logfile << t << " "
            << pose.position.x << " "
            << pose.position.y << " "
            << pose.position.z << " "
            << pose.orientation.x << " "
            << pose.orientation.y << " "
            << pose.orientation.z << " "
            << pose.orientation.w << "\n";
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ground_truth_logger_node");
    ros::NodeHandle nh;

    std::string pkg_path = ros::package::getPath("me5413_evaluation");
    std::string log_path = pkg_path + "/Log/groundtruth.txt";

    logfile.open(log_path);
    if (!logfile.is_open()) {
        ROS_ERROR("Failed to open log file: %s", log_path.c_str());
        return 1;
    }

    ros::Subscriber sub = nh.subscribe("/gazebo/ground_truth/state", 10, callback);

    ros::spin();
    logfile.close();
    return 0;
}

