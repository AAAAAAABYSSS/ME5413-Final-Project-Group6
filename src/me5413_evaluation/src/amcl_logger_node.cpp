#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <fstream>
#include <ros/package.h>

std::ofstream logfile;

void callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
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
    ros::init(argc, argv, "amcl_logger_node");
    ros::NodeHandle nh;

    // logfile.open("/home/lzr/amcl.txt");

    std::string pkg_path = ros::package::getPath("me5413_evaluation");
    std::string log_path = pkg_path + "/Log/amcl.txt";

    logfile.open(log_path);
    if (!logfile.is_open()) {
        ROS_ERROR("Failed to open log file: %s", log_path.c_str());
        return 1;
    }

    ros::Subscriber sub = nh.subscribe("/amcl_pose", 10, callback);

    ros::spin();
    logfile.close();
    return 0;
}

