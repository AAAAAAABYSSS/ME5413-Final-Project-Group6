/* goal_publisher_node.cpp

 * Copyright (C) 2023 SS47816

 * ROS Node for publishing goal poses 
 
**/

#include "me5413_world/goal_publisher_node.hpp"

namespace me5413_world 
{

GoalPublisherNode::GoalPublisherNode() : tf2_listener_(tf2_buffer_)
{
  this->pub_goal_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
  this->pub_absolute_position_error_ = nh_.advertise<std_msgs::Float32>("/me5413_world/absolute/position_error", 1);
  this->pub_absolute_heading_error_ = nh_.advertise<std_msgs::Float32>("/me5413_world/absolute/heading_error", 1);
  this->pub_relative_position_error_ = nh_.advertise<std_msgs::Float32>("/me5413_world/relative/position_error", 1);
  this->pub_relative_heading_error_ = nh_.advertise<std_msgs::Float32>("/me5413_world/relative/heading_error", 1);

  this->timer_ = nh_.createTimer(ros::Duration(0.2), &GoalPublisherNode::timerCallback, this);
  this->sub_robot_odom_ = nh_.subscribe("/gazebo/ground_truth/state", 1, &GoalPublisherNode::robotOdomCallback, this);
  this->sub_goal_name_ = nh_.subscribe("/rviz_panel/goal_name", 1, &GoalPublisherNode::goalNameCallback, this);
  this->sub_goal_pose_ = nh_.subscribe("/move_base_simple/goal", 1, &GoalPublisherNode::goalPoseCallback, this);
  this->sub_box_markers_ = nh_.subscribe("/gazebo/ground_truth/box_markers", 1, &GoalPublisherNode::boxMarkersCallback, this);
  
  // Initialization
  this->robot_frame_ = "base_link";
  this->map_frame_ = "map";
  this->world_frame_ = "world";
  this->absolute_position_error_.data = 0.0;
  this->absolute_heading_error_.data = 0.0;
  this->relative_position_error_.data = 0.0;
  this->relative_heading_error_.data = 0.0;
};

void GoalPublisherNode::timerCallback(const ros::TimerEvent&)
{
  // Calculate absolute errors (wrt to world frame)
  const std::pair<double, double> error_absolute = calculatePoseError(this->pose_world_robot_, this->pose_world_goal_);
  // Calculate relative errors (wrt to map frame)
  const std::pair<double, double> error_relative = calculatePoseError(this->pose_map_robot_, this->pose_map_goal_);
  
  this->absolute_position_error_.data = error_absolute.first;
  this->absolute_heading_error_.data = error_absolute.second;
  this->relative_position_error_.data = error_relative.first;
  this->relative_heading_error_.data = error_relative.second;

  if (this->goal_type_ == "box")
  {
    this->absolute_heading_error_.data = 0.0;
    this->relative_heading_error_.data = 0.0;
  }

  // Publish errors
  this->pub_absolute_position_error_.publish(this->absolute_position_error_);
  this->pub_absolute_heading_error_.publish(this->absolute_heading_error_);
  this->pub_relative_position_error_.publish(this->relative_position_error_);
  this->pub_relative_heading_error_.publish(this->relative_heading_error_);

  return;
};

void GoalPublisherNode::robotOdomCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
  this->world_frame_ = odom->header.frame_id;
  this->robot_frame_ = odom->child_frame_id;
  this->pose_world_robot_ = odom->pose.pose;

  // // Step 1: world → base_link（来自 Gazebo ground truth）
  // tf2::Transform T_world_base = convertPoseToTransform(this->pose_world_robot_);

  // // Step 2: 创建一个绕 Z 轴逆时针旋转 90° 的变换
  // tf2::Transform T_world_rotation;
  // T_world_rotation.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
  // tf2::Quaternion q_rotation;
  // q_rotation.setRPY(0, 0, - M_PI / 2);  // 逆时针旋转 90°
  // q_rotation.normalize();
  // T_world_rotation.setRotation(q_rotation);

  // // Step 3: 旋转修正后的 world → base_link
  // tf2::Transform T_world_base_rotated = T_world_rotation * T_world_base;

  // // Step 4: 获取 odom → base_link（来自 EKF）
  // geometry_msgs::TransformStamped tf_odom_base_msg;
  // tf2::Transform T_odom_base;
  // try
  // {
  //   tf_odom_base_msg = this->tf2_buffer_.lookupTransform("odom", this->robot_frame_, ros::Time(0));
  //   tf2::fromMsg(tf_odom_base_msg.transform, T_odom_base);
  // }
  // catch (tf2::TransformException &ex)
  // {
  //   ROS_WARN("Failed to lookup odom → base_link: %s", ex.what());
  //   return;
  // }

  // // Step 5: base_link → odom
  // tf2::Transform T_base_odom = T_odom_base.inverse();

  // // Step 6: 推出 world → odom
  // tf2::Transform T_world_odom = T_world_base_rotated * T_base_odom;

  // // Step 7: 发布 world → odom
  // geometry_msgs::TransformStamped transformStamped;
  // transformStamped.header.stamp = ros::Time::now();
  // transformStamped.header.frame_id = "world";
  // transformStamped.child_frame_id = "odom";
  // transformStamped.transform.translation.x = T_world_odom.getOrigin().getX();
  // transformStamped.transform.translation.y = T_world_odom.getOrigin().getY();
  // transformStamped.transform.translation.z = T_world_odom.getOrigin().getZ();
  // transformStamped.transform.rotation.x = T_world_odom.getRotation().getX();
  // transformStamped.transform.rotation.y = T_world_odom.getRotation().getY();
  // transformStamped.transform.rotation.z = T_world_odom.getRotation().getZ();
  // transformStamped.transform.rotation.w = T_world_odom.getRotation().getW();

  // this->tf2_bcaster_.sendTransform(transformStamped);
  const tf2::Transform T_world_robot = convertPoseToTransform(this->pose_world_robot_);
  const tf2::Transform T_robot_world = T_world_robot.inverse();

  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = this->robot_frame_;
  transformStamped.child_frame_id = this->world_frame_;
  
  transformStamped.transform.translation.x = T_robot_world.getOrigin().getX();
  transformStamped.transform.translation.y = T_robot_world.getOrigin().getY();
  transformStamped.transform.translation.z = 0.0;
  transformStamped.transform.rotation.x = T_robot_world.getRotation().getX();
  transformStamped.transform.rotation.y = T_robot_world.getRotation().getY();
  transformStamped.transform.rotation.z = T_robot_world.getRotation().getZ();
  transformStamped.transform.rotation.w = T_robot_world.getRotation().getW();
  
  this->tf2_bcaster_.sendTransform(transformStamped);

  return;
};

void GoalPublisherNode::goalNameCallback(const std_msgs::String::ConstPtr& name)
{ 
  const std::string goal_name = name->data;
  const int end = goal_name.find_last_of("_");
  this->goal_type_ = goal_name.substr(1, end-1);
  const int goal_box_id = stoi(goal_name.substr(end+1, 1));

  geometry_msgs::PoseStamped P_world_goal;
  if (this->goal_type_ == "box")
  {
    if (box_poses_.empty())
    {
      ROS_ERROR_STREAM("Box poses unknown, please spawn boxes first!");
      return;
    }
    else if (goal_box_id >= box_poses_.size())
    {
      ROS_ERROR_STREAM("Box id is outside the available range, please select a smaller id!");
      return;
    }
    
    P_world_goal = box_poses_[goal_box_id - 1];
  }
  else
  {
    // Get the Pose of the goal in world frame
    P_world_goal = getGoalPoseFromConfig(goal_name);
  }

  this->pose_world_goal_ = P_world_goal.pose;
  // Get the Transform from world to map from the tf_listener
  geometry_msgs::TransformStamped transform_map_world;
  try
  {
    transform_map_world = this->tf2_buffer_.lookupTransform(this->map_frame_, this->world_frame_, ros::Time(0));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
    return;
  }

  // Transform the goal pose to map frame
  geometry_msgs::PoseStamped P_map_goal;
  tf2::doTransform(P_world_goal, P_map_goal, transform_map_world);
  P_map_goal.header.stamp = ros::Time::now();
  P_map_goal.header.frame_id = map_frame_;

  // Transform the robot pose to map frame
  tf2::doTransform(this->pose_world_robot_, this->pose_map_robot_, transform_map_world);

  // Publish goal pose in map frame 
  if (this->goal_type_ != "box")
  {
    this->pub_goal_.publish(P_map_goal);
  }

  return;
};

void GoalPublisherNode::goalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& goal_pose)
{
  this->pose_map_goal_ = goal_pose->pose;
}

tf2::Transform GoalPublisherNode::convertPoseToTransform(const geometry_msgs::Pose& pose)
{
  tf2::Transform T;
  T.setOrigin(tf2::Vector3(pose.position.x, pose.position.y, 0));
  tf2::Quaternion q;
  q.setValue(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  T.setRotation(q);

  return T;
};

void GoalPublisherNode::boxMarkersCallback(const visualization_msgs::MarkerArray::ConstPtr& box_markers)
{
  this->box_poses_.clear();
  for (const auto& box : box_markers->markers)
  {
    geometry_msgs::PoseStamped pose;
    pose.pose = box.pose;
    this->box_poses_.emplace_back(pose);
  }

  return;
};

geometry_msgs::PoseStamped GoalPublisherNode::getGoalPoseFromConfig(const std::string& name)
{
  /** 
   * Get the Transform from goal to world from the file
   */

  double x, y, yaw;
  nh_.getParam("/me5413_world" + name + "/x", x);
  nh_.getParam("/me5413_world" + name + "/y", y);
  nh_.getParam("/me5413_world" + name + "/yaw", yaw);
  nh_.getParam("/me5413_world/frame_id", this->world_frame_);

  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  q.normalize();

  geometry_msgs::PoseStamped P_world_goal;
  P_world_goal.pose.position.x = x;
  P_world_goal.pose.position.y = y;
  P_world_goal.pose.orientation = tf2::toMsg(q);

  return P_world_goal;
};

std::pair<double, double> GoalPublisherNode::calculatePoseError(const geometry_msgs::Pose& pose_robot, const geometry_msgs::Pose& pose_goal)
{
  // Positional Error
  const double position_error = std::sqrt(
    std::pow(pose_robot.position.x - pose_goal.position.x, 2) + 
    std::pow(pose_robot.position.y - pose_goal.position.y, 2)
  );

  // Heading Error
  tf2::Quaternion q_robot, q_goal;
  tf2::fromMsg(pose_robot.orientation, q_robot);
  tf2::fromMsg(pose_goal.orientation, q_goal);
  const tf2::Matrix3x3 m_robot = tf2::Matrix3x3(q_robot);
  const tf2::Matrix3x3 m_goal = tf2::Matrix3x3(q_goal);

  double roll, pitch, yaw_robot, yaw_goal;
  m_robot.getRPY(roll, pitch, yaw_robot);
  m_goal.getRPY(roll, pitch, yaw_goal);

  const double heading_error = (yaw_robot - yaw_goal)/M_PI*180.0;

  return std::pair<double, double>(position_error, heading_error);
}

} // namespace me5413_world

int main(int argc, char** argv)
{
  ros::init(argc, argv, "goal_publisher_node");
  me5413_world::GoalPublisherNode goal_publisher_node;
  ros::spin();  // spin the ros node.
  return 0;
}