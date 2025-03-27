cmake_minimum_required(VERSION 3.0.2)
project(me5413_perception)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/clu_pcd_v0.5.py
  scripts/pseudo_status.py
  scripts/pub_3d_bbox.py
  scripts/pub_nav_goal_v0.1.py
  scripts/yolo_detect.py
  scripts/fusion_node.py
  scripts/fusion_merged_node.py
  scripts/visualize_fusion.py

  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
