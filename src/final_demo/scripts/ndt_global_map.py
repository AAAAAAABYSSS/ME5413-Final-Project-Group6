#!/usr/bin/env python3
#coding=utf-8

import rospy
import open3d as o3d
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from hdl_global_localization.srv import SetGlobalMap, SetGlobalMapRequest

def load_pcd_and_set_map(pcd_file):
    rospy.init_node("set_global_map_client")

    rospy.loginfo("Enter!")
    # Wait for the service to be available
    rospy.wait_for_service("/hdl_global_localization/set_global_map")

    try:
        # Load PCD file using PCL
        pcd = o3d.io.read_point_cloud(pcd_file)  
        points = [(point[0], point[1], point[2]) for point in pcd.points]  # 只取XYZ
        

        # Convert to PointCloud2
        header = rospy.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "world"  # Ensure this matches your robot’s TF tree

        cloud_msg = pc2.create_cloud_xyz32(header, points)

        # Create service request
        req = SetGlobalMapRequest()
        req.global_map = cloud_msg

        # Call the service
        set_map_service = rospy.ServiceProxy("/hdl_global_localization/set_global_map", SetGlobalMap)
        response = set_map_service(req)

        if response:
            rospy.loginfo("Successfully updated the global map.")
        else:
            rospy.logwarn("Failed to update the global map.")

    except Exception as e:
        rospy.logerr("Service call failed: %s" % str(e))

if __name__ == "__main__":
    pcd_file_path = "/home/jy/data/all_raw_points.pcd"  # Change this
    load_pcd_and_set_map(pcd_file_path)