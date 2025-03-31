#!/usr/bin/env python3
#coding=utf-8
import rospy
import numpy as np
import open3d as o3d
from sensor_msgs.msg import PointCloud2
import random

axis_pcd = o3d.geometry.TriangleMesh.create_coordinate_frame(size=10, origin=[0, 0, 0])
vis = o3d.visualization.Visualizer()
vis.create_window()
vis.add_geometry(axis_pcd)


def out_text(name, input):
    return f"{name} is {input}"

def Callback(msg):
    global vis
    point_cloud = np.frombuffer(msg.data, dtype=np.uint8)
    height = msg.height
    width = msg.width
    point_step = msg.point_step
    row_step = msg.row_step
    is_b = msg.is_bigendian

    # rospy.loginfo("point cloud type is %s" % str(type(point_cloud)))
    # rospy.loginfo("point cloud shape is %s" % str(point_cloud.shape))
    # rospy.loginfo(out_text("point_cloud", point_cloud))
    # point_cloudm = point_cloud.reshape(point_step, width) # 22* xxx
    # point_cloudm = point_cloudm.T
    point_cloudm = point_cloud.reshape(width, point_step) # xxx * 22
    
    # print("-----------------")
    # for i in random.sample(list(range(len(point_cloudm))),10):
    #     point_cloud_line = point_cloudm[i]
    #     print(f"{i}: {point_cloud_line}")
    
    point_cloud_xyz = point_cloudm[:, :3]
    pcd2_transed = o3d.geometry.PointCloud()
    pcd2_transed.points = o3d.utility.Vector3dVector(point_cloud_xyz)
    pcd2_transed.paint_uniform_color([1, 0, 0]) # Red
    vis.add_geometry(pcd2_transed)
    vis.poll_events()
    vis.update_renderer()
    vis.remove_geometry(pcd2_transed)
    

def pc_subscriber():
	# ROS节点初始化
    rospy.init_node('point_cloud_subscriber', anonymous=True)

	
    rospy.Subscriber("/mid/points", PointCloud2, Callback)
	# 循环等待回调函数
    rospy.spin()

if __name__ == '__main__':

    pc_subscriber()
