#!/usr/bin/env python3
# -*- coding:utf-8 -*- 

import rospy
import tf2_ros
import geometry_msgs.msg

def publish_odom_to_map():
    rospy.init_node('odom_to_map_broadcaster', anonymous=True)
    
    # 创建 TF 变换广播器
    br = tf2_ros.TransformBroadcaster()
    
    rate = rospy.Rate(10)  # 10Hz 频率发布 TF
    
    while not rospy.is_shutdown():
        # 创建 TF 变换消息
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "map"
        t.child_frame_id = "odom"
        
        # 设定 map -> odom 变换，使 map 固定在 (0,0,0)
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        
        # 旋转角度 (四元数表示)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0  # 无旋转

        # 发送 TF 变换
        br.sendTransform(t)

        rate.sleep()

if __name__ == "__main__":
    try:
        publish_odom_to_map()
    except rospy.ROSInterruptException:
        pass