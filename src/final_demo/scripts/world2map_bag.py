#!/usr/bin/env python3
# -*- coding:utf-8 -*-  

import rospy
import tf 
from rosgraph_msgs.msg import Clock


class MapOdomPublisher:
    def __init__(self):
        rospy.init_node('map_to_world_tf', anonymous=True)
        rospy.loginfo("Begin!")
        self.br =  tf.TransformBroadcaster()

    def clock_callback(self, msg):
        """ 更新当前时间，使 TF 时间戳与 rosbag 对齐 """
        current_time = msg.clock
        # rospy.loginfo(f"clock:{current_time}")
        
        x=1.0 
        y=1.0
        z=0.0  
        roll=0 
        pitch=0
        yaw=0

        self.br.sendTransform((x,y,z),  
                    tf.transformations.quaternion_from_euler(roll,pitch,yaw),  
                    current_time,  
                    "odom",  
                    "map")
    
    def clock_up_tf(self):
        rospy.loginfo("Listening!")
        rospy.Subscriber("/clock", Clock, self.clock_callback)  # 订阅 /clock 主题
        rospy.spin()

if __name__ == "__main__":
    try:
        node = MapOdomPublisher()
        node.clock_up_tf()
    except rospy.ROSInterruptException:
        pass

