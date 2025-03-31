#!/usr/bin/env python3
#coding=utf-8
import rospy

if __name__ == '__main__':
    rospy.init_node('hello', anonymous=True)
    rospy.loginfo("Hello World!")