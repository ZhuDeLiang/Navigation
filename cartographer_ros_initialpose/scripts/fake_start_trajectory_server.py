#!/usr/bin/env python
#coding:utf-8
from __future__ import print_function
import rospy
from cartographer_ros_msgs.srv import *

def server_node():
    rospy.init_node("fake_star_trajectory_server")
    s=rospy.Service("start_trajectory",StartTrajectory,callback_function)
    rospy.loginfo("Fake server starts")
    rospy.spin()
    return

def callback_function(req):
    print("request received.")
    print(req)
    return StartTrajectoryResponse()

if __name__ == '__main__':
    server_node()