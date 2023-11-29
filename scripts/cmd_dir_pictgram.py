#! /usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import rospy
import rosnode
from std_msgs.msg import String
from std_msgs.msg import Float32, Float32MultiArray, Int8
import time
import psutil
from jsk_rviz_plugins.msg import Pictogram
from scenario_navigation_msgs.msg import cmd_dir_intersection

from system_monitor_ROS.msg import system_monitor

class Pict_pub():
    def __init__(self):
        self.cmd_dir_sub = rospy.Subscriber("/cmd_dir_intersection", cmd_dir_intersection, self.callback_cmd,queue_size=1)
        self.pict_pub = rospy.Publisher("/pict",Pictogram,queue_size=1)
        self.cmd_dir_data = (1,0,0)
    def callback_cmd(self, data):
        self.cmd_dir_data = data.cmd_dir

    def pictogram_pub(self):
        pict_data = Pictogram()
        pict_data.header.frame_id ="base_link"
        # pict_data.header.stamp =rospy.get_time()
        pict_data.pose.position.z = 0.4
        pict_data.pose.orientation.y = -0.71
        pict_data.pose.orientation.w =-0.71
        pict_data.action = Pictogram.ADD
        pict_data.color.r = 1.0
        pict_data.color.a = 1.0
        pict_data.mode = Pictogram.PICTOGRAM_MODE
        pict_data.size = 0.5
        self.cmd_dir_data = (1,0,0)
        # if self.cmd_dir_data == (1,0,0):
        pict_data.character = "up"

        # if self.cmd_dir_data == (0,1,0):
        #     pict_data.character = "left"
        
        # if self.cmd_dir_data == (0,0,1):
        #     pict_data.character = "right"
        
        # if self.cmd_dir_data == (0,0,0):
        #     pict_data.character = "stop"
        self.pict_pub.publish(pict_data)
if __name__ == '__main__':
    rospy.init_node('cmd_dir_pictogram_node')

    pict_pub_node = Pict_pub()
    
    while not rospy.is_shutdown():
        # rospy.spin()
        print("hoge")
        pict_pub_node.pictogram_pub()
        rospy.sleep(1.0)