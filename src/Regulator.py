#!/usr/bin/env python

import numpy as np
import rospy
from nav_msgs.msg import Odometry
import sys
from projectTools import DroneCommand
from std_msgs.msg import Float32,Int32

class RegulatorClass(object):
    def __init__(self,P,I,D):
        self.odom_subscriber = rospy.Subscriber("/curr_data",Odometry,self.read_val)
        self.vel_subscriber = rospy.Subscriber("/tar_in",Float32,self.read_tar)
        self.act_subscriber = rospy.Subscriber("/activation",Int32,self.check_activation)
        self.cmd_reset = rospy.Subscriber("/reset_cmd",Int32,self.reset_cmd)
        self.dc = DroneCommand(P,I,D)
        self.cmd_publisher = rospy.Publisher("/cmd_out",Float32, queue_size=1)
        self.currVal = 0
        self.targVal = 0
        self.activation = 0
    def read_tar(self,ros_data):
        if(self.activation == 0):
            return 
        self.targVal = ros_data.data
        self.cmd = self.dc.computeCommand(self.currVal,self.targVal)
        self.cmd_publisher.publish(self.cmd)
    def check_activation(self,ros_data):
        self.activation = ros_data.data
        if self.activation == 0:
            self.cmd_publisher.publish(0.0)
    def reset_cmd(self,ros_data):
        self.dc.cmd = 0
        self.dc.TotErr = 0
        self.dc.oldErr = 0