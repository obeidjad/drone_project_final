#!/usr/bin/env python

import numpy as np
import rospy
from nav_msgs.msg import Odometry
import sys
from projectTools import DroneCommand
from std_msgs.msg import Float32,Int32

class RegulatorClass:
    def __init__(self,P,I,D):
        self.odom_subscriber = rospy.Subscriber("/odom_data",Odometry,self.read_val)
        self.vel_subscriber = rospy.Subscriber("/vel_in",Float32,self.read_tar)
        self.act_subscriber = rospy.Subscriber("/activation",Int32,self.check_activation)
        self.dc = DroneCommand(P,I,D)
        self.cmd_publisher = rospy.Publisher("/vel_out",Float32, queue_size=1)
        self.currVal = 0
        self.targVal = 0
        self.activation = 0
    def read_tar(self,ros_data):
        self.targVal = ros_data.data
    def check_activation(self,ros_data):
        self.activation = ros_data.data
        if self.activation == 0:
            self.cmd_publisher.publish(0.0)