#!/usr/bin/env python

import numpy as np
import rospy
from nav_msgs.msg import Odometry
import sys
from projectTools import DroneCommand
from std_msgs.msg import Float32,Int32
import time
class RegulatorClass(object):
    def __init__(self,P,I,D):
        self.odom_subscriber = rospy.Subscriber("/curr_in",Odometry,self.read_val)
        self.vel_subscriber = rospy.Subscriber("/tar_in",Float32,self.read_tar)
        #self.act_subscriber = rospy.Subscriber("/activation",Int32,self.check_activation)
        self.cmd_reset = rospy.Subscriber("/reset_cmd",Int32,self.reset_cmd)
        
        self.dc = DroneCommand(P,I,D)
        self.cmd_publisher = rospy.Publisher("/vel_out",Float32, queue_size=1)
        self.ack_reset = rospy.Publisher("/ack_res",Int32,queue_size=1)
        self.currVal = 0
        self.targVal = 0
        self.reset_ack = 0
        self.data_rec = 0
        self.last_rec = -1
    #    self.activation = 0
    def read_tar(self,ros_data):
        print "tar received"
        #if(self.activation == 0):
        #    return 
        self.targVal = ros_data.data
        self.data_rec = 1
        self.last_rec = time.time()
        #self.cmd = self.dc.computeCommand(self.currVal,self.targVal)
        #self.cmd_publisher.publish(self.cmd)
        print "cmd sent"
    def myreset_cmd(self):
        self.dc.cmd = 0
        self.dc.TotErr = 0
        self.dc.oldErr = 0
        self.reset_ack = 1
    def reset_cmd(self,ros_data):
        pass