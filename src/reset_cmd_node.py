#!/usr/bin/env python

import rospy
import sys
from std_msgs.msg import Float32,Int32
from activation_class import NodeActivate
from nav_msgs.msg import Odometry


class resetCmd(NodeActivate):
    def __init__(self):
        super(resetCmd,self).__init__("resetCmd")
        
        self.pubres1 = rospy.Publisher("/reset_cmd_x", Int32, queue_size=1)
        self.pubres2 = rospy.Publisher("/reset_cmd_y", Int32, queue_size=1)
        self.pubres3 = rospy.Publisher("/reset_cmd_z", Int32, queue_size=1)
        self.sync_sub = rospy.Subscriber("/bebop/odom",Odometry,self.reset_cmd_func)
        self.rate = rospy.Rate(10)

    def reset_cmd_func(self,ros_data):
        if(self.node_active == 0):
            return
        for i in range(10) :
            self.pubres1.publish(1)
            self.pubres2.publish(1)
            self.pubres3.publish(1)
            self.rate.sleep()
        self.node_active = 0