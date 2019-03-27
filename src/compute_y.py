#!/usr/bin/env python
"""
This node will be responsible of stabilizing the drone on a given velocity on the Y Axis
This velocity is computed based on the data collected form odom sensor and from camera
It subscribes to the odom topic, and the /sDiffs topic and published the command to the /vel_y topic
"""

import numpy as np
import rospy
from nav_msgs.msg import Odometry
import sys
from projectTools import DroneCommand,GenTools
from std_msgs.msg import Float32

class YCommand:
    def __init__(self):
        self.odom_subscriber = rospy.Subscriber("/bebop/odom",Odometry,self.read_val_odom)
        self.sDiff_subscriber = rospy.Subscriber("/sDiffs",Float32,self.read_val_sDiff)
        self.cmd_publisher = rospy.Publisher("/vel_y",Float32, queue_size=1)
        self.targY = 0
        self.localsDiffs = 0
        self.dc = DroneCommand(0.7,0.03,0.6)
        self.linearY = 0

    def read_val_odom(self,ros_data):
        #In this Topic here we need to send the command to the drone
        #We need to compute the command first in this method
        twist = ros_data.twist.twist
        self.linearY = twist.linear.y
        self.targY = -0.1*self.localsDiffs
        self.targY = GenTools.setMax(self.targY,0.3)
        ycmd = self.dc.computeCommand(self.linearY,self.targY)
        self.cmd_publisher.publish(ycmd)

    def read_val_sDiff(self,ros_data):
        #This method is just to update a variable that gives us the difference between slopes
        self.localsDiffs = ros_data.data
def main(args):
    rospy.init_node('computeY', anonymous=True)
    sc = YCommand()
    #rospy.init_node('send_command', anonymous=True)
    rospy.spin()
if __name__ == '__main__':
    main(sys.argv)