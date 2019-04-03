#!/usr/bin/env python
"""
This node will be responsible of stabilizing the drone on a given velocity on the X Axis
It will get the target velocity from the topic /vel_in and publishes the tilt to /vel_out
"""
import numpy as np
import rospy
from nav_msgs.msg import Odometry
import sys
from Regulator import RegulatorClass
from std_msgs.msg import Float32

class XCommand(RegulatorClass):
    def __init__(self):
        super(XCommand,self).__init__(0.7,0.03,0.6)
    def read_val(self,ros_data):
        twist = ros_data.twist.twist
        self.currVal = twist.linear.x
def main(args):
    rospy.init_node('computeX', anonymous=True)
    sc = XCommand()
    #rospy.init_node('send_command', anonymous=True)
    rospy.spin()
if __name__ == '__main__':
    main(sys.argv)
