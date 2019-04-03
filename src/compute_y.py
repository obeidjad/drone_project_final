#!/usr/bin/env python
"""
This node will be responsible of stabilizing the drone on a given velocity on the Y Axis
It will get the target velocity from the topic /vel_in and publishes the tilt to /vel_out
"""

import numpy as np
import rospy
from nav_msgs.msg import Odometry
import sys
from projectTools import GenTools
from std_msgs.msg import Float32
from Regulator import RegulatorClass


class YCommand(RegulatorClass):
    def __init__(self):
        super(YCommand,self).__init__(0.7,0.03,0.6)

    def read_val(self,ros_data):
        #In this Topic here we need to send the command to the drone
        #We need to compute the command first in this method
        #if(self.activation == 0):
        #    return
        twist = ros_data.twist.twist
        self.currVal = twist.linear.y
        self.targVal = GenTools.setMax(self.targVal,0.3)
        #ycmd = self.dc.computeCommand(self.currVal,self.targVal)
        #self.cmd_publisher.publish(ycmd)

def main(args):
    rospy.init_node('computeY', anonymous=True)
    sc = YCommand()
    rospy.spin()
if __name__ == '__main__':
    main(sys.argv)