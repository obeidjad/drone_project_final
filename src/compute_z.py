#!/usr/bin/env python

import numpy as np
import rospy
import sys
from projectTools import DroneCommand,GenTools
from std_msgs.msg import Float32

class ZCommand:
    def __init__(self):
        self.cent_subscriber = rospy.Subscriber("/centroids",Float32,self.read_val)
        self.dc = DroneCommand(0.007,0,0)
        self.cmd_publisher = rospy.Publisher("/vel_z",Float32, queue_size=1)
    def read_val(self,ros_data):
        centr = ros_data
        zcmd = self.dc.computeCommad(centr,160)
        zcmd = GenTools.setMax(zcmd,0.3)
        self.cmd_publisher.publish(zcmd)

def main(args):
    rospy.init_node('computeZ', anonymous=True)
    sc = ZCommand()
    #rospy.init_node('send_command', anonymous=True)
    rospy.spin()
if __name__ == '__main__':
    main(sys.argv)

