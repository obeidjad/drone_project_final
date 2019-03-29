#!/usr/bin/env python

"""
This node will read the value of the slopes difference from the /sDiffs and compute the desired command to send it to the /vel_in_y
"""

import numpy as np
import rospy
import sys
from projectTools import GenTools,DroneCommand
from Regulator import RegulatorClass
from std_msgs.msg import Float32,Int32

class ComputeYTar:
    def __init__(self):
        self.sDiff_subscriber = rospy.Subscriber("/sDiffs",Float32,self.read_sDiffs)
        self.cmd_publisher = rospy.Publisher("/vel_in_y",Float32,queue_size=1)
        self.targY = 0
    def read_sDiffs(self,ros_data):
        #read the centroid and send the data
        self.targY = ros_data.data
        #The value of the velocity is proportional to the value of the difference of the values
        self.targY = self.targY * -0.1
        #Set the max of the Target to the 
        self.targY = GenTools.setMax(self.targY,0.3)
        self.cmd_publisher.publish(self.targY)
def main(args):
    rospy.init_node('computeTarZ', anonymous=True)
    sc = ComputeYTar()
    #rospy.init_node('send_command', anonymous=True)
    rospy.spin()
if __name__ == '__main__':
    main(sys.argv)


