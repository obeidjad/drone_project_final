#!/usr/bin/env python

"""
This node will read the value of the centroid from the centroids and compute the desired command to send it to the /vel_in_z
"""

import numpy as np
import rospy
import sys
from projectTools import GenTools,DroneCommand
from Regulator import RegulatorClass
from std_msgs.msg import Float32,Int32
from activation_class import NodeActivate

class ComputeZTar(NodeActivate):
    def __init__(self):
        super(ComputeZTar,self).__init__("compute_tar_z")
        cmd_reset = rospy.Publisher("/reset_cmd_z",Int32,queue_size=1)
        self.dc = DroneCommand(0.007,0,0)
        self.centroid_subscriber = rospy.Subscriber("/centroids",Float32,self.read_centroid)
        self.cmd_publisher = rospy.Publisher("/vel_in_z",Float32,queue_size=1)
        self.act_publisher = rospy.Publisher("/activation_z",Int32,queue_size=1)
        self.centroid = 160
        self.TargCentroid = 160
        self.cmd = 0
        self.act_publisher.publish(1)
    def read_centroid(self,ros_data):
        #read the centroid and send the data
        if(self.node_active == 0):
            return
        self.centroid = ros_data.data
        self.cmd = self.dc.computeCommand(self.centroid,self.TargCentroid)
        #Set the max value to 0.3 in absolute value
        self.cmd = GenTools.setMax(self.cmd,0.3)
        self.cmd_publisher.publish(self.cmd)
def main(args):
    rospy.init_node('computeTarZ', anonymous=True)
    sc = ComputeZTar()
    #rospy.init_node('send_command', anonymous=True)
    rospy.spin()
if __name__ == '__main__':
    main(sys.argv)


