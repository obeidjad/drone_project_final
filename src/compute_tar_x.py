#!/usr/bin/env python

"""
This node will publish the desired velocity along the x axis on the topic /vel_in_x
"""

import numpy as np
import rospy
import sys
from projectTools import GenTools,DroneCommand
from Regulator import RegulatorClass
from std_msgs.msg import Float32,Int32
from nav_msgs.msg import Odometry
from activation_class import NodeActivate

class ComputeXTar(NodeActivate):
    def __init__(self):
        super(ComputeXTar,self).__init__("computeTarX")
        cmd_reset = rospy.Publisher("/reset_cmd_x",Int32,queue_size=1)
        self.sync_sub = rospy.Subscriber("/bebop/odom",Odometry,self.sync_cmd)
        self.cmd_publisher = rospy.Publisher("/vel_in_x",Float32,queue_size=1)
        self.targX = 0.2
    def sync_cmd(self,ros_data):
        if(self.node_active == 0):
            return
        self.cmd_publisher.publish(self.targX)
def main(args):
    rospy.init_node('computeTarX', anonymous=True)
    sc = ComputeXTar()
    #rospy.init_node('send_command', anonymous=True)
    rospy.spin()
if __name__ == '__main__':
    main(sys.argv)


