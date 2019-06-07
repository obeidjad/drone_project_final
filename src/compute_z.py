#!/usr/bin/env python
"""
This node will just forward data from /vel_in to /vel_out 
It will get the target velocity from the topic /vel_in and republishes it to /vel_out
"""
import numpy as np
import rospy
import sys
from projectTools import GenTools
from Regulator import RegulatorClass
from std_msgs.msg import Float32,Int32
import time

class ZCommand(RegulatorClass):
    def __init__(self):
        super(ZCommand,self).__init__(0,0,0)
    #The Z is not like the others,we need to just publish the data to the out node, we will override the read_tar
    def read_tar(self,ros_data):
        #if(self.activation == 0):
        #    return
        #self.cmd_publisher.publish(ros_data.data)
        self.cmd = ros_data.data
        self.data_rec = 1
    def read_val(self,ros_data):
        if(np.absolute(self.last_rec - time.time()) > 0.5):
            self.reset_cmd()
        #self.cmd = self.dc.computeCommand(self.currVal,self.targVal)
        if(self.data_rec == 1):
            self.cmd_publisher.publish(self.cmd)
            self.data_rec = 0

def main(args):
    rospy.init_node('computeZ', anonymous=True)
    sc = ZCommand()
    #rospy.init_node('send_command', anonymous=True)
    rospy.spin()
if __name__ == '__main__':
    main(sys.argv)

