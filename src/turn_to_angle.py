#!/usr/bin/env python
"""
This node has an input the target yaw angle and will send the corresponding velocity over the Z axis
The input node is the topic /desired_yaw and the output is the /vel_in_z after activating and reseting the Z
"""

import numpy as np
import rospy
import sys
from projectTools import GenTools,DroneCommand
from Regulator import RegulatorClass
from std_msgs.msg import Float32,Int32

class TurnDrone(RegulatorClass):
    def __init__(self):
        super(TurnDrone,self).__init__(0.05,0,0)
    def read_val(self,ros_data):
        mypose = ros_data.pose.pose
        QuatcurrVal = mypose.orientation.z
        self.currVal = 2*np.arcsin(QuatcurrVal)
def main(args):
    rospy.init_node('TurnDrone', anonymous=True)
    sc = TurnDrone()
    #rospy.init_node('send_command', anonymous=True)
    rospy.spin()
if __name__ == '__main__':
    main(sys.argv)

