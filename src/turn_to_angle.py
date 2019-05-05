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
from nav_msgs.msg import Odometry

class TurnDrone:
    def __init__(self):
        self.tar_sub = rospy.Subscriber("/ang_in_z",Float32,self.read_tar)
        self.cur_sub = rospy.Subscriber("/bebop/odom",Odometry,self.read_cur)
        self.cmd_pub = rospy.Publisher("/vel_in_z",Float32,queue_size=1)
        self.tar_ang = 10.0/180
        self.cur_ang = 0
        self.dc = DroneCommand(0.05,0,0)
    def read_tar(self,ros_data):
        pass
    def read_cur(self,ros_data):
        curr_pose = ros_data.data.pose.pose
        cur_ang_quat = curr_pose.orientation.z
        self.cur_ang = 2*np.arcsin(cur_ang_quat)
        cmd = self.dc.computeCommand(self.cur_ang,self.tar_ang)
        self.cmd_pub.publish(cmd)
def main(args):
    rospy.init_node('TurnDrone', anonymous=True)
    sc = TurnDrone()
    #rospy.init_node('send_command', anonymous=True)
    rospy.spin()
if __name__ == '__main__':
    main(sys.argv)

