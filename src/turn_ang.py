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
from activation_class import NodeActivate

class TurnDrone(NodeActivate):
    def __init__(self):
        super(TurnDrone,self).__init__("turnAng")
        self.tar_sub = rospy.Subscriber("/ang_in_z",Float32,self.read_tar)
        self.cur_sub = rospy.Subscriber("/bebop/odom",Odometry,self.read_cur)
        self.cmd_pub_z = rospy.Publisher("/vel_in_z",Float32,queue_size=1)
        self.tar_ang = 90.0/180*np.pi
        self.cur_ang = 0
        self.dc = DroneCommand(0.5,0,0)
        self.start = time.time()
        self.end = time.time()
    def read_tar(self,ros_data):
        self.tar_ang = ros_data.data/180*np.pi
    def read_cur(self,ros_data):
        if(self.node_active == 0):
            return
        curr_pose = ros_data.pose.pose
        cur_ang_quat = curr_pose.orientation.z
        self.cur_ang = 2*np.arcsin(cur_ang_quat)
        cmd = self.dc.computeCommand(self.cur_ang,self.tar_ang)
        cmd = GenTools.setMax(cmd,0.3)
        self.cmd_pub_z.publish(cmd)
        
def main(args):
    rospy.init_node('TurnDrone', anonymous=True)
    sc = TurnDrone()
    #rospy.init_node('send_command', anonymous=True)
    rospy.spin()
if __name__ == '__main__':
    main(sys.argv)

