#!/usr/bin/env python

"""
This node will read the stairs data and compute the desired command to send it to the /vel_in
"""

import numpy as np
import rospy
import sys
from activation_class import NodeActivate
from projectTools import GenTools, DroneCommand
from Regulator import RegulatorClass
from std_msgs.msg import Float32, Int32, String
import math

class ComputeStairsTar(NodeActivate):
    def __init__(self):
        super(ComputeStairsTar,self).__init__("drone_vs_stairs_round1")
        self.dcx = DroneCommand(1.0,0,0.05)
        self.dcang = DroneCommand(1.0,0.,0)

        self.stairs_presence = rospy.Subscriber("/stairs_presence",Int32,self.de_activation)
        self.st_ctr_x = rospy.Subscriber("/lower_stairs_center_x_relative",Float32,self.directionCheck)
        self.st_ctr_y = rospy.Subscriber("/lower_stairs_center_y_relative",Float32,self.computeStairsCommand)
        
        self.angcmd_publisher = rospy.Publisher("/vel_in_z",Float32,queue_size=1)
        self.xcmd_publisher = rospy.Publisher("/vel_in_x",Float32,queue_size=1)

        self.positionOK_publisher = rospy.Publisher("/ready_to_get_up",Int32,queue_size=1)
        self.check = rospy.Publisher("/check",Int32,queue_size=1)
        self.check.publish(0)

        self.center_diff_x = 0 # >0 si escaliers a droite
        self.center_diff_y = 0 # >0 si escalier au fond de l'image
        self.delta_centerX = 0.005 # a faire varier pour optimiser
        self.delta_centerY = 0.3 # cinquieme inferieur de l'ecran #plus etait trop restrictif
        self.facingstairs = False
        self.xcmd = 0
        self.angcmd = 0
        self.no_stairs = 0
        self.ready_to_get_up = 0
        self.positionOK_publisher.publish(0)
        self.check_test()
    def check_test(self):
        self.check.publish(0)
    
    def de_activation(self,ros_data):
        if(self.node_active == True):
            rec_msg = ros_data.data
            if rec_msg == 0:
                self.no_stairs = 1
                #self.positionOK_publisher.publish(self.deactivate)
            if rec_msg == 1:
                self.no_stairs = 0

    def directionCheck(self,ros_data):
        if(self.node_active == True):
            self.center_diff_x = ros_data.data
            self.facingstairs = abs(self.center_diff_x) < self.delta_centerX

    def computeStairsCommand(self,ros_data):
        if(self.node_active == True):
            self.center_diff_y = ros_data.data
            if self.no_stairs == 0:
                if not(self.facingstairs):
                    self.angcmd = self.dcx.computeCommand(self.center_diff_x,0.0) #on veut un odg de 0.3 environ
                    self.xcmd = .1 #on vise 0.1 en odg 
                else:
                    
                    self.angcmd = 0.0 #stay still
                    self.xcmd = .1
                    if self.center_diff_y - self.delta_centerY > 0: # if lowest stair close:
                        self.xcmd = 0.0
                        self.ready_to_get_up = 1
                        self.send_conf() # ready to get up
                self.angcmd = GenTools.setMax(self.angcmd,0.3)
                self.angcmd_publisher.publish(self.angcmd)
                self.xcmd_publisher.publish(self.xcmd)
                self.positionOK_publisher.publish(self.ready_to_get_up)
        else:
            self.check.publish(1)

def main(args):
    rospy.init_node('drone_vs_stairs_round1', anonymous=True)
    sc = ComputeStairsTar()
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)