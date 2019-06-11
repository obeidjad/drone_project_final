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
from std_msgs.msg import Float32,Int32,String
import math

class ComputeTarStairsUp(NodeActivate):
    def __init__(self):
        super(ComputeTarStairsUp,self).__init__("drone_vs_stairs_round2")        self.dcx = DroneCommand(2.0,0,0.05)
        self.dcang = DroneCommand(1.,0,0.0)
        self.dcy = DroneCommand(1.0,0.0,0.0)
        self.dcz = DroneCommand(1.0,0,0)

        self.stairs_presence = rospy.Subscriber("/stairs_presence",Int32,self.callback)
        #self.ready_to_get_up = rospy.Subscriber("/ready_to_get_up",Int32,self.position)

        self.st_ctr_x = rospy.Subscriber("/stairs_center_x_relative",Float32,self.lateralPositionCheck)
        self.low_st_y = rospy.Subscriber("/lowest_stair_y_relative",Float32,self.verticalPositionCheck)
        self.st_ang = rospy.Subscriber("/stairs_angle_rad",Float32,self.directionCheck)
        
        self.xcmd_publisher = rospy.Publisher("/vel_in_x",Float32,queue_size=1)
        self.ycmd_publisher = rospy.Publisher("/vel_in_y",Float32,queue_size=1)
        self.angcmd_publisher = rospy.Publisher("/vel_in_z",Float32,queue_size=1)
        self.zcmd_publisher = rospy.Publisher("/vel_in_alt",Float32,queue_size=1)
        #self.deact_publisher = rospy.Publisher("/stairs_over",Int32,queue_size=1)
        #TESTERS:
        #self.lat_pos = rospy.Publisher("/lat_pos_ok",Int32,queue_size=1)
        #self.dir_ok = rospy.Publisher("/dir_ok",Int32,queue_size=1)
        #self.vert_pos_ok = rospy.Publisher("/get_up",Int32,queue_size=1)

        self.stairs_detected = 1
        self.directionOK = False
        self.lateralPositionOK = False
        self.verticalPositionOK = False

        self.center_diff_x = 0 # >0 si escaliers a droite
        # self.center_diff_y = 0 # >0 si escalier au fond de l'image
        self.lowest_st_y = 0
        self.diff_angle = 0

        self.delta_angle = 0.02 #3 # empirique
        self.delta_centerX = 0.01 # a faire varier pour optimiser, normalement 0.04 devrait suffir
        self.delta_Y = 0.3 # cinquieme inferieur de l'image

        self.xcmd = 0
        self.ycmd = 0
        self.angcmd = 0
        self.zcmd = 0
        #self.deact_publisher.publish(0)

    #NODE ACTIVATION CONTROL
    def stairs_detection(self,ros_data):
        self.stairs_detected = ros_data.data
    #def position(self,ros_data):
    #    self.ready = ros_data.data
    #NODE DEACTIVATION WHEN STAIRS OVER -- NOT USED YET
    #def de_activation(self,ros_data):
    #    rec_msg = ros_data.data
    #    if rec_msg == 0:# pas d'escaliers ou pas ready
    #        self.deactivate = 1
    #    if rec_msg == 1:
    #        self.deactivate = 0

    def turn(self):
        self.angcmd = self.dcx.computeCommand(self.diff_angle,0.0)*0.1
        self.xcmd = 0.0
    
    def shift(self):
        #center_diff_x > 0 <=> se decaler vers la droite, soit ycmd positif
        if self.directionOK:
            self.ycmd = self.dcy.computeCommand(self.center_diff_x,0.0)*0.7
        else:
            self.ycmd = self.dcy.computeCommand(self.center_diff_x,0.0)
    def goforwards(self):
        self.angcmd = 0.0
        self.ycmd = 0.0
        self.xcmd = 0.25 #une marche par seconde
    
    def stop(self):
        self.angcmd = 0.0
        self.ycmd = 0.0
        self.xcmd = 0.0
        self.zcmd = 0.0
    
    def directionCheck(self,ros_data):
        if (self.node_active == True) and self.stairs_detected:
            self.diff_angle = ros_data.data - math.pi/2
            self.directionOK = abs(self.diff_angle) < self.delta_angle
            if not(self.directionOK) :
                self.turn()
            else:
                self.angcmd = 0.0

    def lateralPositionCheck(self, ros_data):
        if(self.node_active == True) and self.stairs_detected:
            self.center_diff_x = ros_data.data
            self.lateralPositionOK = abs(self.center_diff_x) < self.delta_centerX
            if not(self.lateralPositionOK) :
                    self.shift()
    
    def verticalPositionCheck(self, ros_data): 
        if(self.node_active == True):
            self.zcmd = 0.
            self.lowest_st_y = ros_data.data
            self.verticalPositionOK = self.lowest_st_y > self.delta_Y
            if self.directionOK and self.lateralPositionOK and not(self.verticalPositionOK):
                self.zcmd = 0.35
                #si on avance de 25cm on monte de 35cm car normes: 
                # Hauteur de marches : entre 17 et 21 cm. 
                # Giron de marche : entre 21 et 27 cm.

    def callback(self,ros_data):

        if(self.node_active == True):
            self.stairs_detection(ros_data)
            if self.stairs_detected:
                if self.directionOK and self.lateralPositionOK:
                    self.goforwards()
                #saturation
                self.angcmd = GenTools.setMax(self.angcmd,0.3)
                self.ycmd = GenTools.setMax(self.ycmd,0.1)
                #je ne mets pas de else stop pour pouvoir monter les escaliers jusqu'au bout mais ce serait interessant de tester
                self.angcmd_publisher.publish(self.angcmd)
                self.ycmd_publisher.publish(self.ycmd)
                self.xcmd_publisher.publish(self.xcmd)
                self.zcmd_publisher.publish(self.zcmd)
                #self.positionOK_publisher.publish(self.deactivate)
                #self.lat_pos.publish(self.lateralPositionOK)
                #self.dir_ok.publish(self.directionOK)

def main(args):
    rospy.init_node('drone_vs_stairs_round2', anonymous=True)
    sc = ComputeTarStairsUp()
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)