#!/usr/bin/env python

import numpy as np
import rospy
from nav_msgs.msg import Odometry
import sys
from projectTools import DroneCommand
from std_msgs.msg import Float32

class Regulator:
    def __init__(self):
        self.odom_subscriber = rospy.Subscriber("/odom_data",Odometry,self.read_val)
        self.vel_subscriber = rospy.Subscriber("/in_vel",Float32,self.read_tar)
        