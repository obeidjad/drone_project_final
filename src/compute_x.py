#!/usr/bin/env python
"""
This node will be responsible of stabilizing the drone on a given velocity
It subscribes to the odom topic and published the command to the /vel_x topic
"""
import numpy as np
import rospy
from nav_msgs.msg import Odometry
import sys
from projectTools import DroneCommand
from std_msgs.msg import Float32

class XCommand:
    def __init__(self):
        self.odom_subscriber = rospy.Subscriber("/bebop/odom",Odometry,self.read_val)
        self.dc = DroneCommand(0.7,0.03,0.6)
        self.cmd_publisher = rospy.Publisher("/vel_x",Float32, queue_size=1)
        self.linearX = 0
        self.targX = 0

    def read_data(self,ros_data):
        twist = ros_data.twist.twist
        self.linearX = twist.linear.x
        self.cmd = self.dc.computeCommand(self.linearX,self.targX)
        self.cmd_publisher.publish(self.cmd)

def main(args):
    rospy.init_node('computeX', anonymous=True)
    sc = XCommand()
    #rospy.init_node('send_command', anonymous=True)
    rospy.spin()
if __name__ == '__main__':
    main(sys.argv)
