#!/usr/bin/env python
"""
This node will be responsible of Collecting velocities from the 3 topics and send them to the /cmd_vel of the drone
"""
import numpy as np
import rospy
import sys
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

class collectData:
    def __init__(self):
        self.xcmd_subscriber = rospy.Subscriber("/vel_x", Float32, self.read_vel_x)
        self.ycmd_subscriber = rospy.Subscriber("/vel_y", Float32, self.read_vel_y)
        self.zcmd_subscriber = rospy.Subscriber("/vel_z", Float32, self.read_vel_z)
        self.cmd_publisher = rospy.Publisher("/cmd_vel",Twist,queue_size = 1)
        self.xcmd = 0
        self.ycmd = 0
        self.zcmd = 0

    def read_vel_x(self,ros_data):
        #Just updating values
        self.xcmd = ros_data.data

    def read_vel_y(self,ros_data):
        #Just updating values
        self.ycmd = ros_data.data

    def read_vel_z(self,ros_data):
        #update and sendData Here
        self.zcmd = ros_data.data
        my_cmd = Twist()

        my_cmd.linear.x = self.xcmd
        #my_cmd.linear.x = 0
        #my_cmd.linear.y = self.ycmd
        my_cmd.linear.y = 0
        my_cmd.linear.z = 0
        my_cmd.angular.x = 0
        my_cmd.angular.y = 0
        #my_cmd.angular.z = self.zcmd
        my_cmd.angular.z = 0
        self.cmd_publisher.publish(my_cmd)
def main(args):
    rospy.init_node('SendCommand', anonymous=True)
    sc = collectData()
    #rospy.init_node('send_command', anonymous=True)
    rospy.spin()
if __name__ == '__main__':
    main(sys.argv)
