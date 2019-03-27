#!/usr/bin/env python
"""
This node will be responsible of stabilizing the drone on the Z Axis
This velocity is computed based on the data collected from camera in a way to keep the centroid in the middle
It subscribes to the /centroids topic and published the command to the /vel_z topic
"""
import numpy as np
import rospy
import sys
from projectTools import DroneCommand,GenTools
from std_msgs.msg import Float32

class ZCommand(CommandClass):
    def __init__(self):
        self.cmd_publisher = rospy.Publisher("/vel_z",Float32, queue_size=1)
        self.cent_subscriber = rospy.Subscriber("/centroids",Float32,self.read_val)
        self.dc = DroneCommand(0.007,0,0)
        self.targZ = 160
    def read_val(self,ros_data):
        centr = ros_data.data
        zcmd = self.dc.computeCommand(centr,targZ)
        zcmd = GenTools.setMax(zcmd,0.3)
        self.cmd_publisher.publish(zcmd)

def main(args):
    rospy.init_node('computeZ', anonymous=True)
    sc = ZCommand()
    #rospy.init_node('send_command', anonymous=True)
    rospy.spin()
if __name__ == '__main__':
    main(sys.argv)

