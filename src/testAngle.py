#!/usr/bin/env python

import numpy as np
import rospy
from nav_msgs.msg import Odometry

def callback(ros_data):
    pose = ros_data.pose.pose
    quat = pose.orientation.z
    ang = 2*np.arcsin(quat)*180.0/np.pi
    print "The angle over z is : "+str(ang)
def listener():
    rospy.init_node('listener',anonymous=True)
    rospy.Subscriber('/bebop/odom',Odometry,callback)
    rospy.spin()
if __name__ == '__main__':
    listener()