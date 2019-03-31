#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32,Int32

def talker():
    pubact1 = rospy.Publisher("/activation_x", Int32, queue_size=1)
    pubact2 = rospy.Publisher("/activation_y", Int32, queue_size=1)
    pubact3 = rospy.Publisher("/activation_z", Int32, queue_size=1)
    pubvel1 = rospy.Publisher("/vel_in_x", Float32, queue_size=1)
    pubvel2 = rospy.Publisher("/vel_in_y", Float32, queue_size=1)
    pubvel3 = rospy.Publisher("/vel_in_z", Float32, queue_size=1)
    pubres1 = rospy.Publisher("/reset_cmd_x", Int32, queue_size=1)
    pubres2 = rospy.Publisher("/reset_cmd_y", Int32, queue_size=1)
    pubres3 = rospy.Publisher("/reset_cmd_z", Int32, queue_size=1)
    
    rospy.init_node('tester', anonymous=True)
    pubres1.publish(1)
    pubres2.publish(1)
    pubres3.publish(1)
    
    pubact1.publish(0)
    pubact2.publish(1)
    pubact3.publish(0)
    rate = rospy.Rate(30) # 30hz
    while not rospy.is_shutdown():
        vel = 0.4
        pubvel1.publish(0.0)
        pubvel2.publish(vel)
        pubvel3.publish(0.0)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass