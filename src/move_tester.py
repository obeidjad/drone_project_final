#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32,Int32
import time

def talker():
    pubres1 = rospy.Publisher("/reset_cmd_x", Int32, queue_size=1)
    pubres2 = rospy.Publisher("/reset_cmd_y", Int32, queue_size=1)
    pubres3 = rospy.Publisher("/reset_cmd_z", Int32, queue_size=1)

    pubvel1 = rospy.Publisher("/vel_in_x", Float32, queue_size=1)
    pubvel2 = rospy.Publisher("/vel_in_y", Float32, queue_size=1)
    pubvel3 = rospy.Publisher("/vel_in_z", Float32, queue_size=1)

    rospy.init_node('tester', anonymous=True)
    rate = rospy.Rate(5) # 5hz
    
    for i in range(5):
        pubres1.publish(1)
        pubres2.publish(1)
        pubres3.publish(1)
        rate.sleep()
    start = time.time()
    end = time.time()
    while end - start < 1:
        end = time.time()
        velx = 0.2
        vely = 0.0
        velz = 0.0

        pubvel1.publish(velx)
        pubvel2.publish(vely)
        pubvel3.publish(velz)

        rate.sleep()
    start = time.time()
    end = time.time()
    while end - start < 4:
        end = time.time()
        velx = 0.0
        vely = 0.0
        velz = -0.4
        
        pubvel1.publish(velx)
        pubvel2.publish(vely)
        pubvel3.publish(velz)

        rate.sleep()
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass