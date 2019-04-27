#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32,Int32

def talker():
    
    
    pubres1 = rospy.Publisher("/reset_cmd_x", Int32, queue_size=1)
    pubres2 = rospy.Publisher("/reset_cmd_y", Int32, queue_size=1)
    pubres3 = rospy.Publisher("/reset_cmd_z", Int32, queue_size=1)
    pubres4 = rospy.Publisher("/reset_cmd_ang_z",Int32,queue_size=1)

    pubact1 = rospy.Publisher("/activation_x", Int32, queue_size=1)
    pubact2 = rospy.Publisher("/activation_y", Int32, queue_size=1)
    pubact3 = rospy.Publisher("/activation_z", Int32, queue_size=1)
    pubact4 = rospy.Publisher("/activation_ang_z", Int32, queue_size=1)


    pubvel1 = rospy.Publisher("/vel_in_x", Float32, queue_size=1)
    pubvel2 = rospy.Publisher("/vel_in_y", Float32, queue_size=1)
    pubvel3 = rospy.Publisher("/vel_in_z", Float32, queue_size=1)
    pubvel4 = rospy.Publisher("/ang_in_z", Float32, queue_size=1)

    
    rospy.init_node('tester', anonymous=True)

    pubres1.publish(1)
    pubres2.publish(1)
    pubres3.publish(1)
    pubres4.publish(1)

    rate = rospy.Rate(5) # 5hz
    while not rospy.is_shutdown():
        velx = 0.0
        vely = 0.0
        velz = 0.0
        ang = 20
        
        pubact1.publish(1)
        pubact2.publish(1)
        pubact3.publish(0)
        pubact4.publish(1)

        pubvel1.publish(velx)
        pubvel2.publish(vely)
        pubvel3.publish(velz)
        pubvel4.publish(ang)

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass