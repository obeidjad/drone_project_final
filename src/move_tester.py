#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32,Int32

def talker():
    sub_ack_1 = rospy.Subscriber("/ack_res_x",Int32,update_ack_x)
    sub_ack_2 = rospy.Subscriber("/ack_res_y",Int32,update_ack_y)
    sub_ack_3 = rospy.Subscriber("/ack_res_z",Int32,update_ack_z)
    sub_ack_4 = rospy.Subscriber("/ack_res_a",Int32,update_ack_a)
    
    ack_1 = 0
    ack_2 = 0
    ack_3 = 0
    ack_4 = 0
    
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
        velx = 0.2
        vely = 0.0
        velz = 0.0
        ang = 20.0/180
        
        pubact1.publish(1)
        pubact2.publish(1)
        pubact3.publish(0)
        pubact4.publish(0)

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