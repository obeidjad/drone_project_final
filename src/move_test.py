#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32,Int32

class TurnDrone:
    def __init__(self):
        self.sub_ack_1 = rospy.Subscriber("/ack_res_x",Int32,update_ack_x)
        self.sub_ack_2 = rospy.Subscriber("/ack_res_y",Int32,update_ack_y)
        self.sub_ack_3 = rospy.Subscriber("/ack_res_z",Int32,update_ack_z)
        
        self.ack_1 = 0
        self.ack_2 = 0
        self.ack_3 = 0
        
        self.pubres1 = rospy.Publisher("/reset_cmd_x", Int32, queue_size=1)
        self.pubres2 = rospy.Publisher("/reset_cmd_y", Int32, queue_size=1)
        self.pubres3 = rospy.Publisher("/reset_cmd_z", Int32, queue_size=1)

        self.pubact1 = rospy.Publisher("/activation_x", Int32, queue_size=1)
        self.pubact2 = rospy.Publisher("/activation_y", Int32, queue_size=1)
        self.pubact3 = rospy.Publisher("/activation_z", Int32, queue_size=1)

        self.pubvel1 = rospy.Publisher("/vel_in_x", Float32, queue_size=1)
        self.pubvel2 = rospy.Publisher("/vel_in_y", Float32, queue_size=1)
        self.pubvel3 = rospy.Publisher("/vel_in_z", Float32, queue_size=1)
        self.send_reset()
        self.send_command()
    def update_ack_x(self,ros_data):
        self.ack_1 = 1
    def update_ack_y(self,ros_data):
        self.ack_2 = 1
    def update_ack_z(self,ros_data):
        self.ack_3 = 1
    def send_reset(self):
        self.pubres1.publish(1)
        self.pubres2.publish(1)
        self.pubres3.publish(1)
        if (self.ack_1 != 1 or self.ack_2 != 0 or self.ack_3 != 0):
            self.send_reset()
    def send_command(self):
        rate = rospy.Rate(5) # 5hz
        while not rospy.is_shutdown():
            velx = 0.2
            vely = 0.0
            velz = 0.0
            ang = 20.0/180
            
            #self.pubact1.publish(1)
            #self.pubact2.publish(1)
            #self.pubact3.publish(0)

            self.pubvel1.publish(velx)
            self.pubvel2.publish(vely)
            self.pubvel3.publish(velz)

            rate.sleep()
def main(args):
    rospy.init_node('MoveTest', anonymous=True)
    sc = TurnDrone()
    #rospy.init_node('send_command', anonymous=True)
if __name__ == '__main__':
    main(sys.argv)