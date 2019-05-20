#!/usr/bin/env python
"""
This node is the sequencer, This node will subscribe to the topic mode , 
and will choose the nodes to activate sequentially
"""
import rospy
from std_msgs.msg import String,Int32,Float32
from projectTools import Sequence
import sys

class Sequencer(object):
    def __init__(self):
        self.mode_sub = rospy.Subscriber("/mode",String,self.read_mode)
        self.mode_pub = rospy.Publisher("/mode",String,queue_size=1)
        self.actv_pub = rospy.Publisher("/activations",String,queue_size=1)

        self.loop_pub = rospy.Publisher("/loop_seq",Int32,queue_size=1)
        self.loop_sub = rospy.Subscriber("/loop_seq",Int32,self.enter_loop)

        self.turn_done_sub = rospy.Subscriber("/turn_done",Int32,self.turn_done_callback)
        self.det_doors_sub = rospy.Subscriber("/door_det",Int32,self.door_det_callback)
        
        self.mode = "init"

        self.doors_seq = Sequence("doors")
        self.stairs_seq = Sequence("stairs")
        self.hallway_seq = Sequence("hallway")
        self.init_seq = Sequence("init")

        self.rate = rospy.Rate(20) #10Hz
    def turn_done_callback(self,ros_data):
        self.doors_seq.set_phase(1)
        self.doors_seq.set_published(False)
    def door_det_callback(self,ros_data):
        self.doors_seq.set_phase(2)
        self.doors_seq.set_published(False)
    def read_mode(self,ros_data):
        self.mode = ros_data.data
        self.loop_pub.publish(3)
    def enter_loop(self,ros_data):
        if(self.mode == "init"):
            self.doors_seq.reset_seq()
            self.reset_seq_func()
            pass
        if(self.mode == self.doors_seq.get_mode()):
            self.doors_seq_func()
            pass
        if(self.mode == self.stairs_seq.get_mode()):
            self.stairs_seq_func()
            pass
        if(self.mode == self.hallway_seq.get_mode()):
            self.hallway_seq_func()
            pass
    def reset_seq_func(self):
        self.actv_pub.publish("reset")
        self.actv_pub.publish("resetCmd_1")
    def stairs_seq_func(self):
        pass
    def hallway_seq_func(self):
        pass
    def doors_seq_func(self):
        if(self.doors_seq.get_phase() == 0 ):
            if (self.doors_seq.get_publ() == False) :
                self.actv_pub.publish("reset")
                self.rate.sleep()
                self.actv_pub.publish("curveMotion_1")
                self.rate.sleep()
                self.doors_seq.set_published(True)
            if(self.mode != "init"):
                self.loop_pub.publish(1)
                self.rate.sleep()
        if(self.doors_seq.get_phase() == 1):
            if(self.doors_seq.get_publ() == False):
                self.actv_pub.publish("reset")
                self.rate.sleep()
                self.actv_pub.publish("checkDoors_1")
                self.actv_pub.publish("turnAng_1")
                self.rate.sleep()
                self.doors_seq.set_published(True)
            if(self.mode != "init"):
                self.loop_pub.publish(2)
                self.rate.sleep()
        if(self.doors_seq.get_phase() == 2):
            self.actv_pub.publish("reset")
            self.rate.sleep()
            self.mode_pub.publish('init')

def main(args):
    rospy.init_node('Sequencer', anonymous=True)
    sc = Sequencer()
    #rospy.init_node('send_command', anonymous=True)
    rospy.spin()
if __name__ == '__main__':
    main(sys.argv)