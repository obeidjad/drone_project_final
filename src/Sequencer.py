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
        
        self.mode = "init"
        """
        Init should be a list and not a name
        """
        doors_seq_list = ["doors",[["curveMotion"],["curveMotion"]],[["checkDoors","turnAng"],["checkDoors"]]]
        init_seq_list = ["init",[ ["resetCmd"],[] ] ]
        self.doors_seq = Sequence(doors_seq_list)
        self.init_seq = Sequence(init_seq_list)

        self.rate = rospy.Rate(20) #10Hz
    def read_mode(self,ros_data):
        self.mode = ros_data.data
        self.loop_pub.publish(3)
    def enter_loop(self,ros_data):
        if(self.mode == self.init_seq.get_mode()):
            self.doors_seq.reset_seq()
            #self.hallway_seq.reset_seq()
            self.init_seq.seq_func()
        
        if(self.mode == self.doors_seq.get_mode()):
            self.doors_seq.seq_fun()
def main(args):
    rospy.init_node('Sequencer', anonymous=True)
    sc = Sequencer()
    #rospy.init_node('send_command', anonymous=True)
    rospy.spin()
if __name__ == '__main__':
    main(sys.argv)