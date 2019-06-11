#!/usr/bin/env python
"""
This node is the sequencer, This node will subscribe to the topic mode , 
and will choose the nodes to activate sequentially
"""
import rospy
from std_msgs.msg import String,Int32,Float32
from projectTools import Sequence
from drone_project.msg import actMsg
import sys

class Sequencer(object):
    def __init__(self):
        self.mode_sub = rospy.Subscriber("/mode",String,self.read_mode)
        self.mode_pub = rospy.Publisher("/mode",String,queue_size=1)
        self.actv_publisher = rospy.Publisher("/activations",actMsg,queue_size=1)
        self.loop_pub = rospy.Publisher("/loop_seq",Int32,queue_size=1)
        self.loop_sub = rospy.Subscriber("/loop_seq",Int32,self.enter_loop)
        
        self.mode = "init"
        """
        Init should be a list and not a name
        """
        self.doors_seq_list = ["doors",[["curveMotion"],["curveMotion"]],[["checkDoors","turnAng"],["checkDoors"]]]
        self.init_seq_list = ["init",[ ["resetCmd"],[] ] ]
        self.hallway_seq_list = ["hallway",[["detectVanish","computeTarX","computeTarY","computeTarZ"], [""] ] ]
        self.doors_seq = Sequence(self.doors_seq_list)
        self.init_seq = Sequence(self.init_seq_list)
        self.hallways_seq = Sequence(self.hallway_seq_list)
        #self.new_seq_list = []
        #self.new_seq = Sequence(self.new_seq_list)

        self.rate = rospy.Rate(20) #10Hz
    def read_mode(self,ros_data):
        self.mode = ros_data.data
        if(self.mode == "init"):
            msg = actMsg()
            msg.node_name = "reset"
            msg.activate = True
            self.actv_publisher.publish(msg)
            self.doors_seq = Sequence(self.doors_seq_list)
            self.hallways_seq = Sequence(self.hallway_seq_list)
            #self.new_seq = Sequence(self.new_seq_list)
        else:
            self.loop_pub.publish(3)
    def enter_loop(self,ros_data):
        if(self.mode == self.init_seq.get_mode()):
            #self.new_seq.reset_seq()
            self.init_seq.seq_fun()
        if(self.mode == self.doors_seq.get_mode()):
            self.doors_seq.seq_fun()
        #if(self.mode == self.new_seq.get_mode()):
            #self.new_seq.seq_fun()
        if(self.mode == self.hallways_seq.get_mode()):
            self.hallways_seq.seq_fun()
def main(args):
    rospy.init_node('Sequencer', anonymous=True)
    sc = Sequencer()
    #rospy.init_node('send_command', anonymous=True)
    rospy.spin()
if __name__ == '__main__':
    main(sys.argv)