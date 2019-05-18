#!/usr/bin/env python
import rospy
from std_msgs.msg import String

class NodeActivate(object):
    def __init__(self,my_name):
        self._my_name = my_name
        self.act_sub = rospy.Subscriber("/activations",String,self.check_act)
        self.node_active = 0
    def check_act(self,ros_data):
        msg = ros_data.data
        if(msg == reset):
            self.node_active = 0
        else:
            msg,act = msg.split("_")
            if(msg == self._my_name):
                self.node_active = act