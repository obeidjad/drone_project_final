#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from drone_project.msg import actMsg
class returnResp(object):
    def __init__(self,name):
        self.ret_pub = rospy.Publisher("/return_resp",String,queue_size=10)
        self._name = name
    def send_conf(self):
        self.ret_pub.publish(self._name)
class NodeActivate(object):
    def __init__(self,my_name):
        self._my_name = my_name
        self.act_sub = rospy.Subscriber("/activations",actMsg,self.check_act)
        self.ret_pub = rospy.Publisher("/return_resp",String,queue_size=10)
        self.node_active = 0
    def send_conf(self):
        self.ret_pub.publish(self._my_name)
    def check_act(self,ros_data):
        node_name = ros_data.node_name
        active = ros_data.activate
        if(node_name == 'reset'):
            self.node_active = False
        else:
            if(node_name == self._my_name):
                self.node_active = active