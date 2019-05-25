#!/usr/bin/env python

import rospy
from std_msgs.msg import String,Int32
from drone_project.msg import actMsg

class DroneCommand:
    def __init__(self,P,I,D):
        self._P = P
        self._I = I
        self._D = D
        self.TotErr = 0
        self.oldErr = 0
        self.cmd = 0
        self.MAX_CMD = 10
    def computeCommand(self,currentVal,targetVal):
        err = targetVal - currentVal
        if(abs(self.cmd) < self.MAX_CMD):
            self.TotErr = self.TotErr + err
        if(self.oldErr != 0):
            Derr = err - self.oldErr
        else:
            Derr = 0
        self.cmd = self._P*err + self.TotErr*self._I + self._D*Derr
        return self.cmd

##############################################################################################################

class Sequence:
    def __init__(self,seqList):
        self._seqList = seqList
        self._phase = 0
        self._published=False
        self._mode = seqList[0]
        self._seq = seqList[1:]
        self._phase_nb = len(self._seq)
        self.nodes_to_activate = self._seq[self._phase][0]
        self.conds_to_wait = self._seq[self._phase][1]
        self._verf_conds = [False]*len(self.conds_to_wait)
        self.actv_pub = rospy.Publisher("/activations",actMsg,queue_size=1)
        self.resp_sub = rospy.Subscriber("/return_resp",String,self.updateConds)
        self.loop_pub = rospy.Publisher("/loop_sep",Int32,queue_size=1)
        self.mode_pub = rospy.Publisher("/mode",String,queue_size=1)
        self.rate = rospy.Rate(20) #20Hz
    def reset_seq(self):
        self._phase = 0
        self._published = False
    def updateConds(self,ros_data):
        received = ros_data.data
        if(received in self.conds_to_wait):
            ind = self.conds_to_wait.index(received)
            self._verf_conds[ind] = True

    def seq_fun(self):
        self.nodes_to_activate = self._seq[self._phase][0]
        self.conds_to_wait = self._seq[self._phase][1]
        #First we get the nodes we want
        if(self._published == False):
            self.send_activations()
        if(self.wait_conds()):
            self._phase = self._phase + 1
            if(self._phase < self._phase_nb):
                self._published = False
                self._verf_conds = [False]*len(self._seq[self._phase][1])
                self.nodes_to_activate = self._seq[self._phase][0]
                self.conds_to_wait = self._seq[self._phase][1]
                self.loop_pub.publish(1)
            else:
                #The sequence is over here
                self.mode_pub.publish("init")

    def send_activations():
        #reset message and send some
        self._published = True
        msg = actMsg()
        msg.node_name = "reset"
        msg.activate = True
        self.actv_pub.publish(msg)
        for i in range(len(self.nodes_to_activate)):
		    msg.node_name = self.nodes_to_activate[i]
            msg.activate = True
            self.actv_pub.publish(msg)
        self.rate.sleep()
    def wait_conds():
        return all(item == True for item in self._verf_conds)
    def get_mode(self):
        return self._mode


##########################################################################################################
class GenTools:
    def __init__(self):
        pass
    @staticmethod
    def setMax(val,vmax):
        vmax = abs(vmax)
        if(val > vmax):
            return vmax
        elif(val <  -vmax):
            return -vmax
        else:
            return val