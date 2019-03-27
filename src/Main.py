#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32,Float32
import sys
from PyQt4 import QtGui,QtCore

class Window(QtGui.QMainWindow):
    def __init__(self):
        super(Window,self).__init__()
        self.setGeometry(50,50,500,300)
        self.setWindowTitle("Command Test")
        self.pub = rospy.Publisher("/mode", Int32, queue_size=1)
        self.pub.publish(0)
        self.pubx = rospy.Publisher("/in_vel_x", Float32, queue_size=1)
        self.puby = rospy.Publisher("/in_vel_y", Float32, queue_size=1)

        rospy.init_node("Project_Master", anonymous=True)
        self.xval = 0.2
        self.yval = 0
        self.home()
    def home(self):
        btn = QtGui.QPushButton("Quit",self)
        btn.clicked.connect(self.close_app)
        btn.resize(70,50)
        btn.move(430,230)

        btn = QtGui.QPushButton("Hallway",self)
        btn.clicked.connect(self.hallway_nav)
        btn.resize(100,70)
        btn.move(10,10)

        btn = QtGui.QPushButton("Constant V",self)
        btn.clicked.connect(self.cnstnt_v)
        btn.resize(100,70)
        btn.move(120,10)

        btn = QtGui.QPushButton("Freeze",self)
        btn.clicked.connect(self.freeze_drone)
        btn.resize(100,70)
        btn.move(230,10)

        self.show()
    def hallway_nav(self):
        #In this Mode the drone will navigate in the hallways
        self.pubx.publish(self.xval)
        cmd = 1;
        self.pub.publish(cmd)
    
    def cnstnt_v(self):
        #In this mode the drone will move at a givem velocity , 
        #this velocity is predefined here in the code and it will be sent to /in_vel_x and /in_vel_y
        self.pubx.publish(self.xval)
        self.puby.publish(self.yval)
        cmd = 2;
        self.pub.publish(cmd)
    def freeze_drone(self):
        #In this mode the drone will receive a twist of zero and will stay stable in the position
        cmd = 0;
        self.pub.publish(cmd)

    def close_app(self):
        print "Closing"
        sys.exit()
def run():
    app = QtGui.QApplication(sys.argv)
    GUI = Window()
    sys.exit(app.exec_())
run()