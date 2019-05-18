#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32,Float32,String
import sys
from PyQt4 import QtGui,QtCore

class Window(QtGui.QMainWindow):
    def __init__(self):
        super(Window,self).__init__()
        self.setGeometry(50,50,500,300)
        self.setWindowTitle("Command Test")
        self.mode_pub = rospy.Publisher("/mode",String,queue_size=1)
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

        btn = QtGui.QPushButton("Stairs",self)
        btn.clicked.connect(self.Stairs)
        btn.resize(100,70)
        btn.move(120,10)

        btn = QtGui.QPushButton("Doors",self)
        btn.clicked.connect(self.Doors)
        btn.resize(100,70)
        btn.move(230,10)
        self.show()
    def hallway_nav(self):
        self.mode_pub.publish("hallway")
    def Stairs(self):
        self.mode_pub.publish("stairs")
    def Doors(self):
        self.mode_pub.publish("door")

    def close_app(self):
        print "Closing"
        sys.exit()
def run():
    rospy.init_node('Main', anonymous=True)
    app = QtGui.QApplication(sys.argv)
    GUI = Window()

    sys.exit(app.exec_())
run()