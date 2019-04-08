#!/usr/bin/env python
"""
This node will is for testing the optical flow in order to enter through doors 
"""

import numpy as np
import cv2
import rospy
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
import sys
from matplotlib import pyplot as plt
import math
from cv_bridge import CvBridge, CvBridgeError

class EnterDoors:
    def __init__(self):
        self.img_read = rospy.Subscriber("/image_in",Image,self.transform_image)
        self.br = CvBridge()
        self.prvs = None
        self.next = None
        self.hsv = None
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlabel('Flow')
        self.bins = 320
        self.ax.set_ylabel('mag')
        self.lw = 3
        self.alpha = 0.5
        self.lineGray, = self.ax.plot(np.arange(self.bins), np.zeros((self.bins,1)), c='k', lw=self.lw)
        self.ax.set_xlim(0, self.bins-1)
        self.ax.set_ylim(0, 20)
        plt.ion()

        # Parameters for lucas kanade optical flow
    def transform_image(self,ros_data):
        cv2_image = self.br.imgmsg_to_cv2(ros_data)
        cv2_image = cv2.resize(cv2_image, (320,240), interpolation = cv2.INTER_AREA)
        cv2_gray = cv2.cvtColor(cv2_image,cv2.COLOR_BGR2GRAY)
        self.hsv = np.zeros_like(cv2_image)
        if(self.prvs is None):
            self.prvs = cv2_gray
        else:
            self.next = cv2_gray
            self.draw_and_dispaly() 
            self.fig.canvas.draw()
            self.prvs = self.next.copy()
        
    def draw_and_dispaly(self):
        flow = cv2.calcOpticalFlowFarneback(self.prvs,self.next, None, 0.5, 2, 15, 3, 5, 1.2, 0)
        mag, ang = cv2.cartToPolar(flow[...,0], flow[...,1])
        mag[mag < 2] = 0
        ang2 = np.cos(ang)
        overX = np.multiply(mag,ang2)
        dat = overX.mean(0)
        dat = np.absolute(dat)
        self.hsv[...,0] = ang*180/np.pi/2
        self.hsv[...,2] = cv2.normalize(mag,None,0,255,cv2.NORM_MINMAX)
        bgr = cv2.cvtColor(self.hsv,cv2.COLOR_HSV2BGR)
        cv2.imshow('frame2',bgr)
        cv2.imshow('frame',self.next)
        self.lineGray.set_ydata(dat)

def main(args):
    rospy.init_node('ShowPoints', anonymous=True)
    sc = EnterDoors()
    #rospy.init_node('send_command', anonymous=True)
    #rospy.spin()
    plt.show(block=True)
if __name__ == '__main__':
    main(sys.argv)


