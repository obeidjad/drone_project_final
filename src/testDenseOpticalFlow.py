#!/usr/bin/env python
"""
This node will is for testing the optical flow in order to detect opened doors
"""

import numpy as np
import cv2
import rospy
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
import sys
import math
from cv_bridge import CvBridge, CvBridgeError
import random

class EnterDoors:
    def __init__(self):
        self.img_read = rospy.Subscriber("/bebop/image_raw/compressed",CompressedImage,self.transform_image)
        self.img_pub = rospy.Publisher("/image_graph",Image,queue_size=1)
        self.roi_pub = rospy.Publisher("/image_roi",Image,queue_size=1)
        self.br = CvBridge()
        self.prvs = None
        self.next = None
        self.hsv = None
        self.plot_image = None
        self.old_dat = None
        self.alpha = 0.25
        self.alpha2 = 0.65
        self.thresh = 20
        self.dat_bin = None
        # Parameters for lucas kanade optical flow
    def transform_image(self,ros_data):
        cv2_image = self.br.compressed_imgmsg_to_cv2(ros_data)
        cv2_image = cv2.resize(cv2_image, (320,240), interpolation = cv2.INTER_AREA)
        cv2_roi = cv2_image[80:160,:]
        cv2_gray = cv2.cvtColor(cv2_image,cv2.COLOR_BGR2GRAY)
        cv2_gray_roi = cv2_gray[80:160,:]
        self.hsv = np.zeros_like(cv2_roi)
        self.plot_image = np.ones_like(cv2_gray)
        self.plot_image = 255*self.plot_image
        if(self.prvs is None):
            self.prvs = cv2_gray_roi
        else:
            self.next = cv2_gray_roi
            self.draw_and_dispaly() 
            self.prvs = self.next.copy()
    def check_doors(self,limits_idx):
        for i in range(len(limits_idx) - 1):
            left = limits_idx[i]
            right = limits_idx[i+1]
            img = self.next[...,left:right]
            print img.shape
            height, width = img.shape
            if self.dat_bin[int((left+right)/2)] == 0:
                #In this case we have no optical flow, we need to compute the variance
                myrands_x = random.sample(xrange(width),width)
                myrands_y = random.sample(xrange(height),10)
                tmp_pixels_x = img[...,myrands_x]
                tmp_pixels_y = img[myrands_y,...]
                var_x = np.var(tmp_pixels_x,axis = 0)
                var_y = np.var(tmp_pixels_y,axis = 1)
                m_var_x = np.mean(var_x)
                m_var_y = np.mean(var_y)
                if m_var_x > 1000 and m_var_y > 1000 :
                    print "Opened door detected from "+str(left)+" to "+str(right)
    def draw_and_dispaly(self):
        flow = cv2.calcOpticalFlowFarneback(self.prvs,self.next, None, 0.5, 2, 15, 3, 5, 1.2, 0)
        u = flow[...,0]
        v = flow[...,1]
        mag, ang = cv2.cartToPolar(u, v)
        mag[mag < 2] = 0
        dat2 = np.absolute(u.mean(0))
        dat = u.mean(0)
        dat = np.absolute(dat)
        dat = dat*10
        dat2 = dat2*10
        dat[dat>230] = 230
        for i in range(319):
            dat[i+1] = dat[i] - self.alpha*(dat[i] - dat[i+1])
        if(not self.old_dat is None):
            dat = self.old_dat - self.alpha2*(self.old_dat - dat)
        #print dat
        self.hsv[...,0] = ang*180/np.pi/2
        self.hsv[...,2] = cv2.normalize(mag,None,0,255,cv2.NORM_MINMAX)
        bgr = cv2.cvtColor(self.hsv,cv2.COLOR_HSV2BGR)
        #cv2.imshow('frame2',bgr)
        #cv2.imshow('frame',self.next)
        for i in range(320):
            self.plot_image[int(dat[i]),i] = 0
            self.plot_image[int(dat[i])+1,i] = 0
            self.plot_image[int(dat[i])+2,i] = 0

            self.plot_image[int(dat2[i]),i] = 120
            self.plot_image[int(dat2[i])+1,i] = 120
            self.plot_image[int(dat2[i])+2,i] = 120
        cmprsmsg = self.br.cv2_to_imgmsg(self.plot_image)
        self.img_pub.publish(cmprsmsg)
        cmprsmsg = self.br.cv2_to_imgmsg(self.next)
        self.roi_pub.publish(cmprsmsg)
        
        self.dat_bin = np.zeros_like(dat)
        self.dat_bin[dat > self.thresh] = 1
        diff_dat = np.diff(self.dat_bin)
        diff_dat = np.absolute(diff_dat)
        idx = np.where(diff_dat == 1)
        idx = np.concatenate(idx)

        idx = idx + 1
        idx = np.insert(idx,0,[0],axis = 0)
        idx = np.append(idx,320)
        print idx
        self.check_doors(limits_idx=idx)
        self.old_dat = dat.copy()
def main(args):
    rospy.init_node('ShowPoints', anonymous=True)
    sc = EnterDoors()
    #rospy.init_node('send_command', anonymous=True)
    rospy.spin()
if __name__ == '__main__':
    main(sys.argv)

#export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/ros/melodic/lib/parrot_arsdk
