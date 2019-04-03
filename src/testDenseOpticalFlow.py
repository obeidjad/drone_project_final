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
from std_msgs.msg import String
import sys
from sklearn.cluster import DBSCAN
from sklearn import metrics
import math
from cv_bridge import CvBridge, CvBridgeError

class EnterDoors:
    def __init__(self):
        self.img_read = rospy.Subscriber("/image_in",Image,self.transform_image)
        self.current_image = None
        self.old_image = None
        self.br = CvBridge()
        self.count = 0
        self.color = np.random.randint(0,255,(100,3))
        self.p0 = 0
        self.feature_params = dict( maxCorners = 100,qualityLevel = 0.3,minDistance = 7,blockSize = 7 )
        self.lk_params = dict( winSize  = (15,15),maxLevel = 2,criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
        # Parameters for lucas kanade optical flow
    def transform_image(self,ros_data):
        cv2_image = self.br.imgmsg_to_cv2(ros_data)
        self.current_image = cv2_image
        #cv2_image = cv2.resize(cv2_image, (640,480), interpolation = cv2.INTER_AREA) 
        cv2.rectangle(self.current_image, (0, 220), (900, 260), (0, 0, 0), -1)
        if(self.old_image is None):
            self.old_image = cv2_image
            old_gray = cv2.cvtColor(self.old_image, cv2.COLOR_BGR2GRAY) 
            self.p0 = cv2.goodFeaturesToTrack(old_gray, mask = None, **self.feature_params)
        else :
            img = self.draw_and_dispaly()
            #self.p0 = cv2.goodFeaturesToTrack(old_gray, mask = None, **feature_params)
            cv2.imshow('img',img)
            cv2.waitKey(10)

    def draw_and_dispaly(self):
        old_gray = cv2.cvtColor(self.old_image, cv2.COLOR_BGR2GRAY)
        frame_gray = cv2.cvtColor(self.current_image, cv2.COLOR_BGR2GRAY)
        mask = np.zeros_like(self.old_image)
        self.count = self.count + 1
        frame = self.current_image
        p1, st, err = cv2.calcOpticalFlowPyrLK(old_gray, frame_gray, self.p0, None, **self.lk_params)
        good_new = p1[st==1]
        good_old = self.p0[st==1]
        arr = np.absolute((good_new - good_old)[0,:][0])
        print np.amax(arr)
        for i,(new,old) in enumerate(zip(good_new,good_old)):
            a,b = new.ravel()
            c,d = old.ravel()
            mask = cv2.line(mask, (a,b),(c,d), self.color[i].tolist(), 2)
            frame = cv2.circle(frame,(a,b),5,self.color[i].tolist(),-1)
        img = cv2.add(frame,mask)
        cv2.imshow('img',img)
        self.old_image = frame.copy()
        if(self.count == 50):
            self.count = 0
            self.p0 = cv2.goodFeaturesToTrack(old_gray, mask = None, **self.feature_params)
        else:
            self.p0 = good_new.reshape(-1,1,2)
        return img
def main(args):
    rospy.init_node('ShowPoints', anonymous=True)
    sc = EnterDoors()
    #rospy.init_node('send_command', anonymous=True)
    rospy.spin()
if __name__ == '__main__':
    main(sys.argv)


