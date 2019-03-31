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
        self.count = 0
        self.color = np.random.randint(0,255,(100,3))
    def transform_image(self,ros_data):
        cv2_image = self.br.imgmsg_to_cv2(ros_data)
        cv2_image = cv2.resize(cv2_image, (640,280), interpolation = cv2.INTER_AREA) 
        cv2.rectangle(mask2, (0, 240), (640, 480), (255, 255, 255), -1)
        self.current_image = cv2_image
        if(self.old_image is None):
            self.old_image = cv2_image
        self.draw_and_dispaly()
    def draw_and_dispaly(self):
        if(self.count == 0):
            p0 = cv2.goodFeaturesToTrack(self.old_image, mask = None, **feature_params)
        self.count = self.count + 1
        frame = self.current_image
        old_gray = cv2.cvtColor(self.old_image, cv2.COLOR_BGR2GRAY)
        frame_gray = cv2.cvtColor(self.current_image, cv2.COLOR_BGR2GRAY)
        p1, st, err = cv2.calcOpticalFlowPyrLK(old_gray, frame_gray, p0, None, **lk_params)
        good_new = p1[st==1]
        good_old = p0[st==1]
        print (good_new - good_old)[0,:][0]
        for i,(new,old) in enumerate(zip(good_new,good_old)):
            a,b = new.ravel()
            c,d = old.ravel()
            mask = cv2.line(mask, (a,b),(c,d), self.color[i].tolist(), 2)
            frame = cv2.circle(frame,(a,b),5,self.color[i].tolist(),-1)
        img = cv2.add(frame,mask)
        if(self.count == 50):
            self.count = 0
def main(args):
    rospy.init_node('ShowPoints', anonymous=True)
    sc = EnterDoors()
    #rospy.init_node('send_command', anonymous=True)
    rospy.spin()
if __name__ == '__main__':
    main(sys.argv)


