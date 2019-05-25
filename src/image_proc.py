#!/usr/bin/env python
"""
This node will read image from topic "/drone/front/compressed"
detect the centroid and will publish the centorid to the topic /centroids
and publsih the difference between slopes to the topic /sDiffs
The published data is filtered using a mean filter 
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
from itertools import combinations, chain
import datetime
from activation_class import NodeActivate,returnResp

class image_convert(NodeActivate):
    def __init__(self):
        super(image_convert,self).__init__("detectVanish")
        self.pub = rospy.Publisher("/centroids", Float32, queue_size=1)
        self.pubDiff = rospy.Publisher('/sDiffs',Float32,queue_size=1)
        self.pubForBag = rospy.Publisher('/imgForBag/front/compressed', CompressedImage, queue_size=1)
        #self.subscriber = rospy.Subscriber("/bebop/image_raw",Image, self.callback)
        self.subscriber = rospy.Subscriber("/bebop/image_raw",Image, self.callback)
        self.lsd = cv2.createLineSegmentDetector(0)
        self.real_centr = np.array([160,120])
        self.old_centr = np.array([160,120])
        self.real_sDiff = 0
        self.old_sDiff = 0
        self.length = 50
        self.br = CvBridge()
        self.alfa = 0.65
        self.alfa_sDiff = 0.1
        self.avLeftGlb = 0
        self.avRightGlb = 0
        self.arrSDiff = np.array([])
        self.filteredArrSDiff = np.array([])



    def findGoodLines(self,allLines,length):
        allLines = np.reshape(allLines,(len(allLines),4))
        allLines[:,[1, 2]] = allLines[:,[2, 1]]
        diffLines = np.diff(allLines)
        diffLines = np.delete(diffLines,1,1)
        lengths = np.linalg.norm(diffLines,axis = 1)
        and_arr = np.logical_and(np.absolute(diffLines[:,0]) > 10,np.absolute(diffLines[:,1]) > 10)
        and_arr = np.logical_and(and_arr,lengths > length)
        #print final_and
        goodLines = allLines[and_arr]
        goodDiffs = diffLines[and_arr]
        #print goodLines[:,2]>180
        #on Y1 (60<Y1<180)
        and_arr2 = np.logical_and(goodLines[:,2]>60 , goodLines[:,2]<180)
        #on Y2 (60<Y2<180)
        and_arr3 = np.logical_and(goodLines[:,3]>60 , goodLines[:,3]<180)
        final_and = np.logical_not(np.logical_and(and_arr2,and_arr3))
        goodLines = goodLines[final_and]
        goodDiffs = goodDiffs[final_and]
        goodLines[:,[1, 2]] = goodLines[:,[2, 1]]
        return goodLines,goodDiffs

    def findCartMat(self,Points,Vects):
        #suppose points are given in [Xa,Ya] and the vectors are given in [Xu,Yu] we have that c = YaXu - XaYu = diff(XaYu,YaXu)
        #so we need to swap the Vects Matrix
        Vects[:,[0,1]] = Vects[:,[1,0]]
        cmat = np.multiply(Points,Vects)
        cmat = np.diff(cmat)
        return cmat

    def findIntersec(self,abval,cval):
        abval[:,1] = np.multiply(-1,abval[:,1])
        cval = np.multiply(-1,cval)
        nbLines = cval.shape[0]
        index = np.fromiter(chain.from_iterable(combinations(range(nbLines), 2)), int)
        index = index.reshape(-1,2)
        A = abval[index,:]
        B = cval[index,:]
        interSec =  np.linalg.solve(A, B).reshape(-1,2)
        and_arr1 = np.logical_and(interSec[:,0] > 0,interSec[:,1] > 0)
        and_arr2 = np.logical_and(interSec[:,1] < 320,interSec[:,0] < 240)
        return interSec[np.logical_and(and_arr1,and_arr2)]

    def findCentroid(self,intersec):
        if intersec.shape[0] == 0:
            return 0,0
        if intersec.shape[0] == 1:
            return 1,intersec[0]

        db = DBSCAN(eps=20, min_samples=2).fit(intersec)
        db_labels = db.labels_
        labels, counts = np.unique(db_labels[db_labels>=0], return_counts=True)
        maxlabel = labels[np.argsort(-counts)[:1]]
        goodPoints = intersec[db_labels == maxlabel]
        centroid = np.mean(goodPoints,axis=0) 
        return 1,centroid

    def findAverageSlopes(self,points,abval,cval,zone):
        #zone = 3 : left
        #zone = 4 : right
        if zone == 4:
            or_x = np.logical_and(points[:,[0]] > self.real_centr[0],points[:,[2]] > self.real_centr[0])
            or_y = np.logical_and(points[:,[1]] > self.real_centr[1],points[:,[3]] > self.real_centr[1])
        else:
            or_x = np.logical_and(points[:,[0]] < self.real_centr[0],points[:,[2]] < self.real_centr[0])
            or_y = np.logical_and(points[:,[1]] > self.real_centr[1],points[:,[3]] > self.real_centr[1])
        and_arr = np.logical_and(or_x,or_y)
        and_arr = and_arr.reshape(-1)
        myabval = abval[and_arr]
        mycval = cval[and_arr]
        testMat = np.multiply(myabval,self.real_centr)
        testMat = np.sum(testMat,axis=1)
        testMat = testMat[np.newaxis]
        testMat = testMat.transpose()
              
              
        errMat = np.absolute(testMat + mycval)
              
        cond2 = (errMat < 500).reshape(-1)
        good_ab = myabval[cond2]
        slopes = np.divide(good_ab[:,0],good_ab[:,1])
        return np.mean(slopes)

    def getVanishingPoint(self,in_image):
        sDiff = 0
        self.gray = cv2.cvtColor(in_image, cv2.COLOR_BGR2GRAY)
        self.gray = cv2.resize(self.gray, (320,240), interpolation = cv2.INTER_AREA) 
        lines = self.lsd.detect(self.gray)[0]
        try:
            goodLines,directVect = self.findGoodLines(allLines=lines,length=self.length)

            #goodlines in (x1,y1,x2,y2)
            #directVect is (-b,a)
            #the equation of this line is ax+by+c = 0

            Cmat = self.findCartMat(goodLines[:,[0,1]],directVect)

            #here directVect is (a,-b) 

            frame = self.lsd.drawSegments(self.gray,goodLines)
            intersecPoints = self.findIntersec(directVect,Cmat)

            #here directVect is (a,b)
            #here Cmat = -c

            for point in intersecPoints:
                cv2.circle(frame,(int(point[0]),int(point[1])),3,(0,0,0),-1)
            res,centr = self.findCentroid(intersecPoints)
            if(res == 1 and type(centr[0]) == np.float32):
                if(centr[0] < 320 and centr[1] < 240):
                    self.real_centr = centr
                    self.real_centr = self.old_centr - self.alfa*(self.old_centr - self.real_centr)
                    self.old_centr = self.real_centr
            cv2.circle(frame,(int(self.real_centr[0]),int(self.real_centr[1])),8,(0,255,0),-1)
            self.old_centr = self.real_centr
            avLeft = self.findAverageSlopes(points=goodLines,abval=directVect,cval=Cmat,zone=3)
            if (not math.isnan(avLeft)):
                self.avLeftGlb =  avLeft
            avRight = self.findAverageSlopes(points=goodLines,abval=directVect,cval=Cmat,zone=4)
            if (not math.isnan(avRight)):
                self.avRightGlb =  avRight
            #print "avLeft"
            #print avLeft
            #print "avRight"
            #print avRight
            sDiff = self.avLeftGlb + self.avRightGlb
            print "sDiff"
            print sDiff
            self.arrSDiff = np.append(self.arrSDiff,sDiff)
            
            self.real_sDiff = sDiff
            self.real_sDiff = self.old_sDiff - self.alfa_sDiff*(self.old_sDiff - self.real_sDiff)
            self.filteredArrSDiff = np.append(self.filteredArrSDiff,self.real_sDiff)
            self.old_sDiff = self.real_sDiff
        except Exception as e:
            print e 
            print datetime.datetime.now()
        return frame,self.real_centr[0],self.real_sDiff

    def sendCentroid(self,dat):
        self.pub.publish(dat)
    def sendsDiff(self,dat):
        self.pubDiff.publish(dat)
    def saveData(self):
        np.savetxt("/home/jad/droneStats/sdiffsdat.csv", self.arrSDiff, delimiter=",")
        np.savetxt("/home/jad/droneStats/filteredArrSDiff.csv", self.filteredArrSDiff, delimiter=",")


    def callback(self,ros_data):
        #msg = ""
        #if(self.node_active == 0):
        #    return
        cv2_img = self.br.imgmsg_to_cv2(ros_data)
        #np_arr = np.fromstring(ros_data.data, np.uint8)
        #image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        image,cx,sDiff = self.getVanishingPoint(cv2_img)
        #msg = str(cx) + "_" + str(sDiff)
        cv2.imshow('cv_img', image)
        if not rospy.is_shutdown():
            self.sendCentroid(cx)
            self.sendsDiff(sDiff)
            cmprsmsg = self.br.cv2_to_compressed_imgmsg(image)
            self.pubForBag.publish(cmprsmsg)
        cv2.waitKey(10)

def main(args):
    rospy.init_node('image_feature', anonymous=True)
    ic = image_convert()
    rospy.spin()
    #ic.saveData()
    cv2.destroyAllWindows()



if __name__ == '__main__':
    main(sys.argv)
