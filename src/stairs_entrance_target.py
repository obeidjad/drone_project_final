#!/usr/bin/env python
from __future__ import print_function
import sys
import rospy
import cv2
from std_msgs.msg import String, Int32, Float32
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge, CvBridgeError
import display
import tools
import cluster
import numpy as np
import math
from activation_class import NodeActivate

class ImageProcessorStairs(NodeActivate):

  def __init__(self):
    super(ImageProcessorStairs,self).__init__("image_processor_drone_vs_stairs_round1")
    # subscribers
    self.image_sub = rospy.Subscriber("/drone_img",CompressedImage, self.callback, queue_size = 1, buff_size = 2**22)
    #self.deactivation = rospy.Subscriber("/ready_to_get_up",Int32,self.deactivate)
    # publishers
    self.image_all_lines_pub = rospy.Publisher("/image_topic_all_lines", Image,queue_size = 1)
    self.image_stairs_pub = rospy.Publisher("/image_topic_stairs_lines", Image,queue_size = 1)
    self.st_ctr_x_pub = rospy.Publisher("/lower_stairs_center_x_relative", Float32, queue_size=1)
    self.st_ctr_y_pub = rospy.Publisher("/lower_stairs_center_y_relative", Float32, queue_size=1)
    self.st_ang_pub = rospy.Publisher("/stairs_angle_rad", Float32, queue_size=1)
    self.stairs_pres = rospy.Publisher("/stairs_presence", Int32, queue_size=1)
    # initialisations
    self.bridge = CvBridge()
    self.see_stairs = 1 #juste pour ne pas crasher des le debut
    self.stairs_pres.publish(1)
    self.st_ctr_x_pub.publish(0.0)
    self.st_ctr_y_pub.publish(0.0)
    self.st_ang_pub.publish(math.pi/2)

  def noStairs(self):
    if(self.node_active == True):
      self.see_stairs = 0
      self.stairs_pres.publish(0)

  def callback(self,data):
    if(self.node_active == True):
      try:
        frame = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
        # RESHAPE
        heigth, width, depth = frame.shape
        W = 1000.
        imgscale = 1.0#0.6#W/width
        newX,newY = frame.shape[1]*imgscale, frame.shape[0]*imgscale
        newframe = cv2.resize(frame, (int(newX), int(newY)))
        grayVid = cv2.cvtColor(newframe, cv2.COLOR_BGR2GRAY)
        # DETECT SEGMENTS
        lsdVid = cv2.createLineSegmentDetector(0)
        segmentsArray = lsdVid.detect(grayVid)[0]
        # CONVERTIT LES SEGMENTS EN LIGNES
        linesArray = tools.findParamSegments(segmentsArray)
        if str(linesArray) == "no stairs" or np.size(linesArray) == 0:
          self.noStairs()
        else:
          # NE GARDE QUE LES LIGNES LES PLUS LONGUES
          lmin = 30*imgscale
          long_lines = tools.segmentsLongerThan(linesArray, lmin)
          if np.size(long_lines)==0:
            self.noStairs()
          else:
            drawn_image_long_lines = display.copy_img(grayVid)
            display.draw_segments(drawn_image_long_lines,long_lines,(255,0,0),2)
            image_long_lines = self.bridge.cv2_to_imgmsg(drawn_image_long_lines, encoding="bgr8")
            self.image_all_lines_pub.publish(image_long_lines)

            # CLUSTERISE SUR AB
            epsAB=0.05 
            min_sampleAB=2
            tab_ab, clustersAB, labels = cluster.clusterize_parallel(long_lines,epsAB,min_sampleAB)
            stairs_label = tools.findLabelStairs(tab_ab, clustersAB, labels,0.8,2.4)
            if stairs_label == "NoStairs":
              self.noStairs()
            else:
              a_st,b_st = tab_ab[stairs_label]
              stairs_angle = np.arctan2(b_st,a_st)
              cluster_AB_i = clustersAB[stairs_label]
              # CLUSTERISE SUR XY
              clusterbis,labelbis = cluster.clusterize_xy(cluster_AB_i,50*imgscale,4)#initialement minsample =2, j'ai fait ca pour eviter de se prndre les rembardes
              T=[]
              for cluster_ in clusterbis :
                taille=np.size(cluster_)
                T.append(taille)
              if T == [0] or T==[]:
                self.noStairs()
              else:
                i_max = T.index(max(T))
                print(i_max)
                stairs_lines = clusterbis[i_max]

                #LOWER STAIRS
                lowest_st_nb = 4
                lowest_stairs_lines = tools.findLowerStairs(stairs_lines, lowest_st_nb)
                low_x_moy, low_y_moy = tools.findMiddle(lowest_stairs_lines)
                x_moy, y_moy = tools.findMiddle(stairs_lines)
                
                drawn_image_stairs = display.copy_img(grayVid)
                display.draw_segments(drawn_image_stairs, stairs_lines, (255,0,0), 2)
                display.draw_segments(drawn_image_stairs, lowest_stairs_lines, (0,255,0), 3)
                display.points(drawn_image_stairs, np.array([[x_moy,y_moy]]), 5, np.array([255,0,0]),-1)
                display.points(drawn_image_stairs, np.array([[low_x_moy,low_y_moy]]), 5, np.array([0,255,0]),-1)
                display.points(drawn_image_stairs, np.array([[newX/2,newY/2]]), 3, np.array([0,0,0]),-1)
                display.draw_segments(drawn_image_stairs, np.array([[newX/2,0,newX/2,newY],[0,0.8*newY,newX,0.8*newY]]), (0,0,0), 1)
                display.draw_segments(drawn_image_stairs, np.array([[low_x_moy,0,low_x_moy,newY],[0,low_y_moy,newX,low_y_moy]]), (0,0,255), 2)
                image_stairs = self.bridge.cv2_to_imgmsg(drawn_image_stairs, encoding="bgr8")

                self.image_stairs_pub.publish(image_stairs)
                self.st_ctr_x_pub.publish(low_x_moy/newX - 0.5)
                self.st_ctr_y_pub.publish(low_y_moy/newY - 0.5)
                self.st_ang_pub.publish(stairs_angle)
                self.stairs_pres.publish(1)

      except CvBridgeError as e:
        pass

def main(args):
  rospy.init_node('image_processor_drone_vs_stairs_round1', anonymous=True)
  ia = ImageProcessorStairs()

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
