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

class ImageProcessorStairsUp(NodeActivate):

  def __init__(self):
    super(ImageProcessorStairsUp,self).__init__("image_processor_drone_vs_stairs_round2")
    # subscribers
    #self.ready = rospy.Subscriber("/ready_to_get_up",Int32,self.activate)
    self.image_sub = rospy.Subscriber("/drone_img",CompressedImage, self.callback, queue_size = 1, buff_size = 2**22)
    # publishers
    self.image_pub_all = rospy.Publisher("image_topic_all_lines", Image,queue_size = 1)
    self.image_pub_stairs = rospy.Publisher("/image_topic_stairs_lines", Image,queue_size = 1)
    self.image_pub_merged = rospy.Publisher("image_topic_merged_lines", Image,queue_size = 1)
    self.image_pub_trpz_lin = rospy.Publisher("image_topic_trapeze_lin", Image,queue_size = 1)
    self.image_pub_trpz_svr = rospy.Publisher("image_topic_trapeze_svr", Image,queue_size = 1)

    self.st_ctr_x_pub = rospy.Publisher("/stairs_center_x_relative", Float32, queue_size=1)
    self.st_ctr_y_pub = rospy.Publisher("/stairs_center_y_relative", Float32, queue_size=1)
    self.low_st_y_pub = rospy.Publisher("/lowest_stair_y_relative", Float32, queue_size=1)
    self.st_ang_pub = rospy.Publisher("/stairs_angle_rad", Float32, queue_size=1)
    self.stairs_pres = rospy.Publisher("/stairs_presence", Int32, queue_size=1)

    # trois estimateurs du centre des escaliers:
    self.current_st_ctr_x = 0
    self.old_st_ctr_x = 0
    self.current_st_ctr_x_trpz_lin = 0
    self.old_st_ctr_x_trpz_lin = 0
    self.current_st_ctr_x_trpz_svr = 0
    self.old_st_ctr_x_trpz_svr = 0
    self.alpha = 0.2
    # enregistrement de ces estimateurs et de leur correction (lissage)
    self.center_stairs = np.array([])
    self.corr_center_stairs = np.array([])
    self.center_trpz_lin = np.array([])
    self.corr_center_trpz_lin = np.array([])
    self.center_trpz_svr = np.array([])
    self.corr_center_trpz_svr = np.array([])
    # initialisations:
    self.bridge = CvBridge()
    self.see_stairs = 1 #juste pour ne pas crasher des le debut
    self.stairs_pres.publish(1)
    self.st_ctr_x_pub.publish(0.0)
    self.st_ctr_y_pub.publish(0.0)
    self.low_st_y_pub.publish(0.0)
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
        imgscale = 1.0# W/width
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
            # CLUSTERISE SUR AB
            epsAB = 0.05
            min_sampleAB = 2
            tab_ab, clustersAB, labels = cluster.clusterize_parallel(long_lines,epsAB,min_sampleAB)
            stairs_label = tools.findLabelStairs(tab_ab, clustersAB, labels,1.4,1.7)
            if stairs_label == "NoStairs":
              self.noStairs()
            else:
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
                stairs_lines = clusterbis[i_max]

                #LOWER STAIRS
                lowest_st_nb = 4
                lowest_stairs_lines = tools.findLowerStairs(stairs_lines, lowest_st_nb)
                a_st,b_st = np.mean(lowest_stairs_lines[:,5]),np.mean(lowest_stairs_lines[:,6])
                stairs_angle = np.arctan2(b_st,a_st)
                low_x_moy, low_y_moy = tools.findMiddle(lowest_stairs_lines)
                
                # MERGE LINES
                epsC = 5.0
                min_sampleC = 2
                merged_lines = np.empty((0, 8), dtype=np.float32)
                mean_c, i_list, clustersABC, labels_c = cluster.clusterize_offsets(stairs_lines,epsC,min_sampleC)#clusterizeoffsets pas opti
                new_lines = tools.mergeClusterSerieAfterC(i_list, clustersABC, labels_c)
                merged_lines = np.concatenate((merged_lines,new_lines),axis = 0)
                #L = np.mean(merged_lines[:,4])
                #cond = np.greater(merged_lines[:,4],L)
                #merged_lines = merged_lines[cond]
                if np.size(merged_lines)<16: # 2 lines
                  self.noStairs()

                else:
                  # SELECTION DE LA DERNIERE MARCHE
                  Ymax = tools.findLowestStair(stairs_lines)

                  #ESTIMATION DU CENTRE PAR LES MILIEUX DES SEGMENTS FUSIONNES
                  x_moy, y_moy = tools.findMiddle(merged_lines)
                  self.current_st_ctr_x = self.old_st_ctr_x - self.alpha*(self.old_st_ctr_x - x_moy)
                  self.old_st_ctr_x = self.current_st_ctr_x

                  self.center_stairs = np.append(self.center_stairs, x_moy)
                  self.corr_center_stairs = np.append(self.corr_center_stairs, self.current_st_ctr_x)

                  #ESTIMATION DU CENTRE PAR TRAPEZE DES MOINDRES CARRES
                  trapeze_lines, leftPTS, rightPTS, mid_line = tools.findTrapeze(merged_lines,'lin')
                  x_moy_trpz_lin, y_moy_trpz_lin = tools.findMiddle(trapeze_lines)

                  self.current_st_ctr_x_trpz_lin = self.old_st_ctr_x_trpz_lin - self.alpha*(self.old_st_ctr_x_trpz_lin - x_moy_trpz_lin)
                  self.old_st_ctr_x_trpz_lin = self.current_st_ctr_x_trpz_lin

                  self.center_trpz_lin = np.append(self.center_trpz_lin, x_moy_trpz_lin)
                  self.corr_center_trpz_lin = np.append(self.corr_center_trpz_lin, self.current_st_ctr_x_trpz_lin)

                  #ESTIMATION DU CENTRE PAR TRAPEZE DES MOINDRES CARRES
                  trapeze_lines_svr, leftPTS, rightPTS, mid_line = tools.findTrapeze(merged_lines,'svr')
                  x_moy_trpz_svr, y_moy_trpz_svr = tools.findMiddle(trapeze_lines_svr)
                  self.current_st_ctr_x_trpz_svr = self.old_st_ctr_x_trpz_svr - self.alpha*(self.old_st_ctr_x_trpz_svr - x_moy_trpz_svr)
                  self.old_st_ctr_x_trpz_svr = self.current_st_ctr_x_trpz_svr

                  self.center_trpz_svr = np.append(self.center_trpz_svr, x_moy_trpz_svr)
                  self.corr_center_trpz_svr = np.append(self.corr_center_trpz_svr, self.current_st_ctr_x_trpz_svr)

                  drawn_image_all_lines = display.copy_img(grayVid)
                  display.draw_segments(drawn_image_all_lines,long_lines,(255,0,0),2)
                  image_all_lines = self.bridge.cv2_to_imgmsg(drawn_image_all_lines, encoding="bgr8")

                  drawn_image_stairs = display.copy_img(grayVid)
                  display.draw_segments(drawn_image_stairs, stairs_lines, (255,0,0), 2)
                  display.draw_segments(drawn_image_stairs, lowest_stairs_lines, (0,0,255), 3)
                  display.points(drawn_image_stairs, np.array([[x_moy,y_moy]]), 5, np.array([0,0,0]),-1)#affiche le centre de l'escalier en blanc
                  display.points(drawn_image_stairs, np.array([[newX/2,newY/2]]), 3, np.array([0,0,0]),-1)
                  image_stairs = self.bridge.cv2_to_imgmsg(drawn_image_stairs, encoding="bgr8")

                  drawn_image_merged = display.copy_img(grayVid)
                  display.draw_segments(drawn_image_merged,merged_lines,(255,0,0),2)
                  display.points(drawn_image_merged, np.array([[x_moy_trpz_svr,y_moy_trpz_svr]]), 5, np.array([100,100,0]),-1)
                  display.points(drawn_image_merged, np.array([[x_moy_trpz_lin,y_moy_trpz_lin]]), 5, np.array([0,100,100]),-1)
                  display.points(drawn_image_merged, np.array([[x_moy,y_moy]]), 5, np.array([0,0,0]),-1)#affiche le centre de l'escalier en blanc
                  display.draw_segments(drawn_image_merged, np.array([[newX/2,0,newX/2,newY],[0,0.8*newY,newX,0.8*newY]]), (0,0,0), 2)
                  display.draw_segments(drawn_image_merged, np.array([[x_moy,0,x_moy,newY]]), (0,0,255), 2)
                  display.draw_segments(drawn_image_merged, np.array([[0,Ymax,newX,Ymax]]), (0,150,200), 2)
                  image_merged = self.bridge.cv2_to_imgmsg(drawn_image_merged, encoding="bgr8")

                  drawn_image_trpz_lin = display.copy_img(grayVid)
                  display.draw_segments(drawn_image_trpz_lin,trapeze_lines,(255,0,0),2)
                  display.draw_segments(drawn_image_trpz_lin,mid_line,(255,255,0),4)
                  display.points(drawn_image_trpz_lin, leftPTS, 2, np.array([0,255,0]),5)
                  display.points(drawn_image_trpz_lin, rightPTS, 2, np.array([0,0,255]),5)
                  display.points(drawn_image_trpz_lin, np.array([[x_moy_trpz_lin,y_moy_trpz_lin]]), 5, np.array([0,100,100]),-1)
                  display.points(drawn_image_trpz_lin, np.array([[x_moy,y_moy]]), 5, np.array([0,0,0]),-1)#affiche le centre de l'escalier en blanc
                  display.points(drawn_image_trpz_lin, np.array([[newX/2,newY/2]]), 3, np.array([0,0,0]),-1)
                  image_trpz_lin = self.bridge.cv2_to_imgmsg(drawn_image_trpz_lin, encoding="bgr8")

                  drawn_image_trpz_svr = display.copy_img(grayVid)
                  display.draw_segments(drawn_image_trpz_svr,trapeze_lines_svr,(255,0,0),2)
                  display.draw_segments(drawn_image_trpz_svr,mid_line,(255,255,0),4)
                  display.points(drawn_image_trpz_svr, leftPTS, 2, np.array([0,255,0]),5)
                  display.points(drawn_image_trpz_svr, rightPTS, 2, np.array([0,0,255]),5)
                  display.points(drawn_image_trpz_svr, np.array([[x_moy_trpz_svr,y_moy_trpz_svr]]), 5, np.array([100,100,0]),-1)
                  display.points(drawn_image_trpz_svr, np.array([[newX/2,newY/2]]), 3, np.array([0,0,0]),-1)
                  image_trpz_svr = self.bridge.cv2_to_imgmsg(drawn_image_trpz_svr, encoding="bgr8")

                  self.image_pub_all.publish(image_all_lines)
                  self.image_pub_stairs.publish(image_stairs)
                  self.image_pub_merged.publish(image_merged)
                  self.image_pub_trpz_lin.publish(image_trpz_lin)
                  self.image_pub_trpz_svr.publish(image_trpz_svr)
                  self.st_ctr_x_pub.publish(x_moy/newX - 0.5)
                  self.st_ctr_y_pub.publish(y_moy/newY - 0.5)
                  self.low_st_y_pub.publish(Ymax/newY - 0.5)
                  self.st_ang_pub.publish(stairs_angle)
                  #self.stairs_data_pub.publish(str(stairs_angle)+"_"+str(meanX) +"_"+str(newX/2)+"_"+str(meanY) +"_"+str(newY/2)+"_"+str(Ymax))#permet de connaitre le centre
                  self.stairs_pres.publish(1)

      except CvBridgeError as e:
        pass

  def saveVal(self):
    np.savetxt("/home/qsdfgh/projetDrones2A/centre_esc.csv", self.center_stairs, delimiter=",")
    np.savetxt("/home/qsdfgh/projetDrones2A/centre_esc_corr.csv", self.corr_center_stairs, delimiter=",")
    np.savetxt("/home/qsdfgh/projetDrones2A/centre_trpz_lin.csv", self.center_trpz_lin, delimiter=",")
    np.savetxt("/home/qsdfgh/projetDrones2A/centre_trpz_lin_corr.csv", self.corr_center_trpz_lin, delimiter=",")
    np.savetxt("/home/qsdfgh/projetDrones2A/centre_trpz_svr.csv", self.center_trpz_svr, delimiter=",")
    np.savetxt("/home/qsdfgh/projetDrones2A/centre_trpz_svr_corr.csv", self.corr_center_trpz_svr, delimiter=",")

def main(args):
  rospy.init_node('image_processor_drone_vs_stairs_round2', anonymous=True)
  ia = ImageProcessorStairsUp()
  try:
    rospy.spin()
    ia.saveVal()
  except KeyboardInterrupt:
    ia.saveVal()
    print("Shutting down")
  ia.saveVal()
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)