#!/usr/bin/env python

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

from msgs_perception.msg import BoundingBoxArray
from msgs_perception.msg import BoundingBox

import image_geometry
from sensor_msgs.msg import Image
from image_geometry import PinholeCameraModel
import tf

import os
from sklearn.svm import SVC
from skimage import feature
from sklearn.externals import joblib

from sensor_msgs.msg import CameraInfo

import ros_numpy

class Detector:

  def __init__(self):

    self.bbox_tfsign_pub = rospy.Publisher("/carina/sensor/tfsigns_box", BoundingBoxArray, queue_size = 1)

    self.image_sub =  rospy.Subscriber("/carina/sensor/camera/left/image_raw",Image,self.callback_image_left)
    # self.info_sub_l = rospy.Subscriber("/carina/sensor/camera/left/camera_info", CameraInfo, self.callback_info_left, queue_size=1)

    self.bridge = CvBridge()
    self.camInfo = CameraInfo()

    self.model = PinholeCameraModel()

    if os.path.isfile("clf_google/clf_svc_hog.pkl"):
      print("[INFO] loading classifier: SVC trained on HoG features...")
      self.svc = joblib.load("clf_google/clf_svc_hog.pkl")
      print("[INFO] Classifer is loaded as instance ::svc::")
    else:
      print("[INFO] pre-trained classifier not found.")


  # def callback_info_left(self,camerainfo):
  #   self.model.fromCameraInfo(camerainfo)


  def callback_image_left(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_image.shape
    cv_image_gray = cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY)
    cv_image_gray = cv2.medianBlur(cv_image_gray,5)
    circles = cv2.HoughCircles(cv_image_gray,cv2.HOUGH_GRADIENT,4,700, param1=100,param2=20,minRadius=10,maxRadius=80)
    circles = np.uint16(np.around(circles))

    odom_bbox_tfsign = BoundingBoxArray()

    odom_bbox_tfsign.header.frame_id = data.header.frame_id#
    odom_bbox_tfsign.header.stamp = rospy.Time.now()
    odom_bbox_tfsign.objects = []

    for x,y,r in circles[0,:]:
      if(np.float(x)-np.float(r) <= 0.0):
        x1=0
      else:
        x1 = x-r
      if(np.float(y)-np.float(r) <= 0.0):
        y1=0
      else:
        y1 = y-r
      if(x+r >= cv_image.shape[1]-1):
        x2=cv_image.shape[1]-1
      else:
        x2 = x+r
      if(y+r >= cv_image.shape[0]-1):
        y2=cv_image.shape[0]-1
      else:
        y2 = y+r

      cv2.rectangle(cv_image,(x1,y1),(x2,y2),(255,255,255),2)
      cv2.imshow("classe ",cv_image )
      cv2.waitKey(5)
      crop_img = cv_image[y1:y2, x1:x2]

      if(crop_img.size>4):

        gray = cv2.cvtColor(crop_img , cv2.COLOR_BGR2GRAY)
   
        if(gray.shape is not None):

          grayim = cv2.resize(gray,(40,40))
          (t1_feat, hogImage) = feature.hog(grayim, orientations=9, pixels_per_cell=(4, 4),
            block_norm='L2-Hys',cells_per_block=(2, 2), transform_sqrt=True, visualize=True)
          t1_predict = self.svc.predict(t1_feat.reshape(1, -1))

          bbox_loc = BoundingBox()

          if(t1_predict[0]== 1.0):
            velmaxlabel = 30
            color = (255,0,0)

          elif(t1_predict[0]== 2.0):
            velmaxlabel = 60
            color =(0,255,0)

          elif(t1_predict[0]== 3.0):
            velmaxlabel = 90
            color = (0,0,255)
          else:
            continue

          bbox_loc.classe.data = str(velmaxlabel)#class placa

          bbox_loc.p1.x = x1
          bbox_loc.p1.y = y1
          bbox_loc.p1.z = 0

          bbox_loc.p2.x = x1
          bbox_loc.p2.y = y2
          bbox_loc.p2.z = 0

          bbox_loc.p3.x = x2
          bbox_loc.p3.y = y1
          bbox_loc.p3.z = 0

          bbox_loc.p4.x = x2
          bbox_loc.p4.y = y2
          bbox_loc.p4.z = 0

          bbox_loc.probability = 1.0


          odom_bbox_tfsign.objects.append(bbox_loc)
          

          cv2.rectangle(cv_image,(x1,y1),(x2,y2),color,2)

    if(len(odom_bbox_tfsign.objects)!=0):
      self.bbox_tfsign_pub.publish(odom_bbox_tfsign)



def main(args):
  
  rospy.init_node('detector', anonymous=True)
  ic = Detector()
  # try:
  rospy.spin()
  # except KeyboardInterrupt:
  #   print("Shutting down")
  # cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
