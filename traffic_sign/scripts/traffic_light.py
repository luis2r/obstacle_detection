#!/usr/bin/env python

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Header
from sensor_msgs import point_cloud2
from cv_bridge import CvBridge, CvBridgeError

from msgs_perception.msg import ObstacleArray
from msgs_perception.msg import Obstacle


# import image_geometry, tf, cv2
from image_geometry import PinholeCameraModel
import tf

import os
from sklearn.svm import SVC
from skimage import feature
from sklearn.externals import joblib

from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PointStamped

import ros_numpy

class Detector:

  def __init__(self):
    self.image_pub = rospy.Publisher("/carina/sensor/camera/left/light_image_detection",Image, queue_size=1)
    self.odom_objs_tflight_pub = rospy.Publisher("/carina/sensor/tflight_odom", ObstacleArray, queue_size = 1)

    self.image_sub =  rospy.Subscriber("/carina/sensor/camera/left/image_raw",Image,self.callback_image_left)
    self.info_sub_l = rospy.Subscriber("/carina/sensor/camera/left/camera_info", CameraInfo, self.callback_info_left, queue_size=1)
    self.objs_sub =    rospy.Subscriber("/carina/perception/lidar/obstacles_array", ObstacleArray, self.obstacles_callback, queue_size=1)

    self.bridge = CvBridge()
    self.camInfo = CameraInfo()
    self.tf_l = tf.TransformListener()

    self.obstacles_loc = ObstacleArray()

    self.odom_objs_tflight = ObstacleArray()
    self.odom_objs_tflight.obstacle = []
    self.n_tflight = 0

    self.points=np.zeros((1,3))

    self.model = PinholeCameraModel()


    if os.path.isfile("clf/clf_svc_hog.pkl"):
      print("[INFO] loading classifier: SVC trained on HoG features...")
      self.svc = joblib.load("clf/clf_svc_hog.pkl")
      print("[INFO] Classifer is loaded as instance ::svc::")

    else:
      print("[INFO] pre-trained classifier not found.")




  def obstacles_callback(self,a_obstacles):
    self.obstacles_loc=a_obstacles



  def callback_info_left(self,camerainfo):
    self.model.fromCameraInfo(camerainfo)


  def callback_image_left(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_image.shape
    cv_image_gray = cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY)
    cv_image_gray = cv2.medianBlur(cv_image_gray,5)
    circles = cv2.HoughCircles(cv_image_gray,cv2.HOUGH_GRADIENT,5,500, param1=600,param2=20,minRadius=7,maxRadius=30)
    # circles = cv2.HoughCircles(cv_image_gray,cv2.HOUGH_GRADIENT,4,600, param1=600,param2=100,minRadius=5,maxRadius=35)
    try:
    	circles = np.uint16(np.around(circles))
    except Exception as e:
    	print('excetion: ',e)
    	pass
    else:
	    for x,y,r in circles[0,:]:
	      if(np.float(x)-np.float(r)*4 <= 0.0):
	        x1=1
	      else:
	        x1 = x-r*4
	      if(np.float(y)-np.float(r)*6 <= 0.0):
	        y1=1
	      else:
	        y1 = y-r*6
	      if(x+r*4 >= cv_image.shape[1]-1):
	        x2=cv_image.shape[1]-1
	      else:
	        x2 = x+r*4
	      if(y+r*6 >= cv_image.shape[0]-1):
	        y2=cv_image.shape[0]-1
	      else:
	        y2 = y+r*8

	      cv2.rectangle(cv_image,(x1,y1),(x2,y2),(255,255,255),2)
	      # crop_img = cv_image[y-r-5:y+r+5, x-r-5:x+r+5]    #####################posible warning################
	      crop_img = cv_image[y1:y2, x1:x2]
	      # cv2.imwrite('/media/luis/data/datasets_traffic_sign/Carla_dataset/ligths_bag/'+str(data.header.stamp.secs)+str(data.header.stamp.nsecs)+'.png', crop_img)

	      if(crop_img.size>4):

	        gray = cv2.cvtColor(crop_img , cv2.COLOR_BGR2GRAY)
	   
	        if(gray.shape is not None):

	          grayim = cv2.resize(gray,(40,40))
	          (t1_feat, hogImage) = feature.hog(grayim, orientations=9, pixels_per_cell=(4, 4),
	            block_norm='L2-Hys',cells_per_block=(2, 2), transform_sqrt=True, visualize=True)
	          t1_predict = self.svc.predict(t1_feat.reshape(1, -1))
	          
	          if(t1_predict[0]== 3.0):
	            velmaxlabel = 3
	            color = (255,255,255)
	            continue

	          elif(t1_predict[0]== 0.0):
	            velmaxlabel = 0
	            color = (0,0,255)

	          elif(t1_predict[0]== 1.0):
	            velmaxlabel = 1
	            color =(0,255,0)

	          elif(t1_predict[0]== 2.0):
	            velmaxlabel = 2
	            color = (0,255,255)
	          else:
	            velmaxlabel = -1
	            color = (0,0,0)

	          cv2.rectangle(cv_image,(x1,y1),(x2,y2),color,2)
	          font = cv2.FONT_HERSHEY_SIMPLEX
	          cv2.putText(cv_image, str(velmaxlabel),(x1,y1), font, 1,(255,255,255),2,cv2.LINE_AA)


              for obstacle in self.obstacles_loc.obstacle:
	            obstacle_local = []
	            if( obstacle.pose.position.x>0.0 and obstacle.pose.position.x < 30 and obstacle.scale.x<0.80 and obstacle.scale.y<0.80 and obstacle.scale.x*obstacle.scale.y<0.50):
	              try:
	                (trans,rot) = self.tf_l.lookupTransform('stereo', 'velodyne', rospy.Time(0))
	              except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
	                print('tf error')

	              pc_point=PointStamped()
	              pc_point.header.frame_id = "velodyne"
	              pc_point.header.stamp =rospy.Time(0)
	              pc_point.point=obstacle.pose.position
	              pt=self.tf_l.transformPoint("stereo",pc_point)
	              


	              p_2d =  self.model.project3dToPixel((pt.point.x,pt.point.y,pt.point.z))


	              if( p_2d[0]>x1-30 and p_2d[0]<x2+30 ):
	                # cv2.rectangle(cv_image,(np.int(p_2d[0])-5,0),(np.int(p_2d[0]),np.int(p_2d[1])+5),(255,0,0),2)

	                try:
	                  (trans_odom,rot_odom) = self.tf_l.lookupTransform('odom', 'velodyne', rospy.Time(0))
	                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
	                  print('tf error')
	                pt_odom = self.tf_l.transformPoint("odom",pc_point)

	                obj_odom = Obstacle()
	                obj_odom = obstacle
	                obj_odom.header.frame_id = "odom"
	                obj_odom.header.stamp = rospy.Time.now()
	                obj_odom.ns = "odom_namespace";

	                obj_odom.pose.orientation.x = velmaxlabel#clase placa
	                obj_odom.pose.position = pt_odom.point


	                obj_odom.scale.x = 0.5
	                obj_odom.scale.y = 0.5
	                obj_odom.scale.z = 0.5
	                obj_odom.color.r = color[2]
	                obj_odom.color.g = color[1]
	                obj_odom.color.b = color[0]
	                obj_odom.color.a = 1
	                self.n_tflight+=1
	                print("n traffic signs: ",self.n_tflight)
	                obj_odom.id = self.n_tflight
	                self.odom_objs_tflight.obstacle.append(obj_odom)
    try:
      self.odom_objs_tflight_pub.publish(self.odom_objs_tflight)
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  
  rospy.init_node('tf_light_detector', anonymous=True)
  ic = Detector()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
