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
    self.image_pub = rospy.Publisher("/carina/sensor/camera/left/image_detection",Image, queue_size=1)
    self.pub_cluster_tfsign = rospy.Publisher("/tfsign_cluster", PointCloud2, queue_size = 1)


    self.image_sub =  rospy.Subscriber("/carina/sensor/camera/left/image_raw",Image,self.callback)
    self.info_sub_l = rospy.Subscriber("/carina/sensor/camera/left/camera_info", CameraInfo, self.callback_info_left, queue_size=1)
    self.pcl_sub =    rospy.Subscriber("/carina/sensor/lidar/velodyne_obstacles_full", PointCloud2, self.point_cloud_callback, queue_size=1)

    self.bridge = CvBridge()
    self.camInfo = CameraInfo()
    self.tf_l = tf.TransformListener()
    # self.tf_ros = tf.Transformer()
    self.sign_l_x =0.0
    self.sign_l_y =0.0
    self.sign_r_x =0.0
    self.sign_r_y =0.0

    self.points=np.zeros((1,3))

    self.model = PinholeCameraModel()


    if os.path.isfile("clf/clf_svc_hog.pkl"):
      print("[INFO] loading classifier: SVC trained on HoG features...")
      self.svc = joblib.load("clf/clf_svc_hog.pkl")
      print("[INFO] Classifer is loaded as instance ::svc::")

    else:
      print("[INFO] pre-trained classifier not found.")




  def point_cloud_callback(self,pc2):
    pc = ros_numpy.numpify(pc2)
    # points=np.zeros((pc.shape[0],2))
    self.points=np.zeros((pc.shape[0],3))

    self.points[:,0]=pc['x']
    self.points[:,1]=pc['y']
    self.points[:,2]=pc['z']
    # print(points)
    # points = points[np.where( points[:,0]>1.5 and points[:,0]<50 and points[:,1]<20 and points[:,1]>-20 )[0]]

    self.points = self.points[np.where( self.points[:,0]>  2.0 )[0]]
    self.points = self.points[np.where( self.points[:,0]<  100 )[0]]
    self.points = self.points[np.where( self.points[:,1]>  -40 )[0]]
    self.points = self.points[np.where( self.points[:,1]<   40 )[0]]
    self.points_l = self.points[np.where( self.points[:,1]<   0 )[0]]
    self.points_r = self.points[np.where( self.points[:,1]>=   0 )[0]]

    # print(type(points)



  def callback_info_left(self,camerainfo):
    # self.camInfo = camerainfo
    self.model.fromCameraInfo(camerainfo)

    # rospy.loginfo("P:  %s",camerainfo.P)
    # rospy.loginfo("P:  %s",self.camInfo.P)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_image.shape
    cv_image_gray = cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY)
    cv_image_gray = cv2.medianBlur(cv_image_gray,5)
    circles = cv2.HoughCircles(cv_image_gray,cv2.HOUGH_GRADIENT,4,700, param1=100,param2=5,minRadius=10,maxRadius=80)
    circles = np.uint16(np.around(circles))
    for x,y,r in circles[0,:]:
      if(np.float(x)-np.float(r) <= 0.0):
        x1=1
      else:
        x1 = x-r
      if(np.float(y)-np.float(r) <= 0.0):
        y1=1
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

      cv2.rectangle(cv_image,(x1,y1),(x2,y2),(0,255,0),2)
      crop_img = cv_image[y-r-5:y+r+5, x-r-5:x+r+5]

      if(crop_img.size>4):
        gray = cv2.cvtColor(crop_img , cv2.COLOR_BGR2GRAY)
   
        if(gray.shape is not None):
          grayim = cv2.resize(gray,(40,40))
          (t1_feat, hogImage) = feature.hog(grayim, orientations=9, pixels_per_cell=(4, 4),
            block_norm='L2-Hys',cells_per_block=(2, 2), transform_sqrt=True, visualize=True)
          t1_predict = self.svc.predict(t1_feat.reshape(1, -1))
          # print("predicted: ",t1_predict[0])

          if(t1_predict[0]== 1):
            velmaxlabel = "30"
            cv2.rectangle(cv_image,(x1,y1),(x2,y2),(0,0,255),2)
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(cv_image, velmaxlabel,(x1,y1), font, 1,(255,255,255),2,cv2.LINE_AA)
            self.sign_l_x =x1
            self.sign_l_y =y1
            self.sign_r_x =x2
            self.sign_r_y =y2

            points_traffic_signs = []
            for p in self.points:
              if(p[0]>2.0 and p[1]< 40 and p[1]> -40):
                try:
                  (trans,rot) = self.tf_l.lookupTransform('stereo', 'velodyne', rospy.Time(0))
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                  print('tf error')
                pc_point=PointStamped()
                pc_point.header.frame_id = "velodyne"
                pc_point.header.stamp =rospy.Time(0)
                pc_point.point.x=p[0]
                pc_point.point.y=p[1]
                pc_point.point.z=p[2]
                pt=self.tf_l.transformPoint("stereo",pc_point)
                # print(p)
                p_2d =           self.model.project3dToPixel((pt.point.x,pt.point.y,pt.point.z))
                # unit_vec_ray3d = self.model.projectPixelTo3dRay((x,y))
                # print(unit_vec_ray3d)

                if (p_2d[0]>self.sign_l_x+5 and p_2d[1]>self.sign_l_y+5 and p_2d[0]<self.sign_r_x-5 and p_2d[1]<self.sign_r_y-5 ):
                  # points_traffic_signs.append(Point32(p.point.x,p.point.y,p.point.z))
                  # print("if")
                  # print(p_2d)
                  # print(p_2d[0], self.sign_l_x, p_2d[1], self.sign_l_y, p_2d[0], self.sign_r_x, p_2d[1],self.sign_r_y )
                  # print(pc_point)
                  points_traffic_signs.append([p[0],p[1],p[2]])
            # print()

            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = "velodyne"
            pc2 = point_cloud2.create_cloud_xyz32(header, points_traffic_signs)
            self.pub_cluster_tfsign.publish(pc2)


          if(t1_predict[0]== 2):
            velmaxlabel = "60"
            cv2.rectangle(cv_image,(x1,y1),(x2,y2),(0,0,255),2)
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(cv_image, velmaxlabel,(x1,y1), font, 1,(255,255,255),2,cv2.LINE_AA)
            self.sign_l_x =x1
            self.sign_l_y =y1
            self.sign_r_x =x2
            self.sign_r_y =y2

          if(t1_predict[0]== 3):
            velmaxlabel = "90"
            cv2.rectangle(cv_image,(x1,y1),(x2,y2),(0,0,255),2)
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(cv_image, velmaxlabel,(x1,y1), font, 1,(255,255,255),2,cv2.LINE_AA)
            self.sign_l_x =x1
            self.sign_l_y =y1
            self.sign_r_x =x2
            self.sign_r_y =y2





    #     points_cloud_tfsign = []
    #     for k in unique_labels:
    #         class_member_mask = (labels == k)
    #         xy = points[class_member_mask & core_samples_mask]
    #         r_r, g_r, b_r = np.random.uniform(0, 1, 3)
    #         for point in xy:
    #             x = point[0]
    #             y = point[1]
    #             z = point[2]
    #             pt = [x, y, z, 0]
    #             r = int(r_r * 255.0)
    #             g = int(g_r * 255.0)
    #             b = int(b_r * 255.0)
    #             a = 255
    #             rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
    #             pt[3] = rgb
    #             points_cloud.append(pt)
    #     fields = [PointField('x', 0, PointField.FLOAT32, 1),
    #     PointField('y', 4, PointField.FLOAT32, 1),
    #     PointField('z', 8, PointField.FLOAT32, 1),
    #     PointField('rgb', 12, PointField.UINT32, 1)
    #     ]
    #     header = Header()
    #     header.stamp = rospy.Time.now()
    #     header.frame_id = "velodyne"
    #     pc2 = point_cloud2.create_cloud(header, fields, points_cloud, )           
    # pub.publish(pc2)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  
  rospy.init_node('detector', anonymous=True)
  ic = Detector()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
