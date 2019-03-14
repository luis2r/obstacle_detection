#!/usr/bin/env python

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class Detector:

  def __init__(self):
    self.image_pub = rospy.Publisher("/carla/ego_vehicle/camera/rgb/front/image_detection",Image, queue_size=1)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/carla/ego_vehicle/camera/rgb/front/image_color",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_image.shape
    # if cols > 60 and rows > 60 :
    #   cv2.circle(cv_image, (50,50), 10, 255)

    cv_image_gray = cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY)

    cv_image_gray = cv2.medianBlur(cv_image_gray,5)
    # cimg = cv2.cvtColor(cv_image,cv2.COLOR_GRAY2BGR)
    circles = cv2.HoughCircles(cv_image_gray,cv2.HOUGH_GRADIENT,2,150, param1=100,param2=5,minRadius=10,maxRadius=80)
    circles = np.uint16(np.around(circles))
    for x,y,r in circles[0,:]:
      # draw the outer circle
      # print("shape", cv_image.shape)
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

      crop_img = cv_image[y-r:y+r, x-r:x+r]
      cv2.imshow("cropped", crop_img)
      cv2.waitKey(300)




    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  ic = Detector()
  rospy.init_node('detector', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
