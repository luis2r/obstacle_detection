#!/usr/bin/env python

import roslib
import sys
import rospy
from std_msgs.msg import Header

from msgs_perception.msg import ObstacleArray
from msgs_perception.msg import Obstacle

from msgs_perception.msg import BoundingBoxArray
from msgs_perception.msg import BoundingBox
from image_geometry import PinholeCameraModel
import tf

from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PointStamped

class Imbox2odom:

  def __init__(self):
    # self.image_pub = rospy.Publisher("/carina/sensor/camera/left/tsign_image_detection",Image, queue_size=1)
    self.odom_objs_tfsign_pub = rospy.Publisher("/carina/sensor/tfsigns_odom", ObstacleArray, queue_size = 1)

    self.bbox_sub =  rospy.Subscriber("/carina/sensor/tfsigns_box",BoundingBoxArray,self.boundingbox_callback, queue_size = 1)
    self.info_sub_l = rospy.Subscriber("/carina/sensor/camera/left/camera_info", CameraInfo, self.callback_info_left, queue_size=1)
    self.objs_sub =    rospy.Subscriber("/carina/perception/lidar/obstacles_array", ObstacleArray, self.obstacles_callback, queue_size=1)

    self.camInfo = CameraInfo()
    self.tf_l = tf.TransformListener()

    self.obstacles_loc = ObstacleArray()

    self.model = PinholeCameraModel()
    self.odom_objs_tfsign = ObstacleArray()
    self.odom_objs_tfsign.obstacle = []
    self.n_tfsigns = 0

    self.rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        # tfs = "Publish traffic signs %s" % rospy.get_time()
        # rospy.loginfo(tfs)
       	self.odom_objs_tfsign_pub.publish(self.odom_objs_tfsign)
        self.rate.sleep()



  def obstacles_callback(self,a_obstacles):
    self.obstacles_loc=a_obstacles



  def callback_info_left(self,camerainfo):
    self.model.fromCameraInfo(camerainfo)


  def boundingbox_callback(self,bbox_loc):

	  for obstacle in self.obstacles_loc.obstacle:
	    if( obstacle.pose.position.x>0.0 and obstacle.pose.position.x < 30 and obstacle.scale.x<0.80 and obstacle.scale.y<0.80 and obstacle.scale.x*obstacle.scale.y<0.50):
	      try:
	        # (trans,rot) = self.tf_l.lookupTransform('stereo', 'velodyne', rospy.Time(0))
	        (trans,rot) = self.tf_l.lookupTransform(bbox_loc.header.frame_id, obstacle.header.frame_id, rospy.Time(0))
	      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
	        print('tf error')

	      pc_point=PointStamped()
	      pc_point.header.frame_id = obstacle.header.frame_id#"velodyne"
	      # pc_point.header.frame_id = "velodyne"
	      pc_point.header.stamp =rospy.Time(0)
	      pc_point.point=obstacle.pose.position
	      # pt=self.tf_l.transformPoint("stereo",pc_point)
	      pt=self.tf_l.transformPoint(bbox_loc.header.frame_id,pc_point)

	      p_2d =  self.model.project3dToPixel((pt.point.x,pt.point.y,pt.point.z))

	      for bbox in bbox_loc.objects:
	          if( p_2d[0]>bbox.p1.x-30 and p_2d[0]<bbox.p3.x+30 ):

	            try:
	              # (trans_odom,rot_odom) = self.tf_l.lookupTransform('odom', 'velodyne', rospy.Time(0))
	              (trans_odom,rot_odom) = self.tf_l.lookupTransform('odom', obstacle.header.frame_id, rospy.Time(0))
	            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
	              print('tf error')
	            pt_odom = self.tf_l.transformPoint("odom",pc_point)

	            obj_odom = Obstacle()
	            obj_odom = obstacle
	            obj_odom.header.frame_id = "odom"
	            obj_odom.header.stamp = rospy.Time.now()
	            obj_odom.ns = "odom_namespace";
	            obj_odom.pose.position = pt_odom.point
	            obj_odom.scale.x = 0.5
	            obj_odom.scale.y = 0.5
	            obj_odom.scale.z = 0.5

	            classe = {
	            "30":(1,0.5,0),
	            "60":(1,0,1),
	            "90":(0,1,1),
	            "r":(1,0,0),
	            "g":(0,1,0),
	            "y":(1,1,0)
	            }
	            color = classe[bbox.classe.data]
	            print(color[0])
	            # print(type(obj_odom.color))
	            # obj_odom.color= color
	            obj_odom.color.r = color[0]
	            obj_odom.color.g = color[1]
	            obj_odom.color.b = color[2]
	            obj_odom.color.a = 1
	            obj_odom.lifetime= rospy.Duration(3)
	            self.n_tfsigns+=1
	            print("n traffic signs: ", self.n_tfsigns)
	            obj_odom.id = self.n_tfsigns
	            self.odom_objs_tfsign.obstacle.append(obj_odom)
	  # self.odom_objs_tfsign_pub.publish(self.odom_objs_tfsign)

def main(args):
  
  rospy.init_node('imgBoxtoOdom', anonymous=True)
  ic = Imbox2odom()

  # rospy.spin()


if __name__ == '__main__':
    main(sys.argv)
