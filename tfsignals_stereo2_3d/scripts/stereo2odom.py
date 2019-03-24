#!/usr/bin/env python
import rospy
import sys
from sensor_msgs.msg import PointCloud2, PointField
import ros_numpy
import numpy as np
from std_msgs.msg import Header

from msgs_perception.msg import ObstacleArray
from msgs_perception.msg import Obstacle

from msgs_perception.msg import BoundingBoxArray
from msgs_perception.msg import BoundingBox
from image_geometry import PinholeCameraModel
import tf

from sensor_msgs import point_cloud2

import struct

from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import CameraInfo
import image_geometry
from sensor_msgs.msg import Image

from geometry_msgs.msg import PointStamped

class CloudStereo_2_Odom:
	def __init__(self):
		self.odom_objs_tfsign_pub = rospy.Publisher("carina/sensor/stereo_traffic_sign_odom", ObstacleArray, queue_size = 1)
		self.odom_cloud_signs_pub = rospy.Publisher("carina/sensor/cloud_tf_signs_odom", PointCloud2, queue_size = 1)

		self.elas_sub   = rospy.Subscriber("elas/point_cloud", PointCloud2, self.stereo_cloud_callback)
		self.bbox_sub   = rospy.Subscriber("carina/perception/camera/obstacles",BoundingBoxArray,self.boundingbox_callback, queue_size = 1)
		self.info_sub_l = rospy.Subscriber("carina/sensor/camera/left/camera_info", CameraInfo, self.callback_info_left, queue_size=1)
		self.image_sub =  rospy.Subscriber("carina/sensor/camera/left/image_raw",Image,self.callback_image_left)

		# self.objs_sub   = rospy.Subscriber("carina/perception/lidar/obstacles_array", ObstacleArray, self.obstacles_callback, queue_size=1)
		self.bridge =CvBridge()
		self.camInfo = CameraInfo()
		self.tf_l = tf.TransformListener()

		self.img = Image()

		self.obstacles_loc = ObstacleArray()
		self.bbox_array = BoundingBoxArray()

		self.model = PinholeCameraModel()


		self.pc = PointCloud2()

	def callback_image_left(self,data):
		try:
			self.img =  data
			# self.cv_bridge = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)


	def callback_info_left(self,camerainfo):
		self.model.fromCameraInfo(camerainfo)


	def boundingbox_callback(self,bbox_loc):
		odom_objs_tfsign = ObstacleArray()
		odom_objs_tfsign.obstacle = []
		n_tfsigns = 0
		
		imgcv2 = self.bridge.imgmsg_to_cv2(self.img, "bgr8")

		r_r, g_r, b_r = np.random.uniform(0, 1, 3)
		points_cloud=[]
		for box in bbox_loc.objects:
			p1x=box.p1.x
			p1y=box.p1.y

			p3x=box.p3.x
			p3y=box.p3.y

			pc_roi = self.pc[int(p1y):int(p3y), int(p1x):int(p3x)]

			pc_a = np.zeros((pc_roi.shape[0], pc_roi.shape[1],4))

			pc_a[:,:,0] = pc_roi['x']
			pc_a[:,:,1] = pc_roi['y']
			pc_a[:,:,2] = pc_roi['z']
			pc_a[:,:,3] = pc_roi['rgb']
			valid_pc = np.all([pc_a!=[0.0,0.0,0.0,0.0]], axis=3)
			pc_roi = pc_a[np.where(valid_pc[0])]
			# print("pc_roi ",pc_roi)
			# print("pc_roi shape ",pc_roi)

			mean_a = np.mean(pc_roi,axis=0)
			min_a  = np.min(pc_roi,axis=0)
			max_a  = np.max(pc_roi,axis=0)
			# print("shape mean ",mean_a.shape)
			# print("mean ",mean_a)

			pt = [0, 0, 0, 0]

			if(np.absolute(mean_a[0])<1.5):
				pt[0] = mean_a[0]
			else:
				if(min_a[0]<0):
					pt[0] = max_a[0]
				else:
					pt[0] = min_a[0]

			if(np.absolute(mean_a[1])<1.5):
				pt[1] = mean_a[1]
			else:
				if(min_a[1]<0):
					pt[1] = max_a[1]
				else:
					pt[1] = min_a[1]

			if(np.absolute(mean_a[2])<1.5):
				pt[2] = mean_a[2]
			else:
				if(min_a[2]<0):
					pt[2] = max_a[2]
				else:
					pt[2] = min_a[2]
			# points_cloud.append(pt)

			#############################################publica point cloud##########################################
			# for i in range(int(p1y), int(p3y)):
			#     for j in range( int(p1x), int(p3x)):
			#         x = self.pc[i][j][0]
			#         y = self.pc[i][j][1]
			#         z = self.pc[i][j][2]
			#         pt = [x, y, z, 0]
			#         r = int(r_r * 255.0)
			#         g = int(g_r * 255.0)
			#         b = int(b_r * 255.0)
			#         a = 255
			#         # print r, g, b, a
			#         # rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
			#         rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a ))[0]
			#         # print hex(rgb)
			#         pt[3] = rgb
			#         points_cloud.append(pt)
			# fields = [PointField('x', 0, PointField.FLOAT32, 1),
			# PointField('y', 4, PointField.FLOAT32, 1),
			# PointField('z', 8, PointField.FLOAT32, 1),
			# PointField('rgb', 12, PointField.UINT32, 1)]
			# header = Header()
			# header.stamp = rospy.Time.now()
			# header.frame_id = "stereo"
			# pc2 = point_cloud2.create_cloud(header, fields, points_cloud, )
			# self.odom_cloud_signs_pub.publish(pc2)
			##############################################################################################################
			pc_point=PointStamped()
			pc_point.header.frame_id = bbox_loc.header.frame_id#"stereo"
			pc_point.header.stamp =rospy.Time(0)
			pc_point.point.x=pt[0]
			pc_point.point.y=pt[1]
			pc_point.point.z=pt[2]

			try:
				# print("frame: ",bbox_loc.header.frame_id)
				(trans_odom,rot_odom) = self.tf_l.lookupTransform('odom', bbox_loc.header.frame_id, rospy.Time(0))
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				print('tf error')
			pt_odom = self.tf_l.transformPoint("odom",pc_point)

			obj_odom = Obstacle()
			# obj_odom = obstacle
			obj_odom.header.frame_id = "odom"
			obj_odom.header.stamp = rospy.Time.now()
			obj_odom.ns = "odom_namespace";
			obj_odom.pose.position = pt_odom.point
			obj_odom.scale.x = 0.5
			obj_odom.scale.y = 0.5
			obj_odom.scale.z = 0.5

			classe = {
			"30km":(1,0.5,0),
			"60km":(1,0,1),
			"90km":(0,1,1),
			"traffig_light_red":(1,0,0),
			"traffig_light_green":(0,1,0),
			"traffig_light_yellow":(1,1,0)
			}
			color = classe[box.classe.data]
			# print(color[0])
			obj_odom.classes = [box.classe.data]
			obj_odom.color.r = color[0]
			obj_odom.color.g = color[1]
			obj_odom.color.b = color[2]
			obj_odom.color.a = 1
			obj_odom.lifetime= rospy.Duration(0.5)
			obj_odom.track_status = 1
			obj_odom.type = -1
			n_tfsigns+=1
			print("n traffic signs: ", n_tfsigns)
			obj_odom.id = n_tfsigns
			odom_objs_tfsign.obstacle.append(obj_odom)
		self.odom_objs_tfsign_pub.publish(odom_objs_tfsign)


	def stereo_cloud_callback(self,data):
		self.pc = ros_numpy.numpify(data)


def main(args):
	  
	rospy.init_node('CloudStereo_2_Odom_node', anonymous=True)
	cl_2_odom = CloudStereo_2_Odom()
	rospy.spin()


if __name__ == '__main__':
	main(sys.argv)
