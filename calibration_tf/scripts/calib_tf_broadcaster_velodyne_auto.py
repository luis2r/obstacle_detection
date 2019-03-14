#!/usr/bin/env python  

__author__ = "luis"
__date__ = "$Mar 11, 2016 4:15:34 AM$"

import rosbag
import cv2
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
import std_msgs.msg
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import sys
#import sklearn.preprocessing as preprocessing
import math
import roslib
roslib.load_manifest('calibration_tf')
import rospy
import tf
import cv2
import numpy as np
points = []

# mouse callback function
image_points = []  #puntos en la imagen

if __name__ == '__main__':

    rospy.init_node('radar_cam_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(20.0)

    
    
    while not rospy.is_shutdown():

	    # R_rect_00: 9.999239e-01 9.837760e-03 -7.445048e-03 -9.869795e-03 9.999421e-01 -4.278459e-03 7.402527e-03 4.351614e-03 9.999631e-01

	    R0_rect = 			np.array( [ [ 9.999239e-01,  9.837760e-03, -7.445048e-03, 0 ],
				                   		[ -9.869795e-03, 9.999421e-01, -4.278459e-03, 0 ], 
				                   		[ 7.402527e-03,  4.351614e-03, 9.999631e-01,  0 ], 
				                   		[ 0,             0,             0,            1.0       ] ], dtype = np.float32)

	    Tr_velo_to_cam  = 	np.array( [ [ 7.533745e-03, -9.999714e-01, -6.166020e-04, 0 ],
				                   		[ 1.480249e-02,  7.280733e-04, -9.998902e-01, 0 ], 
				                   		[ 9.998621e-01,  7.523790e-03,  1.480755e-02, 0 ], 
				                   		[ 0,             0,             0,            1.0       ] ], dtype = np.float32)
	    tvec = np.array([[-4.069766e-03], [-7.631618e-02], [-2.717806e-01]])

	    
		# print ("R0_rect: ",R0_rect)
	 #    print ('Tr_velo_to_cam: ',Tr_velo_to_cam)
	    #br.sendTransform((0.0,0.0,0.0),  tf.transformations.quaternion_from_euler(1,1,1), rospy.Time.now(), "bumblebee", "radaresr")
	    #br.sendTransform((tvec[0]/1000, tvec[1]/1000, tvec[2]/1000), tf.transformations.quaternion_from_euler(rvec[0],rvec[1],rvec[2]), rospy.Time.now(), "bumblebee", "radaresr")
	    br.sendTransform((tvec[0], tvec[1], tvec[2]), tf.transformations.quaternion_from_matrix(np.dot(R0_rect,Tr_velo_to_cam)), rospy.Time.now(), "velodyne", "stereo")
	    rate.sleep()