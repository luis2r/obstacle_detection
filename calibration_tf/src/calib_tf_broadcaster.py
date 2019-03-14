#!/usr/bin/env python  

__author__ = "luis"
__date__ = "$Mar 11, 2016 4:15:34 AM$"

import rosbag
import cv2
from sensor_msgs.msg import Image
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
# mouse callback function

def draw_circle(event,x,y,flags,param):
    
    global ix,iy
    if event == cv2.EVENT_LBUTTONUP:
        point = np.array([[x],[y]],dtype=np.float32)
        cv2.circle(cv_image,(x,y),5,(0,255,0),-1)
        image_points.append( point)
        print point

if __name__ == '__main__':

    rospy.init_node('radar_cam_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    
    br.sendTransform((0.0,0.0,0.0),
                         tf.transformations.quaternion_from_euler(0,0,0),
                         rospy.Time.now(),
                         "radar", "bumblebee")    


    cv2.namedWindow('img')
    cv2.setMouseCallback('img',draw_circle)

    points_radar_array = []
    means_radar = []   #medias radar
    
    cluster_n = len(sys.argv)-1
    img_size = 900
    print "argumentos",sys.argv[1]," ",sys.argv[0], " ", len(sys.argv)-1

    # generating bright palette
    colors = np.zeros((1, cluster_n, 3), np.uint8)
    colors[0,:] = 255
    colors[0,:,0] = np.arange(0, 180, 180.0/cluster_n)
    colors = cv2.cvtColor(colors, cv2.COLOR_HSV2BGR)[0]
    bridge = CvBridge()
    kernel = np.ones((11,11),np.uint8)

    #carga de datos radar e imagen
    for bag_path in sys.argv[1:]:  ##### bags
        img_load = False
        print bag_path
        bag = rosbag.Bag(str(bag_path))
        points_radar = []
#        cv_image = []
        for topic, msg, t in bag.read_messages(topics=['/radar_esr_msg_array_1', '/stereo/wide/left/image_raw']): #### topicos

            if topic == "/radar_esr_msg_array_1":

                for m in msg.obstacle:
                    if not(m.pose.position.x == 0 and   m.pose.position.y == 0):
#                        if (m.pose.position.x < 4.5 and m.pose.position.y < 2 and m.pose.position.y > -2):
                        if (m.pose.position.x < 19.5 and m.pose.position.y < 5 and m.pose.position.y > -5):
                            points_radar.append(((m.pose.position.x), (m.pose.position.y)))

            if (topic == "/stereo/wide/left/image_raw" and not(img_load)):
                img_load = True
                print "image load", bag_path
                try:
                    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
                except CvBridgeError as e:
                    print(e)     

        points = np.asarray(points_radar, dtype=np.float32)
        points_radar_array.extend(points_radar)#########verdaderos pontos radar
        
        points_radar_array_1 = ((points_radar_array-np.min(points_radar_array))/(np.max(points_radar_array)-np.min(points_radar_array)))*700 ##normalizando entre 0 y 1 y multiplicando por 300
        mean = np.mean(points, axis=0)
        mean_array = np.array([[mean[0]], [mean[1]], [0.0]],dtype=np.float32)
        means_radar.append(mean_array) 
#        object1 = np.array([[ 3449.1234+200],[  2628.886 ],[0.20]],dtype=np.float32)
        means_radar_points = np.asarray(means_radar)   
        means_radar_points1 = ((means_radar_points-np.min(points_radar_array))/(np.max(points_radar_array)-np.min(points_radar_array)))*700 ##normalizando entre 0 y 1 y multiplicando por 300
#        points = ((points-np.min(points_radar_array))/(np.max(points_radar_array)-np.min(points_radar_array)))*700 ##normalizando entre 0 y 1 y multiplicando por 300
        img = np.zeros((img_size, img_size, 3), np.uint8)

        for (x, y) in points_radar_array_1:
            cv2.circle(img, (900-(int(x)+300), 900-(int(y))), 2, (0, 0, 255), -1) 

        i=1
        for point in means_radar_points1:
            #print point
            cv2.circle(img, (900-(int(point[0])+300), 900-(int(point[1]))), 1, (255,  255, 255), -1)
            cv2.putText(img, str(i), (900-(int(point[0])+300)+3, 900-(int(point[1]))-3), cv2.FONT_HERSHEY_SIMPLEX,0.4, (0,  255, 0))
            i = i+1

        fontScale = 0.4

        j=1
        for (cx,cy) in image_points:
            #print image_points
            text = str(j)
            cv2.circle(cv_image, (cx,cy), 1, (0, 0, 255), 5)
            cv2.putText(cv_image, text, (cx,cy), cv2.FONT_HERSHEY_SIMPLEX,fontScale, (0, 255, 0))
            j=j+1

        cv2.imshow('radar points', img)
        
        while(1):
            cv2.imshow('img',cv_image)
            k = cv2.waitKey(20) & 0xFF    ###tecla escape
            if k == 27:
                break
        
        
        bag.close()

    #cluster de los datos de radar
    term_crit = (cv2.TERM_CRITERIA_EPS, 30, 0.1)
    points_radar_array_2= np.asarray(points_radar_array_1, dtype=np.float32)
    ret, labels, centers = cv2.kmeans(points_radar_array_2, cluster_n, term_crit, 10, 0)

    for (x, y), label in zip(np.float32(points_radar_array_2), labels.ravel()):
        c = map(int, colors[label])
        cv2.circle(img, (900-(int(x)+300), 900-(int(y))), 2, c, -1)   

    i=1
    for point in means_radar_points1:
        #print point
        cv2.circle(img, (900-(int(point[0])+300), 900-(int(point[1]))), 1, (255,  255, 255), -1)
        i = i+1
    
    image_points_array = np.asarray(image_points) 
#    image_points_array.reshape(17,3, 1)
    print image_points_array
    print means_radar_points
    print "puntos"
    print image_points_array.shape
    print means_radar_points.shape
    cv2.imshow('radar points', img)
    cv2.imshow('img', cv_image)
    ch = 0xFF & cv2.waitKey(0)
    cv2.destroyAllWindows()

   


    rvec = np.zeros((3,1,1), np.uint8)    #dtype = np.float32)
    print rvec

    tvec =  np.zeros((3,1,1), np.uint8)    #dtype = np.float32)
    print tvec
    	
    while not rospy.is_shutdown():

        #cv2.imshow('image',img)
        #k = cv2.waitKey(20) & 0xFF
        #if k == 27:

	

        print "puntos"
        print image_points_array.shape
        print means_radar_points.shape
        
        cam_matrix = np.array([[969.887979124872,  0.0,               786.221412658691],                
                               [0.0,               969.887979124872,  490.291244506836],
                               [0.0,               0.0,               1.0]],             np.float32)
  
#        cam_matrix = np.array([[1029.24226091734, 0.0,             668.643287622175],
#                                [0.0,               1028.01653412214,498.637148816509],
#                                [0.0,0.0,1.0]], np.float32)
        #
        #distortion coefficients 
        #D: [-0.353971596707789, 0.147952975025657, 7.98358243397781e-05, 0.000352022660644842, 0.0]
        #distortion matrix
        
        distortion_matrix = np.array([-0.353971596707789,  0.147952975025657,  0.0000798358243397781,  0.000352022660644842,  0.0], np.float32)
        
        rtval, rvec, tvec = cv2.solvePnP(means_radar_points, image_points_array, cam_matrix, distortion_matrix)
        #cv2.Rodrigues(rvec, m[:3,:3])
        m = np.array([ [ 0, 0, 0, 0],
                       [ 0, 0, 0, 0 ], 
                       [ 0, 0, 0, 0 ], 
                       [ 0, 0, 0, 1.0       ] ], dtype = np.float32)
        m[:3,:3] = cv2.Rodrigues(rvec)[0]
        
        
        
        
        cameraMatrix, rotMatrix, transVect, rotMatrixX, rotMatrixY, rotMatrixZ, eulerAngles = cv2.decomposeProjectionMatrix(m[:3,:4])

        print "euler graus",eulerAngles
        print "roll radianes",(eulerAngles[0]*math.pi)/180
        print "pitch radianes",(eulerAngles[1]*math.pi)/180
        print "yaw radianes",(eulerAngles[2]*math.pi)/180

        #cv2.Rodrigues( rvec, rmat )
        print "cam_matrix"
        print cam_matrix
        print "distortion_matrix"
        print distortion_matrix
        print "imagePoints"
        print image_points_array
        print "objectPoints"
        print means_radar_points
        print "rotM_cam"
        print m
	print "tvec"	
        print tvec
        #break
        #elif k == ord('a'):
        #print ix,iy
        #br.sendTransform((0.0,0.0,0.0),  tf.transformations.quaternion_from_euler(1,1,1), rospy.Time.now(), "bumblebee", "radaresr")
        #br.sendTransform((tvec[0]/1000, tvec[1]/1000, tvec[2]/1000), tf.transformations.quaternion_from_euler(rvec[0],rvec[1],rvec[2]), rospy.Time.now(), "bumblebee", "radaresr")
        br.sendTransform((tvec[0], tvec[1], tvec[2]), tf.transformations.quaternion_from_matrix(m), rospy.Time.now(), "radar", "bumblebee")
        rate.sleep()
