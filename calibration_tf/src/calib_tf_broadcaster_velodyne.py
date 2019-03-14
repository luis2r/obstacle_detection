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
from geometry_msgs.msg import Pose
import math

import roslib
roslib.load_manifest('calibration_tf')
import rospy
import tf

image_points = []  #puntos en la imagen

def draw_circle(event,x,y,flags,param):
    
    global ix,iy
    if event == cv2.EVENT_LBUTTONUP:
        point = np.array([[x],[y]],dtype=np.float32)
        cv2.circle(cv_image,(x,y),5,(0,255,0),-1)
        image_points.append( point)
#        print point

if __name__ == '__main__':

    rospy.init_node('radar_cam_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(20.0)

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
        
        points_radar_max_y = []
        points_radar_max_z = []
        points_radar_min_y = []
        points_radar_min_z = []
#        cv_image = []
        for topic, msg, t in bag.read_messages(topics=['/square_calib_vel', '/stereo/narrow/left/image_raw']): #### topicos

            if topic == "/square_calib_vel":
                
                for m in msg.markers:
                    point_square_y_max_y =  -999999
                    point_square_z_max_z =  -999999

                    point_square_y_min_y = 999999
                    point_square_z_min_z = 999999

                    points_square_y_max_y = -999999
                    points_square_z_max_z = -999999

                    points_square_y_min_y = 999999
                    points_square_z_min_z = 999999

                    for n in m.points:
                        if n.z < points_square_z_min_z :
                            points_square_z_min_x = n.x
                            points_square_z_min_y = n.y
                            points_square_z_min_z = n.z
                            
                        if n.y < points_square_y_min_y :
                            points_square_y_min_x = n.x
                            points_square_y_min_y = n.y
                            points_square_y_min_z = n.z
                            
                        if n.z > points_square_z_max_z :
                            points_square_z_max_x = n.x
                            points_square_z_max_y = n.y
                            points_square_z_max_z = n.z
                            
                        if n.y > points_square_y_max_y :
                            points_square_y_max_x = n.x
                            points_square_y_max_y = n.y
                            points_square_y_max_z = n.z
                        
                points_radar_min_z.append(((points_square_z_min_x), (points_square_z_min_y), (points_square_z_min_z)))
                points_radar_min_y.append(((points_square_y_min_x), (points_square_y_min_y), (points_square_y_min_z)))
                points_radar_max_z.append(((points_square_z_max_x), (points_square_z_max_y), (points_square_z_max_z)))
                points_radar_max_y.append(((points_square_y_max_x), (points_square_y_max_y), (points_square_y_max_z)))

            if (topic == "/stereo/narrow/left/image_raw" and not(img_load)):
                img_load = True
                print "image load", bag_path
                try:
                    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
                except CvBridgeError as e:
                    print(e)    

        points_min_z = np.asarray(points_radar_min_z, dtype=np.float32)
        points_min_y = np.asarray(points_radar_min_y, dtype=np.float32)
        points_max_z = np.asarray(points_radar_max_z, dtype=np.float32)
        points_max_y = np.asarray(points_radar_max_y, dtype=np.float32)
        
        points_radar_array.extend(points_radar_min_z,)#########verdaderos pontos radar
        points_radar_array.extend(points_radar_min_y)#########verdaderos pontos radar
        points_radar_array.extend(points_radar_max_z)#########verdaderos pontos radar
        points_radar_array.extend(points_radar_max_y)#########verdaderos pontos radar

        try:
            points_radar_array_1 = ((points_radar_array-np.zeros(1))/(np.max(points_radar_array)-np.zeros(1)))*900 
        except ValueError:
            pass
        mean_min_z = np.mean(points_min_z , axis=0)
        mean_min_y = np.mean(points_min_y , axis=0)
        mean_max_z = np.mean(points_max_z , axis=0)
        mean_max_y = np.mean(points_max_y , axis=0)
        
        mean_array_min_z = np.array(((mean_min_z[0]),(mean_min_z[1]), (mean_min_z[2])),dtype=np.float32)
        mean_array_min_y = np.array(( (mean_min_y[0]), (mean_min_y[1]), (mean_min_y[2])),dtype=np.float32)
        mean_array_max_z = np.array(( (mean_max_z[0]), (mean_max_z[1]), (mean_max_z[2])),dtype=np.float32)
        mean_array_max_y = np.array(( (mean_max_y[0]), (mean_max_y[1]), (mean_max_y[2])),dtype=np.float32)
        
        means_radar.append(mean_array_min_z)
        means_radar.append(mean_array_min_y) 
        means_radar.append(mean_array_max_z) 
        means_radar.append(mean_array_max_y)  
        means_radar_points = np.asarray(means_radar)   
        means_radar_points1 = ((means_radar_points-np.zeros(1))/(np.max(points_radar_array)-np.zeros(1)))*900 ##normalizando entre 0 y 1 y multiplicando por 300
        img = np.zeros((img_size, img_size, 3), np.uint8)


        for (x, y, z) in points_radar_array_1:
            print "x1 ",x," y1 ",y," z1 ",z
            cv2.circle(img, (450-(int(y)), 900-(int(x))), 2, (0, 0, 255), -1) 

        i=1
        for (x, y, z) in means_radar_points1:
            print "x ",x," y ",y," z ",z
            cv2.circle(img, (450-(int(y)), 900-(int(x))), 2, (255, 255, 255), -1) 
            cv2.putText(img, str(i), (450-(int(y))+3, 900-(int(x))-3), cv2.FONT_HERSHEY_SIMPLEX,0.4, (0,  255, 0))
            i = i+1

        fontScale = 0.4

        j=1
        for (cx,cy) in image_points:
            text = str(j)
            cv2.circle(cv_image, (cx,cy), 2, (0, 0, 255), 5)
            cv2.putText(cv_image, text, (cx,cy), cv2.FONT_HERSHEY_SIMPLEX,fontScale, (0, 255, 0))
            j=j+1
            
        cv2.line(img, (1, 1), (10, 1), (255,  255, 255), 1)
        cv2.putText(img, str("%.2f m" % np.max(points_radar_array)), (11, 10), cv2.FONT_HERSHEY_SIMPLEX,fontScale, (255, 255, 255))
        
        cv2.line(img, (0,(img_size-1)/2),   (10,(img_size-1)/2), (255,  255, 255), 1)
        cv2.putText(img, str("%.2f m" % (np.max(points_radar_array)/2)), (11, (img_size)/2+10), cv2.FONT_HERSHEY_SIMPLEX,fontScale, (255, 255, 255))
        
        #lineas eje x
        cv2.line(img, (((img_size)/4)*3,(img_size-1)),   (((img_size)/4)*3,(img_size-1)-10), (255,  255, 255), 1)
        cv2.putText(img, str("%.2f m" % -(np.max(points_radar_array)/4)), (((img_size)/4)*3, img_size-11 ), cv2.FONT_HERSHEY_SIMPLEX,fontScale, (255, 255, 255))
        cv2.line(img, ((img_size)/4,(img_size-1)),   ((img_size)/4,(img_size-1)-10), (255,  255, 255), 1)
        cv2.putText(img, str("%.2f m" % (np.max(points_radar_array)/4)), ((img_size)/4, img_size-11 ), cv2.FONT_HERSHEY_SIMPLEX,fontScale, (255, 255, 255))
        cv2.circle(img, ((img_size-1)/2, (img_size-1)), 15, (255,  255, 255), -1)
        cv2.putText(img, "CaRINA 2 Plataform", (((img_size/2)-62), (img_size-25)), cv2.FONT_HERSHEY_SIMPLEX,fontScale, (0, 255, 0))
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

    for (x, y, z), label in zip(np.float32(points_radar_array_2), labels.ravel()):
        c = map(int, colors[label])
        cv2.circle(img, (450-int(y), 900-(int(x))), 2, c, -1)   

    i=1
    for point in means_radar_points1:
        cv2.circle(img, (450-(int(y)), 900-(int(x))), 2, (255, 255, 255), -1) 
        i = i+1
    
    image_points_array = np.asarray(image_points) 

    cv2.imshow('radar points', img)
    cv2.imshow('img', cv_image)
    ch = 0xFF & cv2.waitKey(0)
    cv2.destroyAllWindows()

    rvec = np.zeros((3,1,1), np.uint8)
    tvec =  np.zeros((3,1,1), np.uint8)

    while not rospy.is_shutdown():
                               
        cam_matrix = np.array([[972.254700,        0.0,               645.829590],  
                               [0,                 972.254700,        468.294891], 
                               [0,                 0,                 1]],             np.float32)          #nueva calibracion                    

        last_rvec = np.array([[0.0], [0.0], [0.0]])
        last_tvec = np.array([[0.0], [0.0], [0.0]])

#        rvec, tvec, inliers = cv2.solvePnPRansac(means_radar_points, image_points_array, cam_matrix,
#        distCoeffs=None, # assuming no distortion
#        rvec=last_rvec,
#        tvec=last_tvec,
#        useExtrinsicGuess=True,
#        flags=cv2.CV_EPNP)

        distortion_matrix = np.array([0.0,  0.0,  0.0,  0.0,  0.0], np.float32)
        
        rtval, rvec, tvec = cv2.solvePnP(means_radar_points, image_points_array, cam_matrix, distCoeffs=None)       
        
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

        print "imagePoints"
        print image_points_array
        print "objectPoints"
        print means_radar_points
        print "rotM_cam"
        print m
	print "tvec"	
        print tvec
        print 'rvec: ',rvec
        #br.sendTransform((0.0,0.0,0.0),  tf.transformations.quaternion_from_euler(1,1,1), rospy.Time.now(), "bumblebee", "radaresr")
        #br.sendTransform((tvec[0]/1000, tvec[1]/1000, tvec[2]/1000), tf.transformations.quaternion_from_euler(rvec[0],rvec[1],rvec[2]), rospy.Time.now(), "bumblebee", "radaresr")
        br.sendTransform((tvec[0], tvec[1], tvec[2]), tf.transformations.quaternion_from_matrix(m), rospy.Time.now(), "velodyne", "bumblebee")
        rate.sleep()
