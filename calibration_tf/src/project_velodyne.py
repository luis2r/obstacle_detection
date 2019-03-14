#!/usr/bin/env python  

__author__ = "luis"
__date__ = "$Mar 11, 2016 4:15:34 AM$"

import rosbag
import cv2
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point

import sensor_msgs.point_cloud2 as pc2

import cv2

import std_msgs.msg
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import sys
#import sklearn.preprocessing as preprocessing

import roslib
roslib.load_manifest('calibration_tf')
import rospy
import tf

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
#        print point

if __name__ == '__main__':

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

    imgFile = cv2.imread('/home/luis/catkin_ws/src/calibration_tf/src/24.png')
    

#    cam_matrix = np.matrix([[969.887979124872,  0.0,               786.221412658691],                
#                             [0.0,               969.887979124872,  490.291244506836],
#                             [0.0,               0.0,               1.0             ]])

    cam_matrix = np.matrix([[972.254700,        0.0,               645.829590],  
                               [0,                 972.254700,        468.294891], 
                               [0,                 0,                 1             ]])        #nueva calibracion   

#    rt_matrix =  np.array([[-0.12733322,    -0.991831 ,     -0.00758284,     0.07135772],##matirz 30 puntos
#
#                            [-0.0168014,      0.00980085 ,   -0.99981081,    -0.2621232],
#                            [0.9917177,      -0.12718172 ,   -0.01791213,     0.07592308]])
                            
#    rt_matrix = np.array(  [[ -1.14127442e-01,  -9.93465900e-01,   6.99067139e-04,   -0.00686071],
#                            [ -1.43666947e-02,   9.46826127e-04,  -9.99896348e-01,   -0.28311785],
#                            [  9.93362248e-01,  -1.14125662e-01,  -1.43808797e-02,   0.08721076]])
#                            
    rt_matrix = np.array(  [[ 0.0159695,  -0.99981761, -0.01047323,   0.00751909],
                            [0.00707279,  0.01058726, -0.99991894,  -0.29864089  ],  
                            [0.99984747,  0.01589413,  0.00724058,  0.00420434  ]])    



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
        for topic, msg, t in bag.read_messages(topics=['/velodyne_points']): #### topicos

            if topic == "/velodyne_points":
                gen = pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z"))
                for m in gen:
                    
                    if ((m[0]>0 and m[0]<100 )and(m[1]>-30 and m[1]<30 )):
#                        print  " x : %f  y: %f  z: %f" %(m[0],m[1],m[2])
    #                    print "velodyne load 2"
                        xyz =  np.matrix([[m[0]],
                                          [m[1]],
                                          [m[2]], 
                                          [1]
                                          ])

                        xy = cam_matrix*rt_matrix*xyz
                        u = xy[0]/xy[2]
                        v = xy[1]/xy[2]



                        if((u>0 and u < 1280)and (v>0 and v < 960)):
#                            print int(u)
#                            print int(v)
                            

#                            if (m[0] > 0 and m[0] < 30):
#                                if (m[0] < 30 and m[0] > 0):

                            vmax = 255.0
                            vmin = 0.0
                            dv = vmax - vmin

                            b = 255.0
                            g = 255.0
                            r = 255.0

                            #d = 255 - depth
                            d = 255.0  -   (m[0]*255.0)/30.0
#                                    d = (255*m[0]) / m[0]
#                                    d =  m[0]
                            print("d:", d)

                            if (d < (vmin + 0.25 * dv)):
                                r = 0.0
                                g = (4.0 * (d - vmin) / dv)*255.0 
                            elif (d < (vmin + 0.5 * dv)):
                                r = 0.0
                                b = (1.0 + 4.0 * (vmin + 0.25 * dv - d) / dv)*255.0
                            elif (d < (vmin + 0.75 * dv)):
                                r = (4.0 * (d - vmin - 0.5 * dv) / dv)*255.0
                                b = 0.0
                            else: 
                                g = (1.0 + 4.0 * (vmin + 0.75 * dv - d) / dv)*255.0
                                print g
                                b = 0.0
                            cv2.circle(imgFile, (int(u),int(v)), int(2), [b,g,r],1) 
                            cv2.imshow('proy',imgFile)
                            cv2.waitKey(5)
               

        points_min_z = np.asarray(points_radar_min_z, dtype=np.float32)
        points_min_y = np.asarray(points_radar_min_y, dtype=np.float32)
        points_max_z = np.asarray(points_radar_max_z, dtype=np.float32)
        points_max_y = np.asarray(points_radar_max_y, dtype=np.float32)
        
        points_radar_array.extend(points_radar_min_z,)#########verdaderos pontos radar
        points_radar_array.extend(points_radar_min_y)#########verdaderos pontos radar
        points_radar_array.extend(points_radar_max_z)#########verdaderos pontos radar
        points_radar_array.extend(points_radar_max_y)#########verdaderos pontos radar
        
#        print "b",points_radar_array

        try:
            points_radar_array_1 = ((points_radar_array-np.zeros(1))/(np.max(points_radar_array)-np.zeros(1)))*900 
        except ValueError:
            pass
        mean_min_z = np.mean(points_min_z , axis=0)
        mean_min_y = np.mean(points_min_y , axis=0)
        mean_max_z = np.mean(points_max_z , axis=0)
        mean_max_y = np.mean(points_max_y , axis=0)
#        print "means"
#        print mean_min_z
#        print mean_min_y
#        print mean_max_y
#        print mean_max_z
        
        
#        mean_array_min_z = np.array([[ mean_min_z[0]],[ mean_min_z[1]],[ mean_min_z[2]  ]],dtype=np.float32)
#        mean_array_min_y = np.array([[ mean_min_y[0]],[ mean_min_y[1]],[ mean_min_y[2]  ]],dtype=np.float32)
        mean_array_max_z = np.array([[ mean_max_z[0]],[ mean_max_z[1]],[ mean_max_z[2]  ]],dtype=np.float32)
#        mean_array_max_y = np.array([[ mean_max_y[0]],[ mean_max_y[1]],[ mean_max_y[2]  ]],dtype=np.float32)
        
#        means_radar.append(mean_array_min_z)
#        means_radar.append(mean_array_min_y) 
        means_radar.append(mean_array_max_z) 
#        means_radar.append(mean_array_max_y)  
#        object1 = np.array([[ 3449.1234+200],[  2628.886 ],[0.20]],dtype=np.float32)
        means_radar_points = np.asarray(means_radar)   
        means_radar_points1 = ((means_radar_points-np.zeros(1))/(np.max(points_radar_array)-np.zeros(1)))*900 ##normalizando entre 0 y 1 y multiplicando por 300
#        points = ((points-np.min(points_radar_array))/(np.max(points_radar_array)-np.min(points_radar_array)))*700 ##normalizando entre 0 y 1 y multiplicando por 300
        img = np.zeros((img_size, img_size, 3), np.uint8)
#        print "max"
#        print np.max(points_radar_array)

        for (x, y, z) in points_radar_array_1:
            cv2.circle(img, (450-(int(y)), 900-(int(x))), 2, (0, 0, 255), -1) 

        i=1
        for point in means_radar_points1:
            #print point
            cv2.circle(img, ((450-int(point[1])), 900-(int(point[2]))), 2, (255,  255, 255), -1)
            cv2.putText(img, str(i), ((450-int(point[1]))+3, 900-(int(point[2]))-3), cv2.FONT_HERSHEY_SIMPLEX,0.4, (0,  255, 0))
            i = i+1

        fontScale = 0.4

        j=1
        for (cx,cy) in image_points:
            #print image_points
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
        #print point
        cv2.circle(img, (450-(point[1]), 900-(int(point[2]))), 1, (255,  255, 255), -1)
        i = i+1
    
    image_points_array = np.asarray(image_points) 
#    image_points_array.reshape(17,3, 1)
#    print image_points_array
#    print means_radar_points
#    print "puntos"
#    print image_points_array.shape
#    print means_radar_points.shape
    cv2.imshow('radar points', img)
    cv2.imshow('img', cv_image)
    ch = 0xFF & cv2.waitKey(0)
    cv2.destroyAllWindows()

   


    rvec = np.zeros((3,1,1), np.uint8)    #dtype = np.float32)
#    print rvec

    tvec =  np.zeros((3,1,1), np.uint8)    #dtype = np.float32)
#    print tvec
    	
    while not rospy.is_shutdown():

        #cv2.imshow('image',img)
        #k = cv2.waitKey(20) & 0xFF
        #if k == 27:

	

#        print "puntos"
#        print image_points_array.shape
#        print means_radar_points.shape

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
#        distortion_matrix = np.array([-0.353971596707789,  0.147952975025657,  0.0000798358243397781,  0.000352022660644842,  0.0], np.float32)
#        distortion_matrix = np.array([0.0,  0.0,  0.0,  0.0,  0.0], np.float32)
#        rvec = np.array([[0.0],  [0.0],  [0.0]], np.float32)
#        tvec = np.array([[0.0],  [0.0],  [0.0]], np.float32)
        last_rvec = np.array([[0.0], [0.0], [0.0]])
        last_tvec = np.array([[0.0], [0.0], [0.0]])
#        print rvec
#        print tvec
#        rtval, rvec, tvec = cv2.solvePnP(means_radar_points, image_points_array, cam_matrix, distortion_matrix, None, None, False, cv2.CV_EPNP)
        
        
#        
#        rvec, tvec, inliers = cv2.solvePnPRansac(means_radar_points, image_points_array, cam_matrix,
#        distCoeffs=None, # assuming no distortion
#        rvec=last_rvec,
#        tvec=last_tvec,
#        useExtrinsicGuess=True,
#        flags=cv2.CV_EPNP)
#        
#        means_radar_points = permute(means_radar_points,[1 3 2]);
#        image_points_array = permute(image_points_array,[1 3 2]);
        distortion_matrix = np.array([0.0,  0.0,  0.0,  0.0,  0.0], np.float32)
        
#        rtval, rvec, tvec = cv2.solvePnP(means_radar_points, image_points_array, cam_matrix, distCoeffs=None)       
        rvec, tvec, inliers = cv2.solvePnPRansac(means_radar_points, image_points_array, cam_matrix, distCoeffs=None)
        #cv2.Rodrigues(rvec, m[:3,:3])
        m = np.array([ [ 0, 0, 0, 0],
                       [ 0, 0, 0, 0 ], 
                       [ 0, 0, 0, 0 ], 
                       [ 0, 0, 0, 1.0       ] ], dtype = np.float32)
        m[:3,:3] = cv2.Rodrigues(rvec)[0]

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
        br.sendTransform((tvec[0], tvec[1], tvec[2]), tf.transformations.quaternion_from_matrix(m), rospy.Time.now(), "velodyne", "bumblebee")
        rate.sleep()
