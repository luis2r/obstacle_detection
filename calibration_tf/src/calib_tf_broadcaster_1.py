#!/usr/bin/env python  
import roslib
roslib.load_manifest('calibration_tf')
import rospy
import tf
import cv2
import numpy as np
points = []

# mouse callback function
def draw_circle(event,x,y,flags,param):
    
    global ix,iy
    if event == cv2.EVENT_LBUTTONUP:
        cv2.circle(img,(x,y),5,(0,255,0),-1)
        #ix,iy = x,y
        point = np.array([[x],[y]],dtype=np.float32)
        points.append( point)
        print point

if __name__ == '__main__':

    #click
    #ix,iy = -1,-1
    # Create a black image, a window and bind the function to window
    img = cv2.imread('/home/luis/catkin_carina/catkin_ws/src/calibration_tf/parqueadero.png')
    cv2.namedWindow('image')
    cv2.setMouseCallback('image',draw_circle)
    points = []
    objects = []
    #objectPoints = np.zeros(shape=(5,3,1))
    #imagePoints = np.zeros(shape=(5,2,1))
    #print objectPoints
    #objectPoints = np.random.random((5,3,1))
    #imagePoints = np.random.random((5,2,1))
    #print objectPoints
    print "imagePoints"
    #print imagePoints
    #object1 = np.array([[2979.0 ],[-1167.61],[0.0]],dtype=np.float32)
    #object2 = np.array([[14595.6],[4912.00],[0.0]],dtype=np.float32)
    #object3 = np.array([[26368.5],[10814.5],[0.0]],dtype=np.float32)
    #object4 = np.array([[33117.6],[-3480.80],[0.0]],dtype=np.float32)
    #object5 = np.array([[21647.9],[-9144.41],[0.0]],dtype=np.float32)
    #objects.append( object1 )
    #objects.append( object2)
    #objects.append( object3)
    #objects.append( object4)
    #objects.append( object5)
    #objectPoints = np.asarray(objects)
    object1 = np.array([[ 3449.1234+200],[  2628.886 ],[0.20]],dtype=np.float32)
    object2 = np.array([[ 6439.6563+200],[ -1849.5287],[0.20]],dtype=np.float32)
    object3 = np.array([[ 5859.8266+200],[  1428.4725],[0.20]],dtype=np.float32)
    object4 = np.array([[ 8258.6765+200],[  2572.2907],[0.2]],dtype=np.float32)
    object5 = np.array([[11041.9890+200],[ -3649.0235],[0.20]],dtype=np.float32)
    object6 = np.array([[11065.9950+200],[ -1307.9101],[0.20]],dtype=np.float32)
    object7 = np.array([[14427.0260+200],[ -3301.6677],[0.20]],dtype=np.float32)
    object8 = np.array([[15940.174+200],[  3856.3261],[0.20]],dtype=np.float32)
    object9 = np.array([[18466.730+200],[ -3781.7969],[0.20]],dtype=np.float32)
    object10 = np.array([[8358.0713+200],[-2350.7347],[0.20]],dtype=np.float32)
    object11 = np.array([[10571.834+200],[ 1693.7307],[0.20]],dtype=np.float32)
    object12 = np.array([[11055.095+200],[ 3977.8626],[0.20]],dtype=np.float32)
    object13 = np.array([[13469.568+200],[ 1466.9203],[0.20]],dtype=np.float32)
    object14 = np.array([[13902.799+200],[ 3945.4639],[0.20]],dtype=np.float32)
    object15 = np.array([[16805.174+200],[-2439.4333],[0.20]],dtype=np.float32)
    object16 = np.array([[17946.676+200],[-313.2607],[0.20]],dtype=np.float32)
    object17 = np.array([[17846.386+200],[ 4483.6683],[0.20]],dtype=np.float32)

    objects.append( object1)
    objects.append( object2)
    objects.append( object3)
    objects.append( object4)
    objects.append( object5)
    objects.append( object6 )
    objects.append( object7)
    objects.append( object8)
    objects.append( object9)
    objects.append( object10)
    objects.append( object11)
    objects.append( object12)
    objects.append( object13)
    objects.append( object14)
    objects.append( object15)
    objects.append( object16)
    objects.append( object17)
    objectPoints = np.asarray(objects)

    points = []          
    point1 = np.array([[270 ],[141+500]],dtype=np.float32)
    point2 = np.array([[888 ],[84+500]],dtype=np.float32)
    point3 = np.array([[511 ],[96+500]],dtype=np.float32)
    point4 = np.array([[437 ],[67+500]],dtype=np.float32)
    point5 = np.array([[929 ],[44+500]],dtype=np.float32)
    point6 = np.array([[778 ],[45+500]],dtype=np.float32)
    point7 = np.array([[880 ],[30+500]],dtype=np.float32)
    point8 = np.array([[465 ],[27+500]],dtype=np.float32)
    point9 = np.array([[850 ],[19+500]],dtype=np.float32)
    point10 = np.array([[905],[64+500]],dtype=np.float32)
    point11 = np.array([[559],[49+500]],dtype=np.float32)
    point12 = np.array([[383],[47+500]],dtype=np.float32)
    point13 = np.array([[575],[36+500]],dtype=np.float32)
    point14 = np.array([[427],[35+500]],dtype=np.float32)
    point15 = np.array([[802],[22+500]],dtype=np.float32)
    point16 = np.array([[684],[19+500]],dtype=np.float32)
    point17 = np.array([[446],[22+500]],dtype=np.float32)
    points.append( point1)
    points.append( point2)
    points.append( point3)
    points.append( point4)
    points.append( point5)
    points.append( point6)
    points.append( point7)
    points.append( point8)
    points.append( point9)
    points.append( point10)
    points.append( point11)
    points.append( point12)
    points.append( point13)
    points.append( point14)
    points.append( point15)
    points.append( point16)
    points.append( point17)
    imagePoints = np.asarray(points)

    rospy.init_node('turtle_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    
    br.sendTransform((0.0,0.0,0.0),
                         tf.transformations.quaternion_from_euler(0,0,0),
                         rospy.Time.now(),
                         "base_link", "base_laser")    

   


    rvec = np.zeros((3,1,1), np.uint8)    #dtype = np.float32)
    print rvec

    tvec =  np.zeros((3,1,1), np.uint8)    #dtype = np.float32)
    print tvec
    	
    while not rospy.is_shutdown():

        #cv2.imshow('image',img)
        #k = cv2.waitKey(20) & 0xFF
        #if k == 27:

	

#        print imagePoints
#        imagePoints = np.swapaxes(imagePoints, 0,2)
        print "puntos"
        print imagePoints.shape
        print objectPoints.shape
        #print imagePoints
        cam_matrix = np.array([[1029.24226091734, 0.0,             668.643287622175],
                                [0.0,               1028.01653412214,498.637148816509],
                                [0.0,0.0,1.0]], np.float32)
        #
        #distortion coefficients 
        #D: [-0.353971596707789, 0.147952975025657, 7.98358243397781e-05, 0.000352022660644842, 0.0]
        #distortion matrix
        distortion_matrix = np.array([-0.353971596707789,  0.147952975025657,  0.0000798358243397781,  0.000352022660644842,  0.0], np.float32)
        rtval, rvec, tvec = cv2.solvePnP(objectPoints, imagePoints, cam_matrix, distortion_matrix)
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
        print imagePoints
        print "objectPoints"
        print objectPoints
        print "rotM_cam"
        print m
	print "tvec"	
        print tvec
        #break
        #elif k == ord('a'):
        #print ix,iy
        #br.sendTransform((0.0,0.0,0.0),  tf.transformations.quaternion_from_euler(1,1,1), rospy.Time.now(), "bumblebee", "radaresr")
        #br.sendTransform((tvec[0]/1000, tvec[1]/1000, tvec[2]/1000), tf.transformations.quaternion_from_euler(rvec[0],rvec[1],rvec[2]), rospy.Time.now(), "bumblebee", "radaresr")
        br.sendTransform((tvec[0]/1000, tvec[1]/1000, tvec[2]/1000), tf.transformations.quaternion_from_matrix(m), rospy.Time.now(), "radar", "bumblebee")
        rate.sleep()
