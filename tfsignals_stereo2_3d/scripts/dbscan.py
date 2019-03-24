#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import ros_numpy
import numpy as np
from sklearn.cluster import DBSCAN
from sklearn import metrics
from sklearn.datasets.samples_generator import make_blobs
from sklearn.preprocessing import StandardScaler

import matplotlib.pyplot as plt

from scipy.spatial import ConvexHull

from visualization_msgs.msg import *

from geometry_msgs.msg import Point
from skimage.measure import find_contours, approximate_polygon

from msgs_perception.msg import ObstacleArray
from msgs_perception.msg import Obstacle

import cv2

from tf.transformations import *

pub = rospy.Publisher('/velodyne_markers', MarkerArray, queue_size=1)
pub_obj = rospy.Publisher('/obstacles', ObstacleArray, queue_size=1)

markerArray = MarkerArray()
obstacle_Array = ObstacleArray()
t = rospy.Duration(0.21) 

def on_new_point_cloud(data):
    markerArray.markers=[]
    obstacle_Array.obstacle=[]

       

    pc = ros_numpy.numpify(data)
    points=np.zeros((pc.shape[0],2))
    # points=np.zeros((pc.shape[0],3))

    points[:,0]=pc['x']
    points[:,1]=pc['y']
    # points[:,2]=pc['z']


    # #############################################################################
    # Compute DBSCAN
    db = DBSCAN(eps=0.7, min_samples=3).fit(points)
    core_samples_mask = np.zeros_like(db.labels_, dtype=bool)
    core_samples_mask[db.core_sample_indices_] = True
    labels = db.labels_


    # Number of clusters in labels, ignoring noise if present.
    n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
    n_noise_ = list(labels).count(-1)


    ##############################################################################
    ##### Plot result
    unique_labels = set(labels)
    id_obj = 0
    for k in unique_labels:
        class_member_mask = (labels == k)
        xy = points[class_member_mask & core_samples_mask]

        r, g, b = np.random.uniform(0, 1, 3)
        col = (r, g, b, 1)
        a,b = xy.shape
        hull_list = []

        if(a>0):
            if(a > 3 and a < 500):
                if(a < 50):
                    xy[0][0] = xy[0][0] + 0.01
                    xy[0][1] = xy[0][1] - 0.01
                try:
                    hull = ConvexHull(xy)
                except ValueError:
                    print("Oops! valor ValueError")
                except QhullError:
                    print("Oops!  QhullError")
                else:

                    if(a>450):
                        marker = Marker()
                        marker.header.frame_id = "/velodyne"
                        marker.type = marker.LINE_STRIP
                        marker.action = marker.ADD
                        marker.ns = "my_namespace";

                        # marker scale
                        marker.scale.x = 0.3
                        marker.scale.y = 0.3
                        marker.scale.z = 0.3

                        # marker color
                        marker.color.a = 1.0
                        marker.color.r = 1.0
                        marker.color.g = 0.0
                        marker.color.b = 0.0

                        # marker orientaiton
                        marker.pose.orientation.x = 0.0
                        marker.pose.orientation.y = 0.0
                        marker.pose.orientation.z = 0.0
                        marker.pose.orientation.w = 1.0

                        # marker position
                        marker.pose.position.x = 0.0
                        marker.pose.position.y = 0.0
                        marker.pose.position.z = 0.0
                        # marker line points
                        marker.points = []

                        for simplex in hull.vertices:
                            hull_list.append((xy[simplex, 0], xy[simplex, 1]))
                            line_point = Point()
                            line_point.x = xy[simplex, 0]
                            line_point.y = xy[simplex, 1]
                            line_point.z = 0.0
                            marker.points.append(line_point)
                        line_point_f = Point()
                        line_point_f.x = hull_list[0][0]
                        line_point_f.y = hull_list[0][1]
                        line_point_f.z = 0.0
                        marker.points.append(line_point_f) 
                        marker.lifetime = t
                        markerArray.markers.append(marker)


                    else:
                        for simplex in hull.vertices:
                            hull_list.append((xy[simplex, 0], xy[simplex, 1]))

                        nphull = np.asarray(hull_list,dtype=np.float32)
                        # print('nphull', nphull)
                        rect = cv2.minAreaRect(nphull)
                        box = cv2.boxPoints(rect)
                        marker = Marker()
                        marker.header.frame_id = "/velodyne"
                        marker.type = marker.LINE_STRIP
                        marker.action = marker.ADD
                        marker.ns = "my_namespace";

                        obj = Obstacle()
                        obj.header.frame_id = "/velodyne"
                        obj.header.stamp = rospy.Time.now()
                        obj.ns = "my_namespace";

                        # marker scale
                        marker.scale.x = 0.3
                        marker.scale.y = 0.3
                        marker.scale.z = 0.3

                        # marker color
                        marker.color.a = 1.0
                        marker.color.r = 1.0
                        marker.color.g = 0.0
                        marker.color.b = 0.0

                        # marker orientaiton
                        marker.pose.orientation.x = 0.0
                        marker.pose.orientation.y = 0.0
                        marker.pose.orientation.z = 0.0
                        marker.pose.orientation.w = 1.0

                        # marker position
                        marker.pose.position.x = 0.0
                        marker.pose.position.y = 0.0
                        marker.pose.position.z = 0

                        # object orientaiton
                        q = quaternion_from_euler(0, 0, math.radians(rect[2]))
                        obj.pose.orientation.x = q[0]
                        obj.pose.orientation.y = q[1]
                        obj.pose.orientation.z = q[2]
                        obj.pose.orientation.w = q[3]

                        # object position
                        obj.pose.position.x = rect[0][0]
                        obj.pose.position.y = rect[0][1]
                        obj.pose.position.z = 0.0

                        
                        obj.scale.x = rect[1][0]
                        obj.scale.y = rect[1][1]
                        obj.scale.z = 2.0

                        obj.id = id_obj

                        obj.color.r = 255
                        obj.color.g = 0
                        obj.color.b = 0
                        obj.color.a = 255

                        obj.track_status = 1   ############## important 1

                        # marker line points
                        marker.points = []

                        for point in box:
                            line_point = Point()
                            line_point.x = point[0]
                            line_point.y = point[1]
                            line_point.z = 0.0
                            marker.points.append(line_point)
                        line_point_f = Point()
                        line_point_f.x = box[0][0]
                        line_point_f.y = box[0][1]
                        line_point_f.z = 0.0
                        marker.points.append(line_point_f) 

                        marker.lifetime = t

                        obj.lifetime = t
                        obj.type = -1
                        obj.animation_speed = 0.5;

                        markerArray.markers.append(marker)
                        obstacle_Array.obstacle.append(obj)

                        id_obj+=1


            else:

                if(a >= 1 and a <=3):
                    marker = Marker()
                    marker.header.frame_id = "/velodyne"
                    marker.type = marker.LINE_STRIP
                    marker.action = marker.ADD
                    marker.ns = "my_namespace";

                    # marker scale
                    marker.scale.x = 1.3
                    marker.scale.y = 1.3
                    marker.scale.z = 1.3

                    # marker color
                    marker.color.a = 1.0
                    marker.color.r = 1.0
                    marker.color.g = 0.0
                    marker.color.b = 0.0

                    # marker orientaiton
                    marker.pose.orientation.x = 0.0
                    marker.pose.orientation.y = 0.0
                    marker.pose.orientation.z = 0.0
                    marker.pose.orientation.w = 1.0

                    # marker position
                    marker.pose.position.x = 0.0
                    marker.pose.position.y = 0.0
                    marker.pose.position.z = 0.0
                    # marker line points
                    marker.points = []

                    for point in xy:

                        line_point_1 = Point()
                        line_point_1.x = point[0]
                        line_point_1.y = point[1]
                        line_point_1.z = 0.0
                        marker.points.append(line_point_1) 
                    # print(" ")
                    # print("max ",np.amax(xy,axis=0))
                    # print("min ",np.amin(xy,axis=0))

                    pmax = np.amax(xy,axis=0)
                    pmin = np.amin(xy,axis=0)


                    obj = Obstacle()
                    obj.header.frame_id = "/velodyne"
                    obj.header.stamp = rospy.Time.now()
                    obj.ns = "my_namespace";


                    # object orientaiton
                    obj.pose.orientation.x = 0
                    obj.pose.orientation.y = 0
                    obj.pose.orientation.z = 0
                    obj.pose.orientation.w = 1

                    # object position
                    obj.pose.position.x = (pmax[0] + pmin[0])/2.0
                    obj.pose.position.y = (pmax[1] + pmin[1])/2.0
                    obj.pose.position.z = 0.0

                    
                    obj.scale.x = 0.2
                    obj.scale.y = 0.2
                    obj.scale.z = 2.0

                    obj.id = id_obj

                    obj.color.r = 0
                    obj.color.g = 255
                    obj.color.b = 0
                    obj.color.a = 255

                    obj.track_status = 1   ############## important 1

                   
                    obj.lifetime = t
                    obj.type = -1
                    obj.animation_speed = 0.5;

                    obstacle_Array.obstacle.append(obj)

                    id_obj+=1





                    for i in range(4-a):
                        line_point_2 = Point()
                        line_point_2.x = xy[0][0]+i
                        line_point_2.y = xy[0][1]+i
                        line_point_2.z = 0.0
                        marker.points.append(line_point_2)
                    marker.lifetime = t
                    markerArray.markers.append(marker)

                else:
  
                    poly= approximate_polygon(xy,  tolerance=1)





    id = 0
    for m in markerArray.markers:
       m.id = id
       id += 1

    pub.publish(markerArray)
    pub_obj.publish(obstacle_Array)


def listener():

    rospy.init_node('dbscan_listener', anonymous=True)
    rospy.Subscriber("velodyne_obstacles", PointCloud2, on_new_point_cloud)


    rospy.spin()


if __name__ == '__main__':
    listener()