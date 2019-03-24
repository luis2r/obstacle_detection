#!/usr/bin/env python
# license removed for brevity
import rospy
# import pcl
from sensor_msgs.msg import PointCloud2, PointField
# import sensor_msgs.point_cloud2 as pc2
import ros_numpy
import numpy as np

import rospy
import struct

from sensor_msgs import point_cloud2
# from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

# import rospy
# import pcl
# import numpy as np
# import ctypes
# import struct
# import sensor_msgs.point_cloud2 as pc2

# from sensor_msgs.msg import PointCloud2, PointField
# from std_msgs.msg import Header
from random import randint

from sklearn.cluster import DBSCAN
from sklearn import metrics
from sklearn.datasets.samples_generator import make_blobs
from sklearn.preprocessing import StandardScaler

import matplotlib.pyplot as plt

from scipy.spatial import ConvexHull

from visualization_msgs.msg import *

from geometry_msgs.msg import Point
from skimage.measure import find_contours, approximate_polygon

import cv2
# pub = rospy.Publisher('/velodyne_markers', MarkerArray, queue_size=1000)
pub = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size = 1)

# markerArray = MarkerArray()



# def xyz_array_to_pointcloud2(points):

#     # timestamp = rospy.Time.from_sec(utime / 1e6)

#     msg = PointCloud2()
#     # msg.header.stamp = timestamp
#     msg.header.frame_id = 'velodyne'

#     # num_values = points.shape[0]
#     num_values = len(points)
#     print(num_values)
#     print(points)

#     # assert(num_values > 0)

#     NUM_FIELDS = 6
#     # assert(np.mod(num_values, NUM_FIELDS) == 0)

#     num_points = num_values / NUM_FIELDS



#     # assert(len(points.shape) == 1)
#     # assert(len(points.shape) == 1)

#     msg.height = 1

#     FLOAT_SIZE_BYTES = 4
#     msg.width = num_values * FLOAT_SIZE_BYTES

#     msg.fields = [
#         PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
#         PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
#         PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
#         PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
#     ]

#     msg.is_bigendian = False
#     msg.point_step = NUM_FIELDS * FLOAT_SIZE_BYTES

#     msg.row_step = msg.point_step * num_points
#     msg.is_dense = False

#     msg.width = num_points
#     msg.data = np.asarray(points, np.float32).tostring()

#     # return timestamp, msg
#     return msg



def on_new_point_cloud(data):
    # markerArray.markers=[]
    # print('subscribing')
    # t = rospy.Duration(0.21)    plt.clf()
    t = rospy.Duration(0.21)    

    pc = ros_numpy.numpify(data)
    # points=np.zeros((pc.shape[0],2))
    points=np.zeros((pc.shape[0],3))

    points[:,0]=pc['x']
    points[:,1]=pc['y']
    points[:,2]=pc['z']
    # print('points')
    # print(points)


    # #############################################################################
    # Compute DBSCAN
    db = DBSCAN(eps=0.7, min_samples=3).fit(points)
    core_samples_mask = np.zeros_like(db.labels_, dtype=bool)
    core_samples_mask[db.core_sample_indices_] = True
    labels = db.labels_
    # print('labels: ')
    # print(labels.size)


    # Number of clusters in labels, ignoring noise if present.
    n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
    n_noise_ = list(labels).count(-1)

    # print('Estimated number of clusters: %d' % n_clusters_)
    # print('Estimated number of noise points: %d' % n_noise_)


    # p = pcl.PointCloud(np.array(points, dtype=np.float32))




    ##############################################################################
    ##### Plot result
    unique_labels = set(labels)

    # XYZRGB_cloud = pcl.PointCloud_PointXYZRGB()
    points_cloud = []
    # lim = 8
    for k in unique_labels:
        class_member_mask = (labels == k)
        xy = points[class_member_mask & core_samples_mask]
        # 

        # xy = np.concatenate((xy[:, 0], xy[:, 1]), axis=0)
        # cl=np.zeros(xy.shape[0],dtype=[('x',np.float32),('y',np.float32)])
        # cl['x'] = xy[:,0]
        # cl['y'] = xy[:,1]
        r_r, g_r, b_r = np.random.uniform(0, 1, 3)
        # col = (r, g, b, 1)
        # a,b = xy.shape


        # for point in xy:
        #     points_list.append([point[0], point[1], 0.0, r,g,b])

        # # XYZRGB_cloud.from_list(points_list)
        # msg = xyz_array_to_pointcloud2(points_list)


        for point in xy:
            x = point[0]
            y = point[1]
            z = point[2]
            pt = [x, y, z, 0]
            r = int(r_r * 255.0)
            g = int(g_r * 255.0)
            b = int(b_r * 255.0)
            a = 255
            # print r, g, b, a
            rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
            # print hex(rgb)
            pt[3] = rgb
            points_cloud.append(pt)

    fields = [PointField('x', 0, PointField.FLOAT32, 1),
    PointField('y', 4, PointField.FLOAT32, 1),
    PointField('z', 8, PointField.FLOAT32, 1),
    PointField('rgb', 12, PointField.UINT32, 1)
    ]
                  # PointField('rgba', 12, PointField.UINT32, 1),

    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "velodyne"
    pc2 = point_cloud2.create_cloud(header, fields, points_cloud, )
                  

    pub.publish(pc2)
            # Publish the MarkerArray
    # pub.publish(markerArray)
    # markerArray.markers.pop(0)
    # markerArray.markers.clear()

        # count += 1
        # print("*********publish************")
        # publisher.publish(marker_ests)




def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('dbscan_listener_to_color_cloud', anonymous=True)
    rospy.Subscriber("velodyne_obstacles", PointCloud2, on_new_point_cloud)

    # objects_publisher = rospy.Publisher("/pcl_objects", PointCloud2, queue_size = 1)
    # table_publisher = rospy.Publisher("/pcl_table", PointCloud2, queue_size = 1)



    # plt.ion()
    # plt.show()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    # while not rospy.is_shutdown():
    #     # do whatever you want here
    #     # pub.publish(markerArray)
    #     rospy.sleep(0.1)  # sleep for one second

if __name__ == '__main__':
    listener()