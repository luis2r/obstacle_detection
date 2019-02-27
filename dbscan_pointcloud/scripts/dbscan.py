#!/usr/bin/env python
# license removed for brevity
import rospy
# import pcl
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import ros_numpy
import numpy as np

from sklearn.cluster import DBSCAN
from sklearn import metrics
from sklearn.datasets.samples_generator import make_blobs
from sklearn.preprocessing import StandardScaler

def on_new_point_cloud(data):

    pc = ros_numpy.numpify(data)
    points=np.zeros((pc.shape[0],3))
    points[:,0]=pc['x']
    points[:,1]=pc['y']
    points[:,2]=pc['z']
    # #############################################################################
    # Compute DBSCAN
    db = DBSCAN(eps=0.5, min_samples=3).fit(points)
    core_samples_mask = np.zeros_like(db.labels_, dtype=bool)
    core_samples_mask[db.core_sample_indices_] = True
    labels = db.labels_
    print('labels: ')
    print(labels.size)


    # Number of clusters in labels, ignoring noise if present.
    n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
    n_noise_ = list(labels).count(-1)

    print('Estimated number of clusters: %d' % n_clusters_)
    # print('Estimated number of noise points: %d' % n_noise_)


    # p = pcl.PointCloud(np.array(points, dtype=np.float32))




    # #############################################################################
    # Plot result
    import matplotlib.pyplot as plt
    # Black removed and is used for noise instead.
    unique_labels = set(labels)
    colors = [plt.cm.Spectral(each)
              for each in np.linspace(0, 1, len(unique_labels))]
    for k, col in zip(unique_labels, colors):
        if k == -1:
            col = [0, 0, 0, 1]
        else:
            r, g, b = np.random.uniform(0, 1, 3)
            col = (r, g, b, 1)
        class_member_mask = (labels == k)
        xy = points[class_member_mask & core_samples_mask]
        plt.plot(xy[:, 0], xy[:, 1], '.', fillstyle='full', markeredgewidth=0, markerfacecolor=tuple(col), markersize=10)
        xy = points[class_member_mask & ~core_samples_mask]
        plt.plot(xy[:, 0], xy[:, 1], 'o', fillstyle='full', markerfacecolor=tuple(col),markersize=2)
    plt.title('Estimated number of clusters: %d' % n_clusters_)
    plt.show()

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('dbscan_listener', anonymous=True)
    rospy.Subscriber("velodyne_obstacles", PointCloud2, on_new_point_cloud)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()