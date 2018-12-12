#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/min_cut_segmentation.h>
#include <pcl/filters/passthrough.h>

#include <image_geometry/pinhole_camera_model.h>
#include <geometry_msgs/PointStamped.h>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cari_msgs_perception/ObstacleArray.h>
#include <cari_msgs_perception/Obstacle.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/filters/median_filter.h>

#include <pcl/features/don.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <boost/foreach.hpp>

#include <ctime>
#include <cstdlib>
#include <iostream>
#include <msg_box3d/Num.h>
#include <msg_box3d/NumArray.h>
//#define foreach BOOST_FOREACH

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudRGB;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudRadar;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudCluster;

ros::Publisher pub;
ros::Publisher pub_cluster;
ros::Publisher pub_radar_cloud;
ros::Publisher ind_pub;
ros::Publisher maxmin_pub;

cari_msgs_perception::ObstacleArray::ConstPtr radarDataArray_;

///The smallest scale to use in the DoN filter.
double scale1 = 0.7;

///The largest scale to use in the DoN filter.
double scale2 = 7.0;

///The minimum DoN magnitude to threshold by
double threshold = 0.2;

void callbackRadar(const cari_msgs_perception::ObstacleArray::ConstPtr& msg_radar) {
    radarDataArray_ = msg_radar;
}

void callback(const PointCloud::ConstPtr& cloud) {
        msg_box3d::NumArray array_minmax;

    pcl::PointCloud<pcl::PointXYZ>::Ptr /*cloud(new pcl::PointCloud<pcl::PointXYZ>),*/ cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
    //    reader.read("/home/luis/NetBeansProjects/pcl-pcd-read/kitti.pcd", *cloud);
    //  reader.read ("/home/luis/NetBeansProjects/pcl-pcd-read/table_scene_lms400.pcd", *cloud);
    //    std::cout << "PointCloud before filtering has: " << cloud->points.size() << " data points." << std::endl; //*

    //     Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud(cloud);
    //  vg.setLeafSize (0.05f, 0.05f, 0.05f);
    vg.setLeafSize(0.02f, 0.02f, 0.02f);
    vg.filter(*cloud_filtered);
    //    std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size() << " data points." << std::endl; //*

    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PCDWriter writer;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
//          seg.setDistanceThreshold (0.02);
    seg.setDistanceThreshold(0.2);

    int i = 0, nr_points = (int) cloud_filtered->points.size();
    while (cloud_filtered->points.size() > 0.6 * nr_points) {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud(cloud_filtered);

        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0) {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers);
        extract.setNegative(false);

        // Get the points associated with the planar surface
        extract.filter(*cloud_plane);

        std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points." << std::endl;

        // Remove the planar inliers, extract the rest
        extract.setNegative(true);
        extract.filter(*cloud_f);
        *cloud_filtered = *cloud_f;
    }

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_filtered);


    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    //      ec.setClusterTolerance (0.02); // 2cm
    ec.setClusterTolerance(0.5); // 30cm

    ec.setMinClusterSize(5);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_filtered);
    ec.extract(cluster_indices);

    int j = 0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster_aux(new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
            cloud_cluster->points.push_back(cloud_filtered->points[*pit]); //*
            cloud_cluster_aux->points.push_back(cloud_filtered->points[*pit]); //* 
        }
                            // Save filtered output
                    //    std::cout << "Filtered Pointcloud: " << doncloud->points.size() << " data points." << std::endl;
                    pcl::PointXYZ minPt, maxPt;
                    //                    pcl::getMinMax3D(*cloud_cluster_aux, minPt, maxPt);
                    pcl::getMinMax3D(*cloud_cluster_aux, minPt, maxPt);
                    std::cout << "Max x: " << maxPt.x << std::endl;

                    msg_box3d::Num minmax;

                    minmax.max_x = maxPt.x;
                    minmax.max_y = maxPt.y;
                    minmax.max_z = maxPt.z;
                    minmax.min_x = minPt.x;
                    minmax.min_y = minPt.y;
                    minmax.min_z = minPt.z;

                    minmax.header.frame_id = cloud->header.frame_id;

                    array_minmax.Nums.push_back(minmax);
                    std::cout << "xxxs " << maxPt.x << std::endl;



    }

    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << std::endl;
    std::stringstream ss;
    ss << "cloud_cluster_" << j << ".pcd";
    //        writer.write<pcl::PointXYZ> (ss.str(), *cloud_cluster, false); //*
    j++;
    cloud_cluster->width = cloud_cluster->points.size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    cloud_cluster->header = cloud->header;
    pub_cluster.publish(cloud_cluster);
    maxmin_pub.publish(array_minmax);








}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pub_pcl");
    ros::NodeHandle nh;
    pub_cluster = nh.advertise<PointCloudCluster> ("cluster1", 1);
    ros::Subscriber sub = nh.subscribe<PointCloud>("passthrough2", 1, callback); /////////////////////////////////////velodyne
    ros::Subscriber sub_radar = nh.subscribe("radar_esr_msg_array_1", 1, callbackRadar);
    maxmin_pub = nh.advertise<msg_box3d::NumArray>("array_cloud_max_min", 1);
    pub_radar_cloud = nh.advertise<PointCloudRadar> ("pointsRadar", 10);
    //  ros::spin();
    ros::spin();

}

//                point.x = pt_cv5.x;
//                point.y = pt_cv5.y - 0.5; //stereo
//                //                point.y = pt_cv5.y; //velodyne
//                point.z = pt_cv5.z + 0.2;
//                foreground_points->points.push_back(point);
//                seg.setForegroundPoints(foreground_points);
//                seg.setSigma(0.3); //velodyne
//                //                seg.setSigma(0.2);
//                seg.setRadius(2.0);
//                //    seg.setRadius(0.5);
//                seg.setNumberOfNeighbours(14);
//                seg.setSourceWeight(0.8);
