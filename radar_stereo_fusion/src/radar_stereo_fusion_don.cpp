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

#include <pcl/filters/median_filter.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/io/pcd_io.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/don.h>


#include <boost/foreach.hpp>

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>

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
//typedef pcl::PointCloud<pcl::PointXYZ> Cloud;
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

    ///segment scene into clusters with given distance tolerance using euclidean clustering
    double segradius = 10.0;

void callbackRadar(const cari_msgs_perception::ObstacleArray::ConstPtr& msg_radar) {
    radarDataArray_ = msg_radar;
    //    printf("Cloud: width = %d, height = %d\n", msg->width, msg->height);
    //    BOOST_FOREACH(const pcl::PointXYZRGB& pt, msg->points)
    //    printf("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);

    //    cari_msgs_perception::Obstacle data_;
    //    cari_msgs_perception::Obstacle data2_;
    //    
    //    BOOST_FOREACH(const cari_msgs_perception::Obstacle& obstacle, radarDataArray_){
    //        std::cout << obstacle->pose.position.x << std::endl;
    //    }

    //    for (int i = 1; i < radarDataArray_->obstacle.size(); ++i) {
    //        if (data == NULL) {
    //
    //            continue;
    //        }
    //ROS_INFO_STREAM("radar array");
    //ROS_INFO(" for %f\n", radarDataArray_->obstacle[i].pose.position.x);
    //        data_ = radarDataArray_->obstacle[i];
    //    }
}

void callback(const PointCloud::ConstPtr& msg) {
    //    printf("Cloud: width = %d, height = %d\n", msg->width, msg->height);
    //    BOOST_FOREACH(const pcl::PointXYZRGB& pt, msg->points)
    //    printf("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);


                    // Create the median filtering object
                //    pcl::PassThrough<pcl::PointXYZRGBA> pass;
//                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_media(new pcl::PointCloud<pcl::PointXYZ>);
//                pcl::MedianFilter<pcl::PointXYZ> media;
//                media.setInputCloud(msg);
//                media.filter(*cloud_filtered_media);
                
                
                

                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ> ());
                pcl::VoxelGrid<pcl::PointXYZ> vg;
                vg.setInputCloud(msg);
//                vg.setInputCloud(cloud_filtered_media);
                vg.setLeafSize(0.1f, 0.1f, 0.1f);
//                vg.setFilterFieldName("y"); //////////////////////////////////stereo
//                vg.setFilterLimits(pt_cv5.y - 3, pt_cv5.y + 3); ///////////////stereo
                //                vg.setFilterFieldName("y"); ////////////////velodyne
                //                vg.setFilterLimits(pt_cv5.y - 3, pt_cv5.y + 3); /////////////velodyne
                vg.filter(*cloud_filtered);
    
    
    
    
    
    // Create a search tree, use KDTreee for non-organized data.
    pcl::search::Search<pcl::PointXYZ>::Ptr tree;
    if (cloud_filtered->isOrganized()) {
        tree.reset(new pcl::search::OrganizedNeighbor<pcl::PointXYZ> ());
    } else {
        tree.reset(new pcl::search::KdTree<pcl::PointXYZ> (false));
    }

    // Set the input pointcloud for the search tree
    tree->setInputCloud(cloud_filtered);

    if (scale1 >= scale2) {
        std::cerr << "Error: Large scale must be > small scale!" << std::endl;
        exit(EXIT_FAILURE);
    }

    // Compute normals using both small and large scales at each point
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::PointNormal> ne;
    ne.setInputCloud(cloud_filtered);
    ne.setSearchMethod(tree);

    /**
     * NOTE: setting viewpoint is very important, so that we can ensure
     * normals are all pointed in the same direction!
     */
    ne.setViewPoint(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());

    // calculate normals with the small scale
//    std::cout << "Calculating normals for scale..." << scale1 << std::endl;
    pcl::PointCloud<pcl::PointNormal>::Ptr normals_small_scale(new pcl::PointCloud<pcl::PointNormal>);

    ne.setRadiusSearch(scale1);
    ne.compute(*normals_small_scale);

    // calculate normals with the large scale
//    std::cout << "Calculating normals for scale..." << scale2 << std::endl;
    pcl::PointCloud<pcl::PointNormal>::Ptr normals_large_scale(new pcl::PointCloud<pcl::PointNormal>);

    ne.setRadiusSearch(scale2);
    ne.compute(*normals_large_scale);

    // Create output cloud for DoN results
    pcl::PointCloud<pcl::PointNormal>::Ptr doncloud(new pcl::PointCloud<pcl::PointNormal>);
    pcl::copyPointCloud<pcl::PointXYZ, pcl::PointNormal>(*cloud_filtered, *doncloud);

//    std::cout << "Calculating DoN... " << std::endl;
    // Create DoN operator
    pcl::DifferenceOfNormalsEstimation<pcl::PointXYZ, pcl::PointNormal, pcl::PointNormal> don;
    don.setInputCloud(cloud_filtered);
    don.setNormalScaleLarge(normals_large_scale);
    don.setNormalScaleSmall(normals_small_scale);

    if (!don.initCompute()) {
//        std::cerr << "Error: Could not intialize DoN feature operator" << std::endl;
        exit(EXIT_FAILURE);
    }

    // Compute DoN
    don.computeFeature(*doncloud);

    // Save DoN features
//    pcl::PCDWriter writer;
//    writer.write<pcl::PointNormal> ("don.pcd", *doncloud, false);

    // Filter by magnitude
//    std::cout << "Filtering out DoN mag <= " << threshold << "..." << std::endl;

    // Build the condition for filtering
    pcl::ConditionOr<pcl::PointNormal>::Ptr range_cond(
            new pcl::ConditionOr<pcl::PointNormal> ()
            );
    range_cond->addComparison(pcl::FieldComparison<pcl::PointNormal>::ConstPtr(
            new pcl::FieldComparison<pcl::PointNormal> ("curvature", pcl::ComparisonOps::GT, threshold))
            );
    // Build the filter
    pcl::ConditionalRemoval<pcl::PointNormal> condrem(range_cond);
    condrem.setInputCloud(doncloud);

    pcl::PointCloud<pcl::PointNormal>::Ptr doncloud_filtered(new pcl::PointCloud<pcl::PointNormal>);

    // Apply filter
    condrem.filter(*doncloud_filtered);

    doncloud = doncloud_filtered;

    // Save filtered output
//    std::cout << "Filtered Pointcloud: " << doncloud->points.size() << " data points." << std::endl;
  pcl::PointNormal minPt, maxPt;
  pcl::getMinMax3D (*doncloud_filtered, minPt, maxPt);
  std::cout << "Max x: " << maxPt.x << std::endl;
  std::cout << "Max y: " << maxPt.y << std::endl;
  std::cout << "Max z: " << maxPt.z << std::endl;
  std::cout << "Min x: " << minPt.x << std::endl;
  std::cout << "Min y: " << minPt.y << std::endl;
  std::cout << "Min z: " << minPt.z << std::endl;
  
  msg_box3d::Num minmax; 
  
  minmax.max_x = maxPt.x;
  minmax.max_y = maxPt.y;
  minmax.max_z = maxPt.z;
  minmax.min_x = minPt.x;
  minmax.min_y = minPt.y;
  minmax.min_z = minPt.z;   
  
     
  minmax.header.frame_id =    msg->header.frame_id;
//  minmax.header.seq =    msg->header.seq;
//  minmax.header.stamp =    msg->header.stamp;
    
    
//    cloud_cluster->width = cloud_cluster->points.size();
//    cloud_cluster->height = 1;
//    cloud_cluster->is_dense = true;
    doncloud->header = msg->header;
    
    maxmin_pub.publish(minmax);
    pub_cluster.publish(doncloud);

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pub_pcl");
    ros::NodeHandle nh;
//    pub = nh.advertise<PointCloudRGB> ("points2", 1);
    pub_cluster = nh.advertise<PointCloudCluster> ("cluster1", 1);
    maxmin_pub = nh.advertise<msg_box3d::Num>("cloud_max_min", 1);
//    pub_radar_cloud = nh.advertise<PointCloudRadar> ("pointsRadar", 1);
    //    ind_pub = nh.advertise< PointCloudCluster>("point_indices",1);
    //        ros::Subscriber sub = nh.subscribe<PointCloud>("stereo/narrow/points2", 1, callback);//////////////stereo
    //    ros::Subscriber sub = nh.subscribe<PointCloud>("velodyne_points", 1, callback); /////////////////////////////////////velodyne
    ros::Subscriber sub = nh.subscribe<PointCloud>("passthrough2", 1, callback); /////////////////////////////////////velodyne
    ros::Subscriber sub_radar = nh.subscribe("radar_esr_msg_array_1", 1, callbackRadar);
    //  ros::spin();
    ros::spin();

}
