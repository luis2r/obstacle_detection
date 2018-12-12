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



#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/convex_hull.h>


#include <geometry_msgs/PointStamped.h>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


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

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

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

#include <pcl/ModelCoefficients.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/convex_hull.h>

#include <velodyne_pointcloud/point_types.h>

#include <msg_box3d/Num.h>
#include <msg_box3d/NumArray.h>

#include <ctime>
#include <cstdlib>
#include <iostream>

//#include "msg_box3d/Num.h"
//#define foreach BOOST_FOREACH
#include <cmath>
//#include <math>





//#define foreach BOOST_FOREACH

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudRGB;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudRadar;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudCluster;

ros::Publisher pub;
ros::Publisher pub_cluster;
ros::Publisher pub_convex_hull;
ros::Publisher ind_pub;
//ros::Publisher maxmin_pub;

ros::Publisher pub_marker;


void callback(const PointCloud::ConstPtr& cloud) 
{
    visualization_msgs::MarkerArray markerArray; //marker_array_msg,
    //msg_box3d::NumArray::ConstPtr nums = msg;




    // Create a search tree, use KDTreee for non-organized data.
    pcl::search::Search<pcl::PointXYZ>::Ptr tree;
    if (cloud->isOrganized()) {
        tree.reset(new pcl::search::OrganizedNeighbor<pcl::PointXYZ> ());
    } else {
        tree.reset(new pcl::search::KdTree<pcl::PointXYZ> (false));
    }
    // Set the input pointcloud for the search tree
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    //      ec.setClusterTolerance (0.02); // 2cm
    ec.setClusterTolerance(10); // 50cm

    ec.setMinClusterSize(3);
    ec.setMaxClusterSize(1000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    int j = 0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ> all_convex_hulls;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) 
    {
        int first_cloud = true;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster_aux(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);

        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) 
        {
            cloud_cluster->points.push_back(cloud->points[*pit]); //*
            cloud_cluster_aux->points.push_back(cloud->points[*pit]); //* 
        }
        int i = cloud_cluster_aux->points.size();

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ConvexHull<pcl::PointXYZ> chull;
        // chull.setInputCloud (cloud_projected);
        chull.setInputCloud (cloud_cluster_aux);
        chull.reconstruct (*cloud_hull);

        std::cerr << "Convex hull has size: " << cloud_hull->points.size () << " data points." << std::endl;
        if (cloud_hull->points.size() > 0.0f){

            std::cerr << "Cloud A: " << std::endl;
            for (size_t i = 0; i < cloud_hull->points.size (); ++i)
                std::cerr << "    " << cloud_hull->points[i].x << " " << cloud_hull->points[i].y << " " << cloud_hull->points[i].z << std::endl;
            /************************convex hull *********************************/   
            visualization_msgs::Marker /*points, line_list, */line_strip;
            line_strip.header.frame_id = cloud->header.frame_id;
            line_strip.action = visualization_msgs::Marker::ADD;
            line_strip.pose.orientation.w = 1.0;
            // %Tag(ID)%
            //points.id = 0;
            line_strip.id = j;
            line_strip.type = visualization_msgs::Marker::LINE_STRIP;
            line_strip.scale.x = 0.15;
            // Line list is red
            line_strip.color.r = 1.0;
            line_strip.color.a = 1.0;
            // %EndTag(COLOR)%

            std::vector<cv::Point2f> poly;

            for (size_t i = 0; i < cloud_hull->points.size (); ++i)
            {
                cv::Matx31d p2(cloud_hull->points[i].x, cloud_hull->points[i].y, cloud_hull->points[i].z);
                std::cout << "point convex_hull  " << p2(0) << " "   << p2(1) << " "     << p2(2) << std::endl;
                //msg_radar_to_stereo->points.push_back(pcl::PointXYZ(p2(0), p2(1), p2(2))); 
                poly.push_back(cv::Point2f(p2(0), p2(1)));
            }


            cv::RotatedRect box = cv::minAreaRect(poly); // rotating calipers

            cv::Point2f vertices[4];
            box.points(vertices);
            //std::cout << "sizeof vertices     "<< sizeof(vertices) << std::endl;
            geometry_msgs::Point p;
            for (int i = 0; i < 4; i++)
            {
                //  cv::line(image, vertices[i], vertices[(i+1)%4], Scalar(0,255,0));
                p.x = vertices[i].x;
                p.y = vertices[i].y;
                p.z = 0.0f;

                line_strip.points.push_back(p);
                std::cout << "rect  "<< i << " " <<vertices[i] << std::endl;
                //msg_radar_to_stereo->points.push_back(pcl::PointXYZ(vertices[i].x, vertices[i].y, 0.0f)); ///////////////////stereo 
            }

            line_strip.lifetime = ros::Duration(0.1);
            markerArray.markers.push_back(line_strip);
        }

    }        
            // Save filtered output
        // pcl::PointXYZ minPt, maxPt;
        // pcl::getMinMax3D(*cloud_cluster_aux, minPt, maxPt);
        // std::cout << "Max x: " << maxPt.x << std::endl;
        // msg_box3d::Num minmax;
        // minmax.max_x = maxPt.x;
        // minmax.max_y = maxPt.y;
        // minmax.max_z = maxPt.z;
        // minmax.min_x = minPt.x;
        // minmax.min_y = minPt.y;
        // minmax.min_z = minPt.z;
        // minmax.header.frame_id = cloud->header.frame_id;
        // array_minmax.Nums.push_back(minmax);

    
    pub_marker.publish(markerArray);
    //markerArray.markers.clear();
    //std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << std::endl;
    //std::stringstream ss;
    //ss << "cloud_cluster_" << j << ".pcd";
    //        writer.write<pcl::PointXYZ> (ss.str(), *cloud_cluster, false); //*
    j++;
    cloud_cluster->width = cloud_cluster->points.size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = false;
    cloud_cluster->header = cloud->header;

    pub_cluster.publish(cloud_cluster);
    //maxmin_pub.publish(array_minmax);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pub_pcl");
    ros::NodeHandle nh;
    pub_cluster = nh.advertise<PointCloudCluster> ("cluster1", 10);
    //ros::Subscriber sub = nh.subscribe<PointCloud>("passthrough2", 1, callback); /////////////////////////////////////velodyne
    ros::Subscriber sub = nh.subscribe<PointCloud>("velodyne_obstacles", 1, callback); /////////////////////////////////////velodyne
    //ros::Subscriber sub_radar = nh.subscribe("radar_esr_msg_array_1", 1, callbackRadar);
    //maxmin_pub = nh.advertise<msg_box3d::NumArray>("array_cloud_max_min", 10);
    pub_convex_hull = nh.advertise<PointCloud> ("convex_hull", 10);


    pub_marker = nh.advertise<visualization_msgs::MarkerArray>("bounding_box", 1);

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
