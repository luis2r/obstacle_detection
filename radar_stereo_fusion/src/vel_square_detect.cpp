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

typedef pcl::PointCloud<velodyne_pointcloud::PointXYZIR> PointCloud1;


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudRGB;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudRadar;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudCluster;
//typedef pcl::PointCloud<pcl::PointXYZ> Cloud;
ros::Publisher pub;
ros::Publisher pub_cluster;
ros::Publisher pub_convex_hull;
ros::Publisher pub_radar_cloud;
ros::Publisher ind_pub;
ros::Publisher maxmin_pub;
cari_msgs_perception::ObstacleArray::ConstPtr radarDataArray_;

///The smallest scale to use in the DoN filter.
//    double scale1 = 0.7;
double scale1 = 0.5;
///The largest scale to use in the DoN filter.
//    double scale2 = 7.0;
double scale2 = 5.0;
///The minimum DoN magnitude to threshold by
double threshold = 0.4;

///segment scene into clusters with given distance tolerance using euclidean clustering
double segradius = 10.0;

//void callbackRadar(const cari_msgs_perception::ObstacleArray::ConstPtr& msg_radar) {
//    radarDataArray_ = msg_radar;
//        printf("Cloud: width = %d, height = %d\n", msg->width, msg->height);
//        BOOST_FOREACH(const pcl::PointXYZRGB& pt, msg->points)
//        printf("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
//
//        cari_msgs_perception::Obstacle data_;
//        cari_msgs_perception::Obstacle data2_;
//        
//        BOOST_FOREACH(const cari_msgs_perception::Obstacle& obstacle, radarDataArray_){
//            std::cout << obstacle->pose.position.x << std::endl;
//        }
//
//        for (int i = 1; i < radarDataArray_->obstacle.size(); ++i) {
//            if (data == NULL) {
//    
//                continue;
//            }
//    ROS_INFO_STREAM("radar array");
//    ROS_INFO(" for %f\n", radarDataArray_->obstacle[i].pose.position.x);
//            data_ = radarDataArray_->obstacle[i];
//        }
//}

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




    //                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ> ());
    //                pcl::VoxelGrid<pcl::PointXYZ> vg;
    //                vg.setInputCloud(msg);
    ////                vg.setInputCloud(cloud_filtered_media);
    ////                vg.setLeafSize(0.01f, 0.01f, 0.01f);
    ////                vg.setFilterFieldName("y"); //////////////////////////////////stereo
    ////                vg.setFilterLimits(pt_cv5.y - 3, pt_cv5.y + 3); ///////////////stereo
    //                                vg.setFilterFieldName("y"); ////////////////velodyne
    //                                vg.setFilterLimits(- 8,  8); /////////////velodyne
    //                vg.filter(*cloud_filtered);
    //    




    // Create a search tree, use KDTreee for non-organized data.
    pcl::search::Search<pcl::PointXYZ>::Ptr tree;
    //    if (cloud_filtered->isOrganized()) {
    if (msg->isOrganized()) {
        tree.reset(new pcl::search::OrganizedNeighbor<pcl::PointXYZ> ());
    } else {
        tree.reset(new pcl::search::KdTree<pcl::PointXYZ> (false));
    }

    // Set the input pointcloud for the search tree
    tree->setInputCloud(msg);
    //    tree->setInputCloud(cloud_filtered);

    if (scale1 >= scale2) {
        std::cerr << "Error: Large scale must be > small scale!" << std::endl;
        exit(EXIT_FAILURE);
    }

    // Compute normals using both small and large scales at each point
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::PointNormal> ne;
    ne.setInputCloud(msg);
    //    ne.setInputCloud(cloud_filtered);
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
    pcl::copyPointCloud<pcl::PointXYZ, pcl::PointNormal>(*msg, *doncloud);
    //    pcl::copyPointCloud<pcl::PointXYZ, pcl::PointNormal>(*cloud_filtered, *doncloud);
    //    std::cout << "Calculating DoN... " << std::endl;
    // Create DoN operator
    pcl::DifferenceOfNormalsEstimation<pcl::PointXYZ, pcl::PointNormal, pcl::PointNormal> don;
    don.setInputCloud(msg);
    //    don.setInputCloud(cloud_filtered);
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



    //    // Create statistical outlayer removal the filtering object
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_filtered2(new pcl::PointCloud<pcl::PointNormal>);
    pcl::StatisticalOutlierRemoval<pcl::PointNormal> sor;
    sor.setInputCloud(doncloud);
//    sor.setMeanK(50);//numero de puntos analizados
    sor.setMeanK(20);
//    sor.setStddevMulThresh(1.0);
    sor.setStddevMulThresh(0.4);//desviacion estandar, los que esta fuera son eliminadoss
    sor.filter(*cloud_filtered2);


    //      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointNormal> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
//    seg.setDistanceThreshold(0.12);
    seg.setDistanceThreshold(0.10);
    seg.setInputCloud(cloud_filtered2);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0) {
        PCL_ERROR("Could not estimate a planar model for the given dataset.");
        return;
    }

    double a = coefficients->values[0];
    double b = coefficients->values[1];
    double c = coefficients->values[2];
    double d = coefficients->values[3];

//    std::cerr << "Model coefficients: " << coefficients->values[0] << " "
//            << coefficients->values[1] << " "
//            << coefficients->values[2] << " "
//            << coefficients->values[3] << std::endl;
    //
    //    std::cerr << "Model inliers: " << inliers->indices.size() << std::endl;
    //    for (size_t i = 0; i < inliers->indices.size(); ++i)
    //        std::cerr << inliers->indices[i] << "    " << cloud_filtered2->points[inliers->indices[i]].x << " "
    //            << cloud_filtered2->points[inliers->indices[i]].y << " "
    //            << cloud_filtered2->points[inliers->indices[i]].z << std::endl;

    pcl::PointCloud<pcl::PointNormal>::Ptr cloud2(new pcl::PointCloud<pcl::PointNormal>), cloud_projected(new pcl::PointCloud<pcl::PointNormal>);

    // Project the model inliers
    pcl::ProjectInliers<pcl::PointNormal> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(cloud_filtered2);
    proj.setIndices(inliers);
    proj.setModelCoefficients(coefficients);
    proj.filter(*cloud_projected);

    // Create a Convex Hull representation of the projected inliers
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointNormal>);
    pcl::ConvexHull<pcl::PointNormal> chull;
    chull.setInputCloud(cloud_projected);
//    chull.reconstruct(*cloud_projected);
        chull.reconstruct(*cloud_hull);

//    std::cerr << "Convex hull has: " << cloud_projected->points.size() << " data points." << std::endl;
    //    std::cerr << "Convex hull has: " << cloud_hull->points.size() << " data points." << std::endl;



//    pcl::PointNormal minPt, maxPt;
//    pcl::getMinMax3D(*doncloud_filtered, minPt, maxPt);
    //    std::cout << "Max x: " << maxPt.x << std::endl;
    //    std::cout << "Max y: " << maxPt.y << std::endl;
    //    std::cout << "Max z: " << maxPt.z << std::endl;
    //    std::cout << "Min x: " << minPt.x << std::endl;
    //    std::cout << "Min y: " << minPt.y << std::endl;
    //    std::cout << "Min z: " << minPt.z << std::endl;

//    msg_box3d::Num minmax;
//
//    minmax.max_x = maxPt.x;
//    minmax.max_y = maxPt.y;
//    minmax.max_z = maxPt.z;
//    minmax.min_x = minPt.x;
//    minmax.min_y = minPt.y;
//    minmax.min_z = minPt.z;


    //    interseccion del plano con el eje y
    double x = 0;
    double z = 0;
//    double y = -(a*x + c*z + d) / b ;
    double y = -d / b ;
    int y_intercept = y;
    std::cerr << "interseccion del plano con el eje y "  << std::endl;    //    interseccion del plano con el eje y
std::cerr << "point y axis: " << x << " " << y << " " << z << std::endl;
    double xplus1 = x;
    double yplus1 = y + 1;
    double zplus1 = z;
std::cerr << "point y+1 axis: " << xplus1 << " " << yplus1 << " " << zplus1 << std::endl;
    //    proyeccion de un punto del eje y (0,1,0) sobre el plano 
    double landa = -(a*xplus1 + b*yplus1 + c*zplus1 + d) / (pow(a,2) + pow(b,2) + pow(c,2));
    double Qx = xplus1 + a * landa;
    double Qy = yplus1 + b * landa;
    double Qz = zplus1 + c * landa;
std::cerr << "Q y axis: " << Qx << " " << Qy << " " << Qz << std::endl;

    double theta_z = acos( sqrt(pow(Qx - x, 2) + pow(Qy - y, 2) + pow(Qz - z, 2)) / 
    sqrt(pow(xplus1 - x, 2) + pow(yplus1 - y, 2) + pow(zplus1 - z, 2))    ); //angulo ente el plano del patron de calib y plano XY

        
//    double theta_z = acos(  ( ((Qx-x)*(xplus1-x)) + ((Qy-y)*(yplus1-y)) + ((Qz-z)*(zplus1-z)))  /  
//            sqrt(pow(Qx-x, 2) + pow(Qy-y, 2) + pow(Qz-z, 2)) * sqrt(pow(xplus1-x, 2) + pow( yplus1-y, 2) + pow( zplus1-z, 2))     ); //angulo ente el plano del patron de calib y plano XY
//    std::cerr << "theta_z: " << theta_z << std::endl;
    //    interseccion del plano con el eje x
//    x = -(b*y + c*z + d) / a ;
    x = -d / a ;
    z = 0;
    y = 0;
        std::cerr << "interseccion del plano con el eje x " << std::endl;    //    interseccion del plano con el eje y
std::cerr << "point x axis: " << x << " " << y << " " << z << std::endl;
    xplus1 = x + 1;
    yplus1 = y;
    zplus1 = z;
std::cerr << "point x+1 axis: " << xplus1 << " " << yplus1 << " " << zplus1 << std::endl;
    //    proyeccion de un punto del eje x (1,0,0) sobre el plano 
    landa = -(a*xplus1 + b*yplus1 + c*zplus1 + d) / (pow(a,2) + pow(b,2) + pow(c,2));
    Qx = xplus1 + a * landa;
    Qy = yplus1 + b * landa;
    Qz = zplus1 + c * landa;
std::cerr << "Q x axis: " << Qx << " " << Qy << " " << Qz << std::endl;

    double theta_y = acos(  sqrt(pow(Qx - x, 2) + pow(Qy - y, 2) + pow(Qz - z, 2))  /  
            sqrt(pow(xplus1 - x, 2) + pow(yplus1 - y, 2) + pow(zplus1 - z, 2))     ); //angulo ente el plano del patron de calib y plano XY

//    double theta_y = acos(  ( ((Qx-x)*(xplus1-x)) + ((Qy-y)*(yplus1-y)) + ((Qz-z)*(zplus1-z)))  /  
//            sqrt(pow(Qx-x, 2) + pow(Qy-y, 2) + pow(Qz-z, 2)) * sqrt(pow(xplus1-x, 2) + pow( yplus1-y, 2) + pow( zplus1-z, 2))     ); //angulo ente el plano del patron de calib y plano XY
    
//std::cerr << "cos: " << sqrt(pow(Qx - x, 2) + pow(Qy - y, 2) + pow(Qz - z, 2))  /  
//            sqrt(pow(xplus1 - x, 2) + pow(yplus1 - y, 2) + pow(zplus1 - z, 2))  << std::endl;
//std::cerr << "ca: " << sqrt(pow(Qx - x, 2) + pow(Qy - y, 2) + pow(Qz - z, 2)) << std::endl;
//std::cerr << "hip: " <<  sqrt(pow(xplus1 - x, 2) + pow(yplus1 - y, 2) + pow(zplus1 - z, 2))<< std::endl;

//std::cerr << "theta_y: " << theta_y<< std::endl;




    //    // Filter by magnitude
    //    std::cout << "Clustering using EuclideanClusterExtraction with tolerance <= " << segradius << "..." << std::endl;
    //
    //    pcl::search::KdTree<pcl::PointNormal>::Ptr segtree(new pcl::search::KdTree<pcl::PointNormal>);
    //    segtree->setInputCloud(doncloud);
    //
    //    std::vector<pcl::PointIndices> cluster_indices;
    //    pcl::EuclideanClusterExtraction<pcl::PointNormal> ec;
    //
    //    ec.setClusterTolerance(segradius);
    //    ec.setMinClusterSize(50);
    //    ec.setMaxClusterSize(100000);
    //    ec.setSearchMethod(segtree);
    //    ec.setInputCloud(doncloud);
    //    ec.extract(cluster_indices);
    //
    //    int j = 0;
    //    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it, j++) {
    //        pcl::PointCloud<pcl::PointNormal>::Ptr cloud_cluster_don(new pcl::PointCloud<pcl::PointNormal>);
    //        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
    //            cloud_cluster_don->points.push_back(doncloud->points[*pit]);
    //        }
    //
    //        cloud_cluster_don->width = int (cloud_cluster_don->points.size());
    //        cloud_cluster_don->height = 1;
    //        cloud_cluster_don->is_dense = true;
    //
    //        //Save cluster
    //        std::cout << "PointCloud representing the Cluster: " << cloud_cluster_don->points.size() << " data points." << std::endl;
    ////        std::stringstream ss;
    ////        ss << "don_cluster_" << j << ".pcd";
    ////        writer.write<pcl::PointNormal> (ss.str(), *cloud_cluster_don, false);
    //    }



//    std::vector<cv::Point2f> poly;
//
//
//    poly.push_back(cv::Point2f(1.1f, 2.2f));
//    poly.push_back(cv::Point2f(3.3f, 4.4f));
//    poly.push_back(cv::Point2f(5.5f, 6.6f));
//    poly.push_back(cv::Point2f(7.7f, 8.8f));

    //    double theta_y = fabs(2 * 1 + (-1)*0 + 1 * 1) / (sqrt(pow(2, 2) + pow(-1, 2) + sqrt(pow(1, 2))) * sqrt(pow(1, 2) + pow(0, 2) + sqrt(pow(1, 2))));
    //    double theta_y = acos(
    //            fabs(a * 1 + b * 1 + c * 0) /
    //            (sqrt(pow(a, 2) + pow(b, 2) + pow(c, 2)) * sqrt(pow(1, 2) + pow(1, 2) + pow(0, 2)))); //angulo ente el plano del patron de calib y plano XY
    //    double theta_z = acos(
    //            fabs(a * 1 + b * 0 + c * 1) /
    //            (sqrt(pow(a, 2) + pow(b, 2) + pow(c, 2)) * sqrt(pow(1, 2) + pow(0, 2) + pow(1, 2)))); //angulo ente el plano del patron de calib y plano XY






    //     double theta_y = acos(
    //     abs( (1.0f * 1.0f) + 0.0f * 1.0f + 1.0 * 0.0f) / 
    //     (sqrt(  pow(1.0f, 2.0f) + pow(0.0f, 2.0f) + pow(1.0f, 2.0f)  )  *  sqrt( pow(1.0f, 2.0f) + pow(1.0f, 2.0f) + pow(0.0f, 2.0f) ))
    //     ); //angulo ente el plano del patron de calib y plano XY
    //    double theta_z = acos( abs(a * 1.0f + b * 0.0f + c * 1.0f) / (sqrt(  pow(a, 2.0f) + pow(b, 2.0f) + pow(c, 2.0f)  )  *  sqrt( pow(1.0f, 2.0f) + pow(0.0f, 2.0f) + pow(1.0f, 2.0f) ))); //angulo ente el plano del patron de calib y plano XY
    //             double abs1 = abs(1.0f * 1.0f + 0.0f * 1.0f + 1.0 * 0.0f) / (sqrt(  pow(1.0f, 2.0f) + pow(0.0f, 2.0f) + pow(1.0f, 2.0f)  )  *  sqrt( pow(1.0f, 2.0f) + pow(1.0f, 2.0f) + pow(0.0f, 2.0f) )); //angulo ente el plano del patron de calib y plano XY
    //        double abs2 = abs(a * 1.0f + b * 0.0f + c * 1.0f) / (sqrt(  pow(a, 2.0f) + pow(b, 2.0f) + pow(c, 2.0f)  )  *  sqrt( pow(1.0f, 2.0f) + pow(0.0f, 2.0f) + pow(1.0f, 2.0f) )); //angulo ente el plano del patron de calib y plano XY
    //    
    //    0.861584 -0.197896 -0.467451 -9.58593
    //        std::cout << "abs " << fabs ((0.86158 * 1.0)  - (0.197896* 0.0)   - (0.467451* 1.0))<< std::endl;
    //        std::cout << "sqrt " <<(double (sqrt(double(pow(a, 2)) + double(pow(b, 2)) + double(pow(c, 2)))) * double(sqrt(double(pow(1, 2)) + double(pow(0, 2)) + double(pow(1, 2)))))
    // << std::endl;
    ////    
//    std::cout << "a " << a << std::endl;
//    std::cout << "b " << b << std::endl;
//    std::cout << "c " << c << std::endl;
//    std::cout << "d " << d << std::endl;

    std::cout << "theta y " << theta_y << std::endl;
    std::cout << "theta z " << theta_z << std::endl;

    cv::Matx33d roty(cos(theta_y), 0,  sin(theta_y),
                0,                 1,             0,
                -sin(theta_y),     0,  cos(theta_y));



    //        cv::Matx33d roty(cos(theta_y),  -sin(theta_y), 0,
    //            sin(theta_y), cos(theta_y),0,
    //            0, 0, 1);

    //        cv::Matx33d roty(1, 0, 0,
    //                0, cos(theta_y),  -sin(theta_y),
    //                0, sin(theta_y), cos(theta_y),
    //            0, 0, 1);       
    cv::Matx33d rotz(cos(theta_z),  -sin(theta_z),     0,
                     sin(theta_z),   cos(theta_z),     0,
                     0,              0,                1);     
    if ( y_intercept >= 0){
//        std::cout << "mayor y= " << y_intercept << std::endl;
//     cv::Matx33d rotz(cos(theta_z),  -sin(theta_z),     0,
//                     sin(theta_z),   cos(theta_z),     0,
//                     0,              0,                1);   
        rotz(0,0) = cos(-theta_z);   rotz(0,1) = -sin(-theta_z);   rotz(0,2) =  0;
        rotz(1,0) = sin(-theta_z);   rotz(1,1) = cos(-theta_z);    rotz(1,2) =  0;
        rotz(2,0) = 0;              rotz(2,1) = 0;               rotz(2,2) =  1;
    }

    
//    
//    cv::Matx33d rotz(cos((2*3.1416)-theta_z),  -sin((2*3.1416)-theta_z),     0,
//                     sin((2*3.1416)-theta_z),   cos((2*3.1416)-theta_z),     0,
//                     0,              0,                1);
    //    
    //            cv::Matx33d roty(1, 0, 0,
    //                0, 1, 0,
    //                0, 0, 1);
    //    
    //        cv::Matx33d rotz(1, 0, 0,
    //                0, 1, 0,
    //                0, 0, 1);

    cv::Matx31d T3;

    PointCloudRadar::Ptr msg_radar_to_stereo(new PointCloudRadar);
//        PointCloudRadar::Ptr rect_cloud(new PointCloudRadar);
            pcl::PointCloud<pcl::PointNormal>::Ptr rect_cloud(new pcl::PointCloud<pcl::PointNormal>);
    rect_cloud->header = msg->header;
    msg_radar_to_stereo->header = msg->header;
    
        std::vector<cv::Point2f> poly;
    
    for (size_t i = 0; i < cloud_projected->points.size(); ++i) {
//        std::cout << "projected point " << cloud_projected->points[i].x - cloud_projected->points[1].x << " " << cloud_projected->points[i].y - cloud_projected->points[1].y << " " << cloud_projected->points[i].z - cloud_projected->points[1].z << std::endl;

        cv::Matx31d p2(cloud_projected->points[i].x - cloud_projected->points[0].x, cloud_projected->points[i].y - cloud_projected->points[0].y, cloud_projected->points[i].z - cloud_projected->points[0].z);
        T3 = 
                roty * 
                rotz * 
                
                p2;
        
        //      double theta = atan(  z/  (  (x*cos(theta_y)- y*sin(theta_y)) ));
//        double theta = atan(p2(2)/(  ((p2(0)*cos(theta_z))- (p2(1)*sin(theta_z))) ));
//std::cerr << "theta: " << theta<< std::endl;
//
//
//T3(0) = ( cos(theta)*( (p2(0)*cos(theta_z)) - (p2(1)*sin(theta_z)) ))+ (p2(2)*sin(theta));//x
//T3(1) = ((p2(0)*sin(theta_z))- (p2(1)*cos(theta_z)));//y
//T3(2) =  (-sin(theta)*((p2(0)*cos(theta_z))-  (p2(1)*sin(theta_z)  )))+ p2(2)*cos(theta); //z    
        std::cout << "rotated point   " << T3(0) << " "
                << T3(1) << " "
                << T3(2) << std::endl;
        msg_radar_to_stereo->points.push_back(pcl::PointXYZ(T3(0), T3(1), T3(2))); ///////////////////stereo 
        




    poly.push_back(cv::Point2f(T3(0), T3(1)));

    }


    cv::RotatedRect box = cv::minAreaRect(poly); // now it works!
    
    cv::Point2f vertices[4];
    box.points(vertices);
//    std::cout << "sizeof vertices     "<< sizeof(vertices) << std::endl;
for (int i = 0; i < 4; i++)
{
//        cv::line(image, vertices[i], vertices[(i+1)%4], Scalar(0,255,0));
    std::cout << "rect x      "<< i << " " <<vertices[i] << std::endl;
            msg_radar_to_stereo->points.push_back(pcl::PointXYZ(vertices[i].x, vertices[i].y, 0.0f)); ///////////////////stereo 

//    std::cout << "rect y      " <<box. << std::endl;
//    std::cout << "rect width  " <<box.x << std::endl;
//    std::cout << "rect height " <<box.x << std::endl;
                    cv::Matx31d p2(vertices[i].x /*- cloud_projected->points[0].x*/, vertices[i].y /*- cloud_projected->points[0].y*/, 0.0f /*- cloud_projected->points[0].z*/);
        T3 = 
                roty.inv() * 
                rotz.inv() * 
                p2;
        
        //      double theta = atan(  z/  (  (x*cos(theta_y)- y*sin(theta_y)) ));
//        double theta = atan(p2(2)/(  ((p2(0)*cos(theta_z))- (p2(1)*sin(theta_z))) ));
//std::cerr << "theta: " << theta<< std::endl;
//
//
//T3(0) = ( cos(theta)*( (p2(0)*cos(theta_z)) - (p2(1)*sin(theta_z)) ))+ (p2(2)*sin(theta));//x
//T3(1) = ((p2(0)*sin(theta_z))- (p2(1)*cos(theta_z)));//y
//T3(2) =  (-sin(theta)*((p2(0)*cos(theta_z))-  (p2(1)*sin(theta_z)  )))+ p2(2)*cos(theta); //z    
        std::cout << "rotated point   " << T3(0)+cloud_projected->points[0].x << " "
                << T3(1)+cloud_projected->points[0].y << " "
                << T3(2)+cloud_projected->points[0].z << std::endl;
        msg_radar_to_stereo->points.push_back(pcl::PointXYZ(T3(0)+cloud_projected->points[0].x, T3(1)+cloud_projected->points[0].y, T3(2)+cloud_projected->points[0].z)); ///////////////////stereo 
        pcl::PointNormal p;
       p.x = T3(0)+cloud_projected->points[0].x;
       p.y = T3(1)+cloud_projected->points[0].y;
       p.z = T3(2)+cloud_projected->points[0].z;
        rect_cloud->points.push_back(pcl::PointNormal( p)); ///////////////////stereo 

}

    
//        pcl::PointCloud<pcl::PointNormal>::Ptr cloud_filtered2(new pcl::PointCloud<pcl::PointNormal>);
        pcl::PointCloud<pcl::PointNormal>::Ptr rect_cloud_proj(new pcl::PointCloud<pcl::PointNormal>);

    // Project the model inliers
    pcl::ProjectInliers<pcl::PointNormal> proj_rec;
    proj_rec.setModelType(pcl::SACMODEL_PLANE);
    proj_rec.setInputCloud(rect_cloud);
//    proj_rec.setIndices(inliers);
    proj_rec.setModelCoefficients(coefficients);
    proj_rec.filter(*rect_cloud_proj);

//    // Create a Convex Hull representation of the projected inliers
//    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointNormal>);
//    pcl::ConvexHull<pcl::PointNormal> chull;
//    chull.setInputCloud(cloud_projected);
//    chull.reconstruct(*cloud_projected);
//    //    chull.reconstruct(*cloud_hull);
    
    


//    minmax.header.frame_id = msg->header.frame_id;

    doncloud->header = msg->header;

//    maxmin_pub.publish(minmax);
        pub_cluster.publish(cloud_filtered2);
//    pub_cluster.publish(cloud_projected);
    pub_convex_hull.publish(cloud_hull);
    //    pub_cluster.publish(cloud_filtered2);
    //    pub_cluster.publish(doncloud);
//    pub_radar_cloud.publish(msg_radar_to_stereo);
//    pub_radar_cloud.publish(rect_cloud);
    pub_radar_cloud.publish(rect_cloud_proj);
    cloud_projected->clear(); 
    rect_cloud_proj->clear();
    msg_radar_to_stereo->clear();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pub_pcl");
    ros::NodeHandle nh;
    pub_cluster = nh.advertise<PointCloudCluster> ("cluster", 1);
    pub_convex_hull = nh.advertise<PointCloudCluster> ("convex_hull", 1);
    maxmin_pub = nh.advertise<msg_box3d::Num>("cloud_max_min", 1);
    ros::Subscriber sub = nh.subscribe<PointCloud>("passthrough2", 1, callback); /////////////////////////////////////velodyne
    //    ros::Subscriber sub_radar = nh.subscribe("radar_esr_msg_array_1", 1, callbackRadar);
    //  ros::spin();
    pub_radar_cloud = nh.advertise<PointCloudRadar> ("pointsSquareCalib", 10);
    ros::spin();

}
