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
#include <pcl/filters/median_filter.h>

#include <pcl/io/pcd_io.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/extract_clusters.h>

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

void callback(const PointCloud::ConstPtr& msg) {

    //    pcl::PointXYZRGB radar_point; ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointNormal>);
    msg_box3d::NumArray array_minmax;
    PointCloudRadar::Ptr msg_radar_to_stereo(new PointCloudRadar);

    for (int j = 0; j < radarDataArray_->obstacle.size(); j++) {
        if (radarDataArray_->obstacle[j].pose.position.x != 0 and radarDataArray_->obstacle[j].pose.position.y != 0) {
            geometry_msgs::PointStamped radar_point2;
            radar_point2.header.frame_id = "radaresr1";
            radar_point2.header.stamp = ros::Time();
            radar_point2.point.x = radarDataArray_->obstacle[j].pose.position.x;
            radar_point2.point.y = radarDataArray_->obstacle[j].pose.position.y;
            radar_point2.point.z = radarDataArray_->obstacle[j].pose.position.z;

            cv::Matx41d p2((radar_point2.point.x * 1000), (radar_point2.point.y * 1000), (radar_point2.point.z * 1000), 1);

            cv::Matx34d mat_ext(-0.00753608, -0.99995464, -0.00582364, 185.21485096,
                    -0.02996473, 0.00604701, -0.99953264, 976.07698905,
                    0.99952257, -0.00735806, -0.03000895, 2364.9362015);

            cv::Matx31d T3;

            T3 = mat_ext*p2;

//            PointCloudRadar::Ptr msg_radar(new PointCloudRadar);
            msg_radar_to_stereo->header = msg->header;
            cv::Point3d pt_cv5(T3(0, 0) / 1000, T3(1, 0) / 1000, T3(2, 0) / 1000); ///////////////////////////////////////stereo 
            //            cv::Point3d pt_cv5(radar_point2.point.x + 1.3, radar_point2.point.y /*- 1.0*/, radar_point2.point.z); //////////velodyne
            //            msg_radar->points.push_back(pcl::PointXYZ(pt_cv5.x, pt_cv5.y, pt_cv5.z));
            //            msg_radar->points.push_back(pcl::PointXYZ(pt_cv5.x + 0.5, pt_cv5.y, pt_cv5.z)); //////////velodyne
            msg_radar_to_stereo->points.push_back(pcl::PointXYZ(pt_cv5.x, pt_cv5.y - 0.5, pt_cv5.z)); ///////////////////stereo 
            float limts_x_min = -8.0;
            float limts_x_max = 8.0;
            float limts_z_min = 0.0;
            float limts_z_max = 40.0;
            if (pt_cv5.x > limts_x_min and pt_cv5.x < limts_x_max and///////////////////////////////////////stereo 
                    pt_cv5.z > limts_z_min and pt_cv5.z < limts_z_max) {////////////////////////////////////stereo 

                //            if (pt_cv5.y > limts_x_min and pt_cv5.y < limts_x_max and //////////velodyne
                //                    pt_cv5.x > limts_z_min and pt_cv5.x < limts_z_max) { //////////velodyne

                // Create the median filtering object
                //    pcl::PassThrough<pcl::PointXYZRGBA> pass;
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ> ());
                pcl::VoxelGrid<pcl::PointXYZ> vg;
                vg.setInputCloud(msg);
                //                vg.setInputCloud(cloud_filtered_media);
                vg.setLeafSize(0.1f, 0.1f, 0.1f);
                                vg.setFilterFieldName("z"); //////////////////////////////////stereo
                                vg.setFilterLimits(pt_cv5.z - 3, pt_cv5.z + 3); ///////////////stereo
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
                std::cout << "Calculating normals for scale..." << scale2 << std::endl;
                pcl::PointCloud<pcl::PointNormal>::Ptr normals_large_scale(new pcl::PointCloud<pcl::PointNormal>);

                ne.setRadiusSearch(scale2);
                ne.compute(*normals_large_scale);

                // Create output cloud for DoN results
                pcl::PointCloud<pcl::PointNormal>::Ptr doncloud(new pcl::PointCloud<pcl::PointNormal>);
                pcl::copyPointCloud<pcl::PointXYZ, pcl::PointNormal>(*cloud_filtered, *doncloud);

                std::cout << "Calculating DoN... " << std::endl;
                // Create DoN operator
                pcl::DifferenceOfNormalsEstimation<pcl::PointXYZ, pcl::PointNormal, pcl::PointNormal> don;
                don.setInputCloud(cloud_filtered);
                don.setNormalScaleLarge(normals_large_scale);
                don.setNormalScaleSmall(normals_small_scale);

                if (!don.initCompute()) {
                    //        std::cerr << "Error: Could not intialize DoN feature operator" << std::endl;
                    exit(EXIT_FAILURE);
                }
                std::cout << "ccccccc " << std::endl;
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




                pcl::PointCloud<pcl::PointNormal>::Ptr cloud_filtered_SOR(new pcl::PointCloud<pcl::PointNormal>);
                // Create the filtering object
                pcl::StatisticalOutlierRemoval<pcl::PointNormal> sor;
                sor.setInputCloud(doncloud);
                sor.setMeanK(50);
                sor.setStddevMulThresh(0.7);
                sor.filter(*cloud_filtered_SOR);





                pcl::IndicesPtr indices_x(new std::vector <int>);
                pcl::IndicesPtr indices_xy(new std::vector <int>);
                pcl::IndicesPtr indices_xyz(new std::vector <int>);
                pcl::PassThrough<pcl::PointNormal> pass;
                //                                pass.setInputCloud(msg);
                pass.setInputCloud(cloud_filtered_SOR);
                ////////////////////                pass.setInputCloud(doncloud);
                //                pass.setFilterFieldName("z"); //////////////////////////////////stereo
                //                pass.setFilterLimits(pt_cv5.z - 3, pt_cv5.z + 3); //////////////stereo
                pass.setFilterFieldName("x"); /////////////////////////////////////////////////////////velodyne
                pass.setFilterLimits(pt_cv5.x - 3, pt_cv5.x + 3); /////////////////////////////////////velodyne
                pass.filter(*indices_xyz);








                pcl::MinCutSegmentation<pcl::PointNormal> seg;

                //                seg.setInputCloud(cloud_filtered);
                seg.setInputCloud(doncloud);

                seg.setIndices(indices_xyz);

                pcl::PointCloud<pcl::PointNormal>::Ptr foreground_points(new pcl::PointCloud<pcl::PointNormal> ());
                pcl::PointNormal point;

                point.x = pt_cv5.x;
                point.y = pt_cv5.y - 0.5; //stereo
                //                point.y = pt_cv5.y; //velodyne
                point.z = pt_cv5.z + 0.2;
                foreground_points->points.push_back(point);
                seg.setForegroundPoints(foreground_points);
                seg.setSigma(0.1); //velodyne
                //                seg.setSigma(0.2);
                seg.setRadius(1.5);
                //    seg.setRadius(0.5);
                seg.setNumberOfNeighbours(14);
                seg.setSourceWeight(0.8);

                std::vector <pcl::PointIndices> clusters;
                seg.extract(clusters);
                if (clusters.size() > 0) {
                    //                pcl::PointCloud <pcl::PointNormal>::Ptr colored_cloud = seg.getColoredCloud();
                    //                colored_cloud->header = msg->header;

                    uint8_t r = rand() % 255;
                    uint8_t g = rand() % 255;
                    uint8_t b = rand() % 255;
                    int32_t rgb = ((int) r) << 16 | ((int) g) << 8 | ((int) b);
                    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_cluster_aux(new pcl::PointCloud<pcl::PointNormal>);
                    std::cout << "rrrrrr" << std::endl;
                    std::cout << "cl " << clusters.size() << std::endl;

                    for (std::vector<int>::const_iterator pit = clusters.at(1).indices.begin(); pit != clusters.at(1).indices.end(); ++pit) {
                        //                    doncloud->points[*pit].rgba = rgb; //color_cloud % 255;
                        //                    cloud_filtered->points[*pit].rgba = rgb; //color_cloud % 255;
                        //                    cloud_cluster->points.push_back(cloud_filtered->points[*pit]); //*                                        
                        std::cout << "ffffffffffffffffff " << std::endl;
                        cloud_cluster->points.push_back(doncloud->points[*pit]); //*  
                        cloud_cluster_aux->points.push_back(doncloud->points[*pit]); //* 
                    }

                    // Save filtered output
                    //    std::cout << "Filtered Pointcloud: " << doncloud->points.size() << " data points." << std::endl;
                    pcl::PointNormal minPt, maxPt;
                    //                    pcl::getMinMax3D(*cloud_cluster_aux, minPt, maxPt);
                    pcl::getMinMax3D(*cloud_cluster_aux, minPt, maxPt);
                    std::cout << "Max x: " << maxPt.x << std::endl;
                    //                std::cout << "Max y: " << maxPt.y << std::endl;
                    //                std::cout << "Max z: " << maxPt.z << std::endl;
                    //                std::cout << "Min x: " << minPt.x << std::endl;
                    //                std::cout << "Min y: " << minPt.y << std::endl;
                    //                std::cout << "Min z: " << minPt.z << std::endl;

                    msg_box3d::Num minmax;

                    minmax.max_x = maxPt.x;
                    minmax.max_y = maxPt.y;
                    minmax.max_z = maxPt.z;
                    minmax.min_x = minPt.x;
                    minmax.min_y = minPt.y;
                    minmax.min_z = minPt.z;

                    minmax.header.frame_id = msg->header.frame_id;
                    //  minmax.header.seq =    msg->header.seq;
                    //  minmax.header.stamp =    msg->header.stamp;


                    //    cloud_cluster->width = cloud_cluster->points.size();
                    //    cloud_cluster->height = 1;
                    //    cloud_cluster->is_dense = true;
                    //                doncloud->header = msg->header;

                    //                maxmin_pub.publish(minmax);
                    array_minmax.Nums.push_back(minmax);
                    std::cout << "xxxs " << maxPt.x << std::endl;
                }
            }
        }
    }
    cloud_cluster->width = cloud_cluster->points.size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    cloud_cluster->header = msg->header;

    pub_radar_cloud.publish(msg_radar_to_stereo);

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
