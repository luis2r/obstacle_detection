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

#include <msg_box3d/Num.h>
#include <msg_box3d/NumArray.h>

#include <pcl/filters/median_filter.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

#include <boost/foreach.hpp>
//#define foreach BOOST_FOREACH

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRadar;
//typedef pcl::PointCloud<pcl::PointXYZ> Cloud;
ros::Publisher pub;
ros::Publisher pub_radar_cloud;
cari_msgs_perception::ObstacleArray::ConstPtr radarDataArray_;

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
    printf("Cloud: width = %d, height = %d\n", msg->width, msg->height);
    //    BOOST_FOREACH(const pcl::PointXYZRGB& pt, msg->points)
    //    printf("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);



    PointCloudRadar::Ptr msg_radar(new PointCloudRadar);
    msg_radar->header = msg->header;
    //  msg_radar->header.frame_id = "some_tf_frame";
    //  msg_radar->height = msg->width = 1;



    pcl::PointXYZRGB radar_point; ///////////////////////////////////////////////////////////////////////////////////////////////////////////////

    for (int j = 0; j < radarDataArray_->obstacle.size(); j++) {
        //            data2_ = radarDataArray_->obstacle[j];
        if (radarDataArray_->obstacle[j].pose.position.x != 0 and radarDataArray_->obstacle[j].pose.position.y != 0) {
            printf("if");
            //we'll create a point in the base_radar(viene de radaresr(podria usarse radar esr envez de base_radar)) frame that we'd like to transform to the /bumblebee frame
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

                        cv::Point3d pt_cv5(T3(0, 0) / 1000, T3(1, 0) / 1000, T3(2, 0) / 1000);///////////////////////////////////////stereo 
//            cv::Point3d pt_cv5(radar_point2.point.x + 1.5, radar_point2.point.y /*- 1.0*/, radar_point2.point.z); //////////velodyne

            msg_radar->points.push_back(pcl::PointXYZRGB(pt_cv5.x, pt_cv5.y /*- 1.0*/, pt_cv5.z));
            float limts_x_min = -8.0;
            float limts_x_max = 8.0;
            float limts_z_min = 1.0;
            float limts_z_max = 50.0;
            if (pt_cv5.x > limts_x_min and pt_cv5.x < limts_x_max and///////////////////////////////////////stereo 
                    pt_cv5.z > limts_z_min and pt_cv5.z < limts_z_max) {////////////////////////////////////stereo 
                
//                if (pt_cv5.y > limts_x_min and pt_cv5.y < limts_x_max and //////////velodyne
//                    pt_cv5.x > limts_z_min and pt_cv5.x < limts_z_max) { //////////velodyne
                printf("if");
                // Create the filtering object: downsample the dataset using a leaf size of 1cm
                //                        pcl::IndicesPtr indices_voxels(new std::vector <int>);
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB> ());
                pcl::VoxelGrid<pcl::PointXYZRGB> vg;
                vg.setInputCloud(msg);
                vg.setLeafSize(0.02f, 0.02f, 0.02f);
                                vg.setFilterFieldName("y");//////////////////////////////////stereo
                                vg.setFilterLimits(-0.5, 1.5);///////////////stereo
//                vg.setFilterFieldName("z"); ////////////////velodyne
//                vg.setFilterLimits(-2.5, 2.5); /////////////velodyne
                vg.filter(*cloud_filtered);



                pcl::IndicesPtr indices_x(new std::vector <int>);
                pcl::IndicesPtr indices_xy(new std::vector <int>);
                pcl::IndicesPtr indices_xyz(new std::vector <int>);
                //    pcl::IndicesPtr indices_voxels(new std::vector <int>);
                pcl::PassThrough<pcl::PointXYZRGB> pass;
                //                                pass.setInputCloud(msg);
                pass.setInputCloud(cloud_filtered);
                                pass.setFilterFieldName("x");//////////////////////////////////stereo
                                pass.setFilterLimits(pt_cv5.x - 3, pt_cv5.x + 3);//////////////stereo
//                pass.setFilterFieldName("y"); /////////////////////////////////////////////////////////velodyne
//                pass.setFilterLimits(pt_cv5.y - 3, pt_cv5.y + 3); ////////////////////////////////////velodyne
                pass.filter(*indices_x);

                //    pass.setIndices(indices_x);
                //    pass.setFilterFieldName("y");
                //    pass.setFilterLimits(-2.0, 20.0);
                //    //pass.setNegative (true);
                //    pass.filter(*indices_xy);

                //    pass.setIndices(indices_xy);
                pass.setIndices(indices_x);
                                pass.setFilterFieldName("z");//////////////////////////////////stereo
                                pass.setFilterLimits(pt_cv5.z - 3, pt_cv5.z + 3);//////////////stereo
//                pass.setFilterFieldName("x"); /////////////////////////////////////////////////////////velodyne
//                pass.setFilterLimits(pt_cv5.x - 3, pt_cv5.x + 3); /////////////////////////////////////velodyne
                //pass.setNegative (true);
                pass.filter(*indices_xyz);




                //    PointCloud::Ptr msg_seg(new PointCloud);
                //    msg_seg = *msg;
                pcl::MinCutSegmentation<pcl::PointXYZRGB> seg;

                seg.setInputCloud(cloud_filtered);
                //                seg.setInputCloud(msg);

                //                    seg.setIndices(indices_voxels);
                seg.setIndices(indices_xyz);

                pcl::PointCloud<pcl::PointXYZRGB>::Ptr foreground_points(new pcl::PointCloud<pcl::PointXYZRGB> ());
                pcl::PointXYZRGB point;

                point.x = pt_cv5.x;
                point.y = pt_cv5.y /*- 0.1*/; //calibracao scene
                point.z = pt_cv5.z;
                foreground_points->points.push_back(point);
                seg.setForegroundPoints(foreground_points);

                seg.setSigma(0.28);
                seg.setRadius(0.50);
                //    seg.setRadius(0.5);
                seg.setNumberOfNeighbours(14);
                seg.setSourceWeight(0.8);

                std::vector <pcl::PointIndices> clusters;
                seg.extract(clusters);

                std::cout << "Maximum flow is " << seg.getMaxFlow() << std::endl;
                std::cout << "size points" << seg.getForegroundPoints().max_size() << std::endl;
                pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = seg.getColoredCloud();
                colored_cloud->header = msg->header;

                pub.publish(colored_cloud);
            }

        }

    }
    pub_radar_cloud.publish(msg_radar);

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pub_pcl");
    ros::NodeHandle nh;
    pub = nh.advertise<PointCloudRGB> ("points2", 1);
    pub_radar_cloud = nh.advertise<PointCloudRadar> ("pointsRadar", 1);
        ros::Subscriber sub = nh.subscribe<PointCloud>("stereo/narrow/points2", 1, callback);//////////////stereo
//    ros::Subscriber sub = nh.subscribe<PointCloud>("velodyne_points", 1, callback);/////////////////////////////////////velodyne
    ros::Subscriber sub_radar = nh.subscribe("radar_esr_msg_array_1", 1, callbackRadar);
    //  ros::spin();
    ros::spin();

}




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
#include <cari_msgs_perception/Obstacle.h>//  <radaresr15/radaresr15Data.h>

#include <pcl/filters/median_filter.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

#include <boost/foreach.hpp>
//#define foreach BOOST_FOREACH

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudRadar;
//typedef pcl::PointCloud<pcl::PointXYZ> Cloud;
ros::Publisher pub;
ros::Publisher pub_radar_cloud;
cari_msgs_perception::ObstacleArray::ConstPtr radarDataArray_;

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



    PointCloudRadar::Ptr msg_radar(new PointCloudRadar);
    msg_radar->header = msg->header;
    //  msg_radar->header.frame_id = "some_tf_frame";
    //  msg_radar->height = msg->width = 1;

    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    //                        pcl::IndicesPtr indices_voxels(new std::vector <int>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB> ());
    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    vg.setInputCloud(msg);
    vg.setLeafSize(0.05f, 0.05f, 0.05f);
    vg.setFilterFieldName("y");
    vg.setFilterLimits(-0.5, 1.5);
    vg.filter(*cloud_filtered);

    pcl::PointXYZ radar_point;///////////////////////////////////////////////////////////////////////////////////////////////////////////////

    for (int j = 0; j < radarDataArray_->obstacle.size(); j++) {
        //            data2_ = radarDataArray_->obstacle[j];
        if (radarDataArray_->obstacle[j].pose.position.x != 0 and radarDataArray_->obstacle[j].pose.position.y != 0) {

            //we'll create a point in the base_radar(viene de radaresr(podria usarse radar esr envez de base_radar)) frame that we'd like to transform to the /bumblebee frame
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

            cv::Point3d pt_cv5(T3(0, 0) / 1000, T3(1, 0) / 1000, T3(2, 0) / 1000);
            //            ROS_INFO(" Punto: %f %f %f\n", pt_cv5.x, pt_cv5.y, pt_cv5.z);            
            //            msg_radar->points.push_back(pcl::PointXYZ(pt_cv5.x, pt_cv5.y, pt_cv5.z));
            msg_radar->points.push_back(pcl::PointXYZ(pt_cv5.x, pt_cv5.y - 1.0, pt_cv5.z));

            float limts_x_min = -5.0;
            float limts_x_max = 5.0;
            float limts_z_min = 1.0;
            float limts_z_max = 30.0;
            if (pt_cv5.x > limts_x_min and pt_cv5.x < limts_x_max and
                    pt_cv5.z > limts_z_min and pt_cv5.z < limts_z_max) {



                                pcl::IndicesPtr indices_x(new std::vector <int>);
                                pcl::IndicesPtr indices_xy(new std::vector <int>);
                                pcl::IndicesPtr indices_xyz(new std::vector <int>);
                                //    pcl::IndicesPtr indices_voxels(new std::vector <int>);
                                pcl::PassThrough<pcl::PointXYZRGB> pass;
//                                pass.setInputCloud(msg);
                                pass.setInputCloud(cloud_filtered);
                                pass.setFilterFieldName("x");
                                pass.setFilterLimits(limts_x_min, limts_x_max);
                                pass.filter(*indices_x);
                
                                //    pass.setIndices(indices_x);
                                //    pass.setFilterFieldName("y");
                                //    pass.setFilterLimits(-2.0, 20.0);
                                //    //pass.setNegative (true);
                                //    pass.filter(*indices_xy);
                
                                //    pass.setIndices(indices_xy);
                                pass.setIndices(indices_x);
                                pass.setFilterFieldName("z");
                                pass.setFilterLimits(limts_z_min, limts_z_max);
                                //pass.setNegative (true);
                                pass.filter(*indices_xyz);




                //    PointCloud::Ptr msg_seg(new PointCloud);
                //    msg_seg = *msg;
                pcl::MinCutSegmentation<pcl::PointXYZRGB> seg;

                seg.setInputCloud(cloud_filtered);
                //                seg.setInputCloud(msg);

                //                    seg.setIndices(indices_voxels);
                                seg.setIndices(indices_xyz);

                pcl::PointCloud<pcl::PointXYZRGB>::Ptr foreground_points(new pcl::PointCloud<pcl::PointXYZRGB> ());
                pcl::PointXYZRGB point;

                point.x = pt_cv5.x;
                point.y = pt_cv5.y - 0.1; //calibracao scene
                point.z = pt_cv5.z;
                foreground_points->points.push_back(point);
                seg.setForegroundPoints(foreground_points);

                seg.setSigma(1.8);
                seg.setRadius(1.0);
                //    seg.setRadius(0.5);
                seg.setNumberOfNeighbours(14);
                seg.setSourceWeight(0.8);

                std::vector <pcl::PointIndices> clusters;
                seg.extract(clusters);

                std::cout << "Maximum flow is " << seg.getMaxFlow() << std::endl;
                std::cout << "size points" << seg.getForegroundPoints().max_size() << std::endl;
                pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = seg.getColoredCloud();
                colored_cloud->header = msg->header;

                pub.publish(colored_cloud);
            }

        }

    }
    pub_radar_cloud.publish(msg_radar);

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pub_pcl");
    ros::NodeHandle nh;
    pub = nh.advertise<PointCloud> ("points2", 1);
    pub_radar_cloud = nh.advertise<PointCloudRadar> ("pointsRadar", 1);
    ros::Subscriber sub = nh.subscribe<PointCloud>("stereo/narrow/points2", 1, callback);
//    ros::Subscriber sub = nh.subscribe<PointCloud>("voxel_grid/output", 1, callback);
    ros::Subscriber sub_radar = nh.subscribe("radar_esr_msg_array_1", 1, callbackRadar);
    //  ros::spin();
    ros::spin();

}
