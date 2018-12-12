/*
 * Copyright (c) 2010, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// %Tag(FULLTEXT)%
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <cari_msgs_perception/ObstacleArray.h>
#include <cari_msgs_perception/Obstacle.h>
#include <msg_box3d/Num.h>
#include <msg_box3d/NumArray.h>
#include <cmath>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>

#include <iostream>
#include <math.h>       /* sqrt */

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

ros::Publisher pub_marker;

void callbackRadar(const PointCloud::ConstPtr& msg) {
    visualization_msgs::MarkerArray markerArray; //marker_array_msg,
    //    msg_box3d::NumArray::ConstPtr nums = msg;
    //    msg_box3d::Num::ConstPtr num = msg;

    //        markerArray.markers.resize(64); //final->width * final->height);








    //        for (int j = 0; j < nums->Nums.size(); j++) {
    //        if (msg->.x != 0 and msg->obstacle[j].pose.position.y != 0) {


    // %Tag(MARKER_INIT)%
    visualization_msgs::Marker /*points, line_list, */line_strip;
    //        line_strip.header = msg->header;
    /*points.header.frame_id = line_list.header.frame_id =*/line_strip.header.frame_id = msg->header.frame_id;
    /*points.header.stamp = line_list.header.stamp =*/line_strip.header.stamp = ros::Time::now();
    /*points.ns =  line_list.ns = */line_strip.ns = "bounding_box";
    /*points.action = line_list.action =*/line_strip.action = visualization_msgs::Marker::ADD;
    /*points.pose.orientation.w = line_list.pose.orientation.w =  */line_strip.pose.orientation.w = 1.0;
    // %EndTag(MARKER_INIT)%

    // %Tag(ID)%
    //points.id = 0;
    //        line_strip.id = *pit;

    line_strip.id = 0;

    //line_list.id = 2;
    // %EndTag(ID)%

    // %Tag(TYPE)%
    //points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    //line_list.type = visualization_msgs::Marker::LINE_LIST;
    // %EndTag(TYPE)%

    // %Tag(SCALE)%
    // POINTS markers use x and y scale for width/height respectively
    //points.scale.x = 0.2;
    //points.scale.y = 0.2;

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.041;
    //line_list.scale.x = 0.1;
    // %EndTag(SCALE)%

    // %Tag(COLOR)%
    // Points are green
    //points.color.g = 1.0f;
    //points.color.a = 1.0;

    // Line strip is blue
    line_strip.color.g = 1.0;
    line_strip.color.a = 1.0;

    // Line list is red
    //line_list.color.r = 1.0;
    //line_list.color.a = 1.0;
    // %EndTag(COLOR)%

    // %Tag(HELIX)%
    // Create the vertices for the points and lines
    //    for (uint32_t i = 0; i < 100; ++i)
    //    {
    //                double min_x = nums->Nums[j].min_x;
    //                double max_x = nums->Nums[j].max_x;
    //                double min_y = nums->Nums[j].min_y;
    //                double max_y = nums->Nums[j].max_y;
    //                double min_z = nums->Nums[j].min_z;
    //                double max_z = nums->Nums[j].max_z;

    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D(*msg, minPt, maxPt);
    std::cout << "Max x: " << maxPt.x << std::endl;
    std::cout << "Max y: " << maxPt.y << std::endl;
    std::cout << "Max z: " << maxPt.z << std::endl;
    std::cout << "Min x: " << minPt.x << std::endl;
    std::cout << "Min y: " << minPt.y << std::endl;
    std::cout << "Min z: " << minPt.z << std::endl;
    double min_x = minPt.x;
    double max_x = maxPt.x;
    double min_y = minPt.y;
    double max_y = maxPt.y;
    double min_z = minPt.z;
    double max_z = maxPt.z;


    //                        double min_x = 0;
    //            double max_x = 1;
    //            double min_y = 0;
    //            double max_y = 1;
    //            double min_z = 0;
    //            double max_z = 1;

    std::cout << "mix: " << min_x << std::endl;
    std::cout << "max: " << max_x << std::endl;
    std::cout << "miy: " << min_x << std::endl;
    std::cout << "may: " << max_y << std::endl;
    std::cout << "miz: " << min_x << std::endl;
    std::cout << "maz: " << max_y << std::endl;


    int i = 0;
    geometry_msgs::Point pfin;

    if (i == 0)

        for (pcl::PointCloud<pcl::PointXYZ>::const_iterator pit = msg->begin(); pit != msg->end(); ++pit) {
            if (i == 0) {
                pfin.x = pit->x;
                pfin.y = pit->y;
                pfin.z = pit->z;
            }

            //                    doncloud->points[*pit].rgba = rgb; //color_cloud % 255;
            //                    cloud_filtered->points[*pit].rgba = rgb; //color_cloud % 255;
            //                    cloud_cluster->points.push_back(cloud_filtered->points[*pit]); //*

            std::cout << pit->x << ", " << pit->y << ", " << pit->z << std::endl;
            //        std::cout << "max x: " << maxPt.x << std::endl;

            //        cloud_cluster->points.push_back(doncloud->points[*pit]); //*  
            //        cloud_cluster_aux->points.push_back(doncloud->points[*pit]); //* 
            //    }            

            geometry_msgs::Point p;

            /***************************************bounding box velodine***********************************************/
            //        p.x = min_x;
            //        p.y = min_y;
            //        p.z = min_z;


            p.x = pit->x;
            p.y = pit->y;
            p.z = pit->z;

            line_strip.points.push_back(p);

            //        p.x = min_x;
            //        p.y = min_y;
            //        p.z = max_z;
            //
            //        line_strip.points.push_back(p);
            //
            //        p.x = min_x;
            //        p.y = max_y;
            //        p.z = max_z;
            //
            //        line_strip.points.push_back(p);
            //
            //        p.x = min_x;
            //        p.y = max_y;
            //        p.z = min_z;
            //
            //        line_strip.points.push_back(p);
            //
            //        p.x = min_x;
            //        p.y = min_y;
            //        p.z = min_z;
            //
            //        line_strip.points.push_back(p);
            //
            //        p.x = max_x;
            //        p.y = min_y;
            //        p.z = min_z;
            //
            //        line_strip.points.push_back(p);
            //
            //        p.x = max_x;
            //        p.y = min_y;
            //        p.z = max_z;
            //
            //        line_strip.points.push_back(p);
            //
            //        p.x = min_x;
            //        p.y = min_y;
            //        p.z = max_z;
            //
            //        line_strip.points.push_back(p);
            //
            //        p.x = min_x;
            //        p.y = max_y;
            //        p.z = max_z;
            //
            //        line_strip.points.push_back(p);
            //
            //        p.x = max_x;
            //        p.y = max_y;
            //        p.z = max_z;
            //
            //        line_strip.points.push_back(p);
            //
            //        p.x = max_x;
            //        p.y = max_y;
            //        p.z = min_z;
            //
            //        line_strip.points.push_back(p);
            //
            //        p.x = min_x;
            //        p.y = max_y;
            //        p.z = min_z;
            //
            //        line_strip.points.push_back(p);
            //
            //        p.x = max_x;
            //        p.y = max_y;
            //        p.z = min_z;
            //
            //        line_strip.points.push_back(p);
            //
            //        p.x = max_x;
            //        p.y = min_y;
            //        p.z = min_z;
            //
            //        line_strip.points.push_back(p);
            //
            //        p.x = max_x;
            //        p.y = min_y;
            //        p.z = max_z;
            //
            //        line_strip.points.push_back(p);
            //
            //
            //        p.x = max_x;
            //        p.y = max_y;
            //        p.z = max_z;
            //
            //        line_strip.points.push_back(p);
            //
            //        p.x = max_x;
            //        p.y = max_y;
            //        p.z = min_z;
            //
            //        line_strip.points.push_back(p);
            /***************************************bounding box velodine***********************************************/
            //            p.x = max_x;
            //            p.y = min_y;
            //            p.z = min_z;
            //
            //            line_strip.points.push_back(p);
            //
            //            p.x = min_x;
            //            p.y = min_y;
            //            p.z = min_z;
            //
            //            line_strip.points.push_back(p);
            //
            //            p.x = min_x;
            //            p.y = min_y;
            //            p.z = max_z;
            //
            //            line_strip.points.push_back(p);
            //
            //            p.x = max_x;
            //            p.y = min_y;
            //            p.z = max_z;
            //
            //            line_strip.points.push_back(p);
            //
            //            p.x = max_x;
            //            p.y = min_y;
            //            p.z = min_z;
            //
            //            line_strip.points.push_back(p);
            //
            //            p.x = max_x;
            //            p.y = max_y;
            //            p.z = min_z;
            //
            //            line_strip.points.push_back(p);
            //
            //            p.x = min_x;
            //            p.y = max_y;
            //            p.z = min_z;
            //
            //            line_strip.points.push_back(p);
            //
            //            p.x = min_x;
            //            p.y = max_y;
            //            p.z = max_z;
            //
            //            line_strip.points.push_back(p);
            //
            //            p.x = max_x;
            //            p.y = max_y;
            //            p.z = max_z;
            //
            //            line_strip.points.push_back(p);
            //
            //            p.x = max_x;
            //            p.y = max_y;
            //            p.z = min_z;
            //
            //            line_strip.points.push_back(p);
            //
            //
            //            p.x = min_x;
            //            p.y = min_y;
            //            p.z = min_z;
            //
            //            line_strip.points.push_back(p);
            //
            //            p.x = min_x;
            //            p.y = max_y;
            //            p.z = max_z;
            //
            //            line_strip.points.push_back(p);
            //
            //            p.x = min_x;
            //            p.y = min_y;
            //            p.z = max_z;
            //
            //            line_strip.points.push_back(p);
            //
            //            p.x = max_x;
            //            p.y = max_y;
            //            p.z = max_z;
            //
            //            line_strip.points.push_back(p);
            //
            //            p.x = max_x;
            //            p.y = min_y;
            //            p.z = max_z;
            //            line_strip.points.push_back(p);


            //      p.x = 0;
            //      p.y = 1;
            //      p.z = 0;
            //      //points.points.push_back(p);
            //      line_strip.points.push_back(p);
            //
            //      p.x = 0;
            //      p.y = 0;
            //      p.z = 0;
            //      //points.points.push_back(p);
            //      line_strip.points.push_back(p);





            // The line list needs two points for each line
            //line_list.points.push_back(p);
            //p.z += 1.0;
            //line_list.points.push_back(p);
            //    }
            // %EndTag(HELIX)%

            //marker_pub.publish(points);

            //marker_pub.publish(line_list);

            //    r.sleep();

            //        }
            i++;
        }
    line_strip.points.push_back(pfin);


    line_strip.lifetime = ros::Duration(3);
    markerArray.markers.push_back(line_strip);
    //                markerArray.markers.push_back(line_strip);




    //}
    std::cout << "publicar " << std::endl;
    std::cout << "size " << markerArray.markers.size() << std::endl;
    double ladoA = sqrt(pow((line_strip.points.at(0).x-line_strip.points.at(1).x),2) + pow((line_strip.points.at(0).y-line_strip.points.at(1).y),2) + pow((line_strip.points.at(0).z-line_strip.points.at(1).z),2) );
    double ladoB = sqrt(pow((line_strip.points.at(1).x-line_strip.points.at(2).x),2) + pow((line_strip.points.at(1).y-line_strip.points.at(2).y),2) + pow((line_strip.points.at(1).z-line_strip.points.at(2).z),2) );

    if ((ladoA<1.20 and ladoA>0.5)and(ladoB<1.20 and ladoB>0.5)){
        
            pub_marker.publish(markerArray);
    }

    markerArray.markers.clear();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "square_calib_vel");
    //    ros::NodeHandle n;
    ros::NodeHandle nh;
    //    marker_pub = n.advertise<visualization_msgs::Marker>("bounding_box", 10);
    //    ros::Publisher pub_marker = n.advertise<visualization_msgs::MarkerArray>("bounding_box", 10);
    //        ros::Subscriber sub_radar = nh.subscribe("cloud_max_min", 10, callbackRadar);
    ros::Subscriber sub_radar = nh.subscribe("pointsSquareCalib", 10, callbackRadar);


    pub_marker = nh.advertise<visualization_msgs::MarkerArray>("square_calib_vel", 1);


    //  ros::spin();
    ros::spin();
    return 0;
    //  ros::Rate r(30);
}
// %EndTag(FULLTEXT)%

