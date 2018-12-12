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

#include <cmath>

ros::Publisher pub_marker;

void callbackRadar(const cari_msgs_perception::ObstacleArray::ConstPtr& msg_radar) {

    visualization_msgs::MarkerArray markerArray; //marker_array_msg,
    cari_msgs_perception::ObstacleArray::ConstPtr radarDataArray_ = msg_radar;
    
        markerArray.markers.resize(64); //final->width * final->height);
    for (int j = 0; j < radarDataArray_->obstacle.size(); j++) {
        //        if (radarDataArray_->obstacle[j].pose.position.x != 0 and radarDataArray_->obstacle[j].pose.position.y != 0) {


        // %Tag(MARKER_INIT)%
        visualization_msgs::Marker /*points, line_list, */line_strip;
        line_strip.header = radarDataArray_->obstacle[j].header;
        //            /*points.header.frame_id = line_list.header.frame_id =*/line_strip.header.frame_id = "/velodyne";
        //            /*points.header.stamp = line_list.header.stamp =*/line_strip.header.stamp = ros::Time::now();
        //            /*points.ns =  line_list.ns = */line_strip.ns = "bounding_box";
        /*points.action = line_list.action =*/line_strip.action = visualization_msgs::Marker::ADD;
        /*points.pose.orientation.w = line_list.pose.orientation.w =  */line_strip.pose.orientation.w = 1.0;
        // %EndTag(MARKER_INIT)%

        // %Tag(ID)%
        //points.id = 0;
        line_strip.id = 1;
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
        line_strip.scale.x = 0.1;
        //line_list.scale.x = 0.1;
        // %EndTag(SCALE)%

        // %Tag(COLOR)%
        // Points are green
        //points.color.g = 1.0f;
        //points.color.a = 1.0;

        // Line strip is blue
        line_strip.color.b = 1.0;
        line_strip.color.a = 1.0;

        // Line list is red
        //line_list.color.r = 1.0;
        //line_list.color.a = 1.0;
        // %EndTag(COLOR)%

        // %Tag(HELIX)%
        // Create the vertices for the points and lines
        //    for (uint32_t i = 0; i < 100; ++i)
        //    {
        double min_x = radarDataArray_->obstacle[j].pose.position.x;
        double max_x = radarDataArray_->obstacle[j].pose.position.x + 1;
        double min_y = radarDataArray_->obstacle[j].pose.position.y;
        double max_y = radarDataArray_->obstacle[j].pose.position.y + 1;
        double min_z = radarDataArray_->obstacle[j].pose.position.z;
        double max_z = radarDataArray_->obstacle[j].pose.position.z + 1;

        //                        double min_x = 0;
        //            double max_x = 1;
        //            double min_y = 0;
        //            double max_y = 1;
        //            double min_z = 0;
        //            double max_z = 1;

        //            std::cout << "mix: " << min_x << std::endl;
        //            std::cout << "max: " << max_x << std::endl;
        //            std::cout << "miy: " << min_x << std::endl;
        //            std::cout << "may: " << max_y << std::endl;
        //            std::cout << "miz: " << min_x << std::endl;
        //            std::cout << "maz: " << max_y << std::endl;

        geometry_msgs::Point p;

        p.x = (int32_t) max_x;
        p.y = (int32_t) min_y;
        p.z = (int32_t) min_z;
        //points.points.push_back(p);
        line_strip.points.push_back(p);

        p.x = (int32_t) min_x;
        p.y = (int32_t) min_y;
        p.z = (int32_t) min_z;
        //points.points.push_back(p);
        line_strip.points.push_back(p);

        p.x = (int32_t) min_x;
        p.y = (int32_t) min_y;
        p.z = (int32_t) max_z;
        //points.points.push_back(p);
        line_strip.points.push_back(p);

        p.x = (int32_t) max_x;
        p.y = (int32_t) min_y;
        p.z = (int32_t) max_z;
        //points.points.push_back(p);
        line_strip.points.push_back(p);

        p.x = (int32_t) max_x;
        p.y = (int32_t) min_y;
        p.z = (int32_t) min_z;
        //points.points.push_back(p);
        line_strip.points.push_back(p);

        p.x = (int32_t) max_x;
        p.y = (int32_t) max_y;
        p.z = (int32_t) min_z;
        //points.points.push_back(p);
        line_strip.points.push_back(p);

        p.x = (int32_t) min_x;
        p.y = (int32_t) max_y;
        p.z = (int32_t) min_z;
        //points.points.push_back(p);
        line_strip.points.push_back(p);

        p.x = (int32_t) min_x;
        p.y = (int32_t) max_y;
        p.z = (int32_t) max_z;
        //points.points.push_back(p);
        line_strip.points.push_back(p);

        p.x = (int32_t) max_x;
        p.y = (int32_t) max_y;
        p.z = (int32_t) max_z;
        //points.points.push_back(p);
        line_strip.points.push_back(p);

        p.x = (int32_t) max_x;
        p.y = (int32_t) max_y;
        p.z = (int32_t) min_z;
        //points.points.push_back(p);
        line_strip.points.push_back(p);


        p.x = (int32_t) min_x;
        p.y = (int32_t) min_y;
        p.z = (int32_t) min_z;
        //points.points.push_back(p);
        line_strip.points.push_back(p);

        p.x = (int32_t) min_x;
        p.y = (int32_t) max_y;
        p.z = (int32_t) max_z;
        //points.points.push_back(p);
        line_strip.points.push_back(p);

        p.x = (int32_t) min_x;
        p.y = (int32_t) min_y;
        p.z = (int32_t) max_z;
        //points.points.push_back(p);
        line_strip.points.push_back(p);

        p.x = (int32_t) max_x;
        p.y = (int32_t) max_y;
        p.z = (int32_t) max_z;
        //points.points.push_back(p);
        line_strip.points.push_back(p);

        p.x = (int32_t) max_x;
        p.y = (int32_t) min_y;
        p.z = (int32_t) max_z;
        //points.points.push_back(p);
        line_strip.points.push_back(p);


        //      p.x = (int32_t)0;
        //      p.y = (int32_t)1;
        //      p.z = (int32_t)0;
        //      //points.points.push_back(p);
        //      line_strip.points.push_back(p);
        //
        //      p.x = (int32_t)0;
        //      p.y = (int32_t)0;
        //      p.z = (int32_t)0;
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

        markerArray.markers.at(j) = line_strip;
//                markerArray.markers.push_back(line_strip);
        //        }
    }
    std::cout << "publicar " << std::endl;
    std::cout << "size " << markerArray.markers.size() << std::endl;
    pub_marker.publish(markerArray);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "bounding_box");
    //    ros::NodeHandle n;
    ros::NodeHandle nh;
    //    marker_pub = n.advertise<visualization_msgs::Marker>("bounding_box", 10);
    //    ros::Publisher pub_marker = n.advertise<visualization_msgs::MarkerArray>("bounding_box", 10);
    ros::Subscriber sub_radar = nh.subscribe("radar_esr_msg_array_1", 10, callbackRadar);



    pub_marker = nh.advertise<visualization_msgs::MarkerArray>("bounding_box", 100);


    //  ros::spin();
    ros::spin();
    return 0;
    //  ros::Rate r(30);
}
// %EndTag(FULLTEXT)%
