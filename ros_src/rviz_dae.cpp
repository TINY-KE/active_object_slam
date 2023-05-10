/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>

#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>



int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");

    ros::NodeHandle n;
    ros::Rate r(10);

    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("dae_model_marker", 1);

    visualization_msgs::Marker marker;
    double scale = 0.145;
    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = scale;
    marker.lifetime.sec = 10000;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.mesh_resource = "file:///home/zhjd/active_eao/src/active_eao/mesh/bear_brown.dae";
    marker.mesh_use_embedded_materials = true;
    marker.header.frame_id = "/map";
    marker.pose.position.x = 0.887667 + 0.05;   //0.887667 -0.104503 1.244793
    marker.pose.position.y = -0.104503;
    marker.pose.position.z = 1.244793 + 0.21;
    marker.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI/180.0  *  -1 * 85);  //-M_PI/2.0

    while (ros::ok())
      {
        marker_pub.publish(marker);

        ros::spinOnce();
        r.sleep();
      }

    return 0;
}
