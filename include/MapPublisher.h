/**
* This file is part of ORB-SLAM.
*
* Copyright (C) 2014 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <http://webdiis.unizar.es/~raulmur/orbslam/>
*
* ORB-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef MAPPUBLISHER_H
#define MAPPUBLISHER_H

#include<ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include"Map.h"
#include"MapPoint.h"
#include "MapPlane.h"
#include"KeyFrame.h"
#include "Object.h"
#include "Converter.h"
#include <random>
#include <mutex>
// #include "Plane3D.h"
namespace ORB_SLAM2
{
class MapPlane;

class MapPublisher
{
public:
    MapPublisher(Map* pMap);

    Map* mpMap;

    void Refresh();
    void PublishMapPoints(const std::vector<MapPoint*> &vpMPs, const std::vector<MapPoint*> &vpRefMPs);
    void PublishKeyFrames(const std::vector<KeyFrame*> &vpKFs);
    void PublishCurrentCamera(const cv::Mat &Tcw);
    //void PublishPlane(const vector<MapPlane *> &vpMPls );
    void PublishObject(const vector<Object_Map*> &vpObjs );
    void PublishIE(const vector<Object_Map*> &vObjs );
    geometry_msgs::Point corner_to_marker(Eigen::Vector3d& v);
    geometry_msgs::Point corner_to_marker(const std::vector<float>& v);
    geometry_msgs::Point corner_to_marker(const std::vector<double>& v);
    void SetCurrentCameraPose(const cv::Mat &Tcw);

private:

    cv::Mat GetCurrentCameraPose();
    bool isCamUpdated();
    void ResetCamFlag();

    ros::NodeHandle nh;
    ros::Publisher publisher;
    ros::Publisher publisher_IE;

    visualization_msgs::Marker mPoints;
    visualization_msgs::Marker mReferencePoints;
    visualization_msgs::Marker mKeyFrames;
    visualization_msgs::Marker mReferenceKeyFrames;
    visualization_msgs::Marker mCovisibilityGraph;
    visualization_msgs::Marker mMST;
    visualization_msgs::Marker mCurrentCamera;

    visualization_msgs::Marker mPlanes;

    int object_id_init = 8;

    float fCameraSize;
    float fPointSize;

    cv::Mat mCameraPose;// = cv::Mat::eye(4,1,CV_32F);
    bool mbCameraUpdated;

    std::mutex mMutexCamera;

    const char* MAP_FRAME_ID = "map"; //  odom   imu_link   /ORB_SLAM/World    map
    const char* POINTS_NAMESPACE = "MapPoints";
    const char* KEYFRAMES_NAMESPACE = "KeyFrames";
    const char* PLANES_NAMESPACE = "MapPlanes";
    const char* OBJECTS_NAMESPACE = "MapObjects";
    const char* GRAPH_NAMESPACE = "Graph";
    const char* CAMERA_NAMESPACE = "Camera";

//物体
public:

//plane
public:
    typedef pcl::PointXYZRGB PointT;
    typedef pcl::PointCloud<PointT> PointCloud;
    void DrawMapPlanes();
    int DrawMapPlane_i = 0 ;
    void DrawMapPlanesOld(const vector<MapPlane *> &vpMPs);
    ros::Publisher pubCloud;
};

} //namespace ORB_SLAM

#endif // MAPPUBLISHER_H
