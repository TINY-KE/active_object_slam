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
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/common/io.h>
#include <pcl/common/impl/io.hpp>

// pcl边界
#include <pcl/features/boundary.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>

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
    ros::Publisher publisher_candidate;
    ros::Publisher publisher_IE;

    visualization_msgs::Marker mPoints;
    visualization_msgs::Marker mReferencePoints;
    visualization_msgs::Marker mKeyFrames;
    visualization_msgs::Marker mReferenceKeyFrames;
    visualization_msgs::Marker mCovisibilityGraph;
    visualization_msgs::Marker mMST;
    visualization_msgs::Marker mCurrentCamera;

    visualization_msgs::Marker mPlanes;
    visualization_msgs::Marker mCandidate;

    int object_id_init;

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
    const char* CANDIDATE_NAMESPACE = "Candidate";

//物体
public:

//plane
public:
    typedef pcl::PointXYZRGB PointT;
    typedef pcl::PointCloud<PointT> PointCloud;
    void DrawMapPlanes();
    int DrawMapPlane_i = 0 ;
    void PublishMapPlanes(const vector<MapPlane *> &vpMPs);
    ros::Publisher pubCloud;

    void BoundaryExtraction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boundary, int resolution)
    {   // BoundaryExtraction(cloud, cloud_boundary, 200);
        pcl::PointXYZ px_min = *std::min_element(cloud->begin(), cloud->end(), [](pcl::PointXYZ pt1, pcl::PointXYZ pt2) {return pt1.x < pt2.x; });
        pcl::PointXYZ px_max = *std::max_element(cloud->begin(), cloud->end(), [](pcl::PointXYZ pt1, pcl::PointXYZ pt2) {return pt1.x < pt2.x; });

        float delta_x = (px_max.x - px_min.x) / resolution;
        float min_y = INT_MAX, max_y = -INT_MAX;
        std::vector<int> indexs_x(2 * resolution);
        std::vector<std::pair<float, float>> minmax_x(resolution, { INT_MAX,-INT_MAX });
        for (size_t i = 0; i < cloud->size(); ++i)
        {
            int id = (cloud->points[i].x - px_min.x) / delta_x;
            if (cloud->points[i].y < minmax_x[id].first)
            {
                minmax_x[id].first = cloud->points[i].y;
                indexs_x[id] = i;
            }
            else if (cloud->points[i].y > minmax_x[id].second)
            {
                minmax_x[id].second = cloud->points[i].y;
                indexs_x[id + resolution] = i;
            }
        }

        pcl::PointXYZ py_min = *std::min_element(cloud->begin(), cloud->end(), [](pcl::PointXYZ pt1, pcl::PointXYZ pt2) {return pt1.y < pt2.y; });
        pcl::PointXYZ py_max = *std::max_element(cloud->begin(), cloud->end(), [](pcl::PointXYZ pt1, pcl::PointXYZ pt2) {return pt1.y < pt2.y; });

        float delta_y = (py_max.y - py_min.y) / resolution;
        float min_x = INT_MAX, max_x = -INT_MAX;
        std::vector<int> indexs_y(2 * resolution);
        std::vector<std::pair<float, float>> minmax_y(resolution, { INT_MAX,-INT_MAX });
        for (size_t i = 0; i < cloud->size(); ++i)
        {
            int id = (cloud->points[i].y - py_min.y) / delta_y;
            if (cloud->points[i].x < minmax_y[id].first)
            {
                minmax_y[id].first = cloud->points[i].x;
                indexs_y[id] = i;
            }
            else if (cloud->points[i].x > minmax_y[id].second)
            {
                minmax_y[id].second = cloud->points[i].x;
                indexs_y[id + resolution] = i;
            }
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xboundary(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*cloud, indexs_x, *cloud_xboundary);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_yboundary(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*cloud, indexs_y, *cloud_yboundary);
        *cloud_boundary = *cloud_xboundary + *cloud_yboundary;
    }


};

} //namespace ORB_SLAM

#endif // MAPPUBLISHER_H
