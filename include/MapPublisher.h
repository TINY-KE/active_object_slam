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
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
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

//eigen cv的convert
#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>
#include<opencv2/core/eigen.hpp>

// #include "Plane3D.h"
namespace ORB_SLAM2
{
class MapPlane;

class MapPublisher
{
public:
    MapPublisher(Map* pMap, const string &strSettingPath);

    Map* mpMap;

    void Refresh();
    void PublishMapPoints(const std::vector<MapPoint*> &vpMPs, const std::vector<MapPoint*> &vpRefMPs);
    void PublishKeyFrames(const std::vector<KeyFrame*> &vpKFs);
    void PublishCurrentCamera(const cv::Mat &Tcw);
    //void PublishPlane(const vector<MapPlane *> &vpMPls );
    void PublishObject(const vector<Object_Map*> &vpObjs );
    void PublishIE(const vector<Object_Map*> &vObjs );
    void PublishMainDirection(const vector<Object_Map*> &vObjs );
    geometry_msgs::Point corner_to_marker(Eigen::Vector3d& v);
    geometry_msgs::Point corner_to_marker(const std::vector<float>& v);
    geometry_msgs::Point corner_to_marker(const std::vector<double>& v);
    void SetCurrentCameraPose(const cv::Mat &Tcw);

//NBV MAM
public:
    void SetMAM(const double &angle);
private:
    double mMAM_angle;
    bool mbMAMUpdated = false;
//NBV MAM end

private:

    cv::Mat GetCurrentCameraPose();
    bool isCamUpdated();
    void ResetCamFlag();

    ros::NodeHandle nh;
    ros::Publisher publisher;
    ros::Publisher publisher_curframe;
    ros::Publisher publisher_KF;
    ros::Publisher publisher_CoView;
    ros::Publisher publisher_object;
    ros::Publisher publisher_object_points;
    ros::Publisher publisher_IE_maindirection;
    ros::Publisher publisher_IE_cylinder;
    ros::Publisher publisher_IE_ellipse;
    ros::Publisher publisher_IE_half;
    ros::Publisher publisher_MainDirection;
    ros::Publisher publisher_SumMainDirection;
    //ros::Publisher publisher_robotpose;
    //ros::Publisher publisher_mam_rviz;   //NBV MAM
    ros::Publisher publisher_IEtext;

    //tf tree
    //tf::TransformBroadcaster odom_broadcaster;
    //tf::TransformBroadcaster camera_broadcaster;

    visualization_msgs::Marker mPoints;
    visualization_msgs::Marker mReferencePoints;
    visualization_msgs::Marker mKeyFrames;
    visualization_msgs::Marker mReferenceKeyFrames;
    visualization_msgs::Marker mCovisibilityGraph;
    visualization_msgs::Marker mMST;
    visualization_msgs::Marker mCurrentCamera;
    visualization_msgs::Marker mIEtext;


    int object_id_init;
    int IE_id;
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

    float mObject_Duration;
    float mDirection_Duration;
    float mIE_Duration;

//plane
public:
    //typedef pcl::PointXYZRGB PointT;
    //typedef pcl::PointCloud<PointT> PointCloud;

//机器人底盘的位姿
public:
    cv::Mat mT_body_cam;   //相机在机器人上的坐标
    geometry_msgs::Quaternion mQuaternion_robot_camera;   //机器人到相机坐标系的旋转关系
    geometry_msgs::Vector3_<float> mTranslation_robot_camera;   //机器人到相机坐标系的平移关系
};

} //namespace ORB_SLAM

#endif // MAPPUBLISHER_H
