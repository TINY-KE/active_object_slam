//
// Created by zhjd on 11/17/22.
//

#ifndef ACTIVE_EAO_NEW_NBVGENERATOR_H
#define ACTIVE_EAO_NEW_NBVGENERATOR_H

//ros
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

//内部
#include "Map.h"
#include "Object.h"
#include "Converter.h"
#include "Tracking.h"

namespace ORB_SLAM2
{
class Tracking;
class FrameDrawer;
class MapPublisher;
class MapDrawer;
class System;

class NbvGenerator {

public:
    NbvGenerator();
    NbvGenerator(Map* map, Tracking *pTracking, const string &strSettingPath);

    std::vector<cv::Mat> mvCandidates;

    void Run();

    void RequestFinish();

private:
    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    bool mbStopped;
    bool mbStopRequested;
    std::mutex mMutexStop;

private:
    Map* mpMap;
    Tracking* mpTracker;

    ros::NodeHandle nh;
    visualization_msgs::Marker mCandidate;
    visualization_msgs::Marker mPlanesCentroid;

    const char* CANDIDATE_NAMESPACE = "Candidate";
    const char* MAP_FRAME_ID = "map"; //  odom   imu_link   /ORB_SLAM/World    map
    float fCameraSize;
    float fPointSize;

    ros::Publisher publisher_centroid;
    ros::Publisher pubCloud;
    ros::Publisher publisher_candidate;

    vector<cv::Mat> mvAllCandidate;
    pcl::PointCloud<pcl::PointXYZ>::Ptr mCloud_boundary;
    void ExtractCandidates(const vector<MapPlane *> &vpMPs);
    void ExtractNBV();
    void PublishMapPlanes();
    void BoundaryExtraction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boundary, int resolution);
    void PublishCurrentCamera(const vector<cv::Mat> &Tcw);

};

}


#endif //ACTIVE_EAO_NEW_NBVGENERATOR_H
