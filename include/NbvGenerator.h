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

//eigen cv的convert
#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>
#include<opencv2/core/eigen.hpp>

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

struct Candidate{
    cv::Mat pose;
    double reward;
};

class NbvGenerator {

public:
    NbvGenerator();
    NbvGenerator(Map* map, Tracking *pTracking, const string &strSettingPath);

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
    int mtest = 5;
    ros::Publisher publisher_candidate_unsort;

    vector<Candidate> mvGlobalCandidate;
    vector<Candidate> mvLocalCandidate;
    Candidate NBV;
    vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> mvCloudBoundary;
    vector<PointCloud::Ptr> mvPlanes_filter;

    void ExtractCandidates(const vector<MapPlane *> &vpMPs);
    vector<Candidate> RotateCandidates(Candidate& initPose);
    double computeCosAngle(cv::Mat &candidate, cv::Mat &objectPose, Eigen::Vector3d &ie);
    void computeReward(Candidate &candidate, vector<Object_Map*> obj3ds);
    void ExtractNBV();
    void PublishPlanesAndCamera();
    void PublishNBV();
    void BoundaryExtraction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boundary, int resolution);
    void PublishCamera(const vector<Candidate> &candidates);

public: //NBV MAM
    float mfx, mfy, mcx, mcy;
    float mImageWidth, mImageHeight;
    float mdivide;
    float down_nbv_height;       //nbv的高度
    cv::Mat mT_body_cam;   //相机在机器人上的坐标
    double mPitch;  //相机的俯仰角
    //double mgreat_angle = 0;
    //std::mutex mMutexMamAngle;
    //double getMamGreadAngle();
    string mstrSettingPath;
};

}


#endif //ACTIVE_EAO_NEW_NBVGENERATOR_H
