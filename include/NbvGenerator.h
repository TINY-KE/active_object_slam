//
// Created by zhjd on 11/17/22.
//

#ifndef ACTIVE_EAO_NEW_NBVGENERATOR_H
#define ACTIVE_EAO_NEW_NBVGENERATOR_H

//ros
#include <ros/ros.h>
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

//movebase action
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;



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
    BackgroudObject* bo;
};

struct localCandidate{
    double angle;
    double reward;
    int num;
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
    int  mbPubNavGoal;

    const char* CANDIDATE_NAMESPACE = "Candidate";
    const char* MAP_FRAME_ID = "map"; //  odom   imu_link   /ORB_SLAM/World    map
    float fCameraSize;
    float fPointSize;
    bool mbEnd_active_map = false;

    ros::Publisher publisher_centroid;
    ros::Publisher publisher_candidate;
    int mtest = 5;
    ros::Publisher publisher_candidate_unsort;
    ros::Publisher publisher_nbv;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>* mActionlib;
    vector<Candidate> mvGlobalCandidate;
    vector<localCandidate> mvLocalCandidate;
    Candidate NBV;
    vector<Candidate> mNBVs_old; //存储已到达的NBV，从而使下一个NBV尽量已到达的位置。
    double mNBVs_scale = 0;
    vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> mvCloudBoundary;
    vector<PointCloud::Ptr> mvPlanes_filter;
    vector<BackgroudObject*> mvBackgroud_objects;

    void Filter_BackgroudObjects_and_Extract_Candidates(const vector<MapPlane *> &vpMPls,  const vector<Object_Map*> &ForegroundObjectMaps);
    void Filter_BackgroudObjects(const vector<MapPlane *> &vpMPls,  const vector<Object_Map*> &ForegroundObjectMaps);
    void Extract_Candidates( BackgroudObject* bo_first );
    void Extract_Candidates();
    void ExtractCandidates(const vector<MapPlane *> &vpMPs);

    double IntersectionScale(const cv::Point2f& p1Start, const cv::Point2f& p1End, const cv::Point2f& p2Start, const cv::Point2f& p2End);
    bool computeIntersection(const cv::Point2f& rayStart, const cv::Point2f& rayEnd, const cv::Point2f& segmentStart, const cv::Point2f& segmentEnd, cv::Point2f& SafeNBVPoint);
    cv::Point2f normalize(cv::Point2f ray_n  );

    vector<Candidate> RotateCandidates(Candidate& initPose);
    double computeCosAngle_Signed(Eigen::Vector3d &v1,  Eigen::Vector3d &v2 , bool isSigned);
    void computeReward(Candidate &candidate); //, vector<Object_Map*> obj3ds

    void BoundaryExtraction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boundary, int resolution);
    void PublishGlobalNBVRviz(const vector<Candidate> &candidates);
    void addOldNBV(Candidate &candidate);
    void clearOldNBV();

//plane和背景物体的可视化
private:
    ros::Publisher pubCloud;
    ros::Publisher publisher_object_backgroud;
    //MapPublisher mappublisher;
    void PublishBackgroudObjects_and_SupportingPlane();
    void publishBackgroudObject(pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane );
    void publishBackgroudObject( BackgroudObject* bo );
    geometry_msgs::Point corner_to_marker(Eigen::Vector3d& v);


private:
    float mfx, mfy, mcx, mcy;
    float mImageWidth, mImageHeight;
    float down_nbv_height;       //nbv的高度
    float mMaxPlaneHeight, mMinPlaneHeight, mMinPlaneSafeRadius, mGobalCandidateNum;
    cv::Mat mT_basefootprint_cam;       //相机在机器人底盘上的坐标
    cv::Mat mT_world_initbaselink;     //初始机器人底盘在世界中的坐标
    cv::Mat mT_world_cam;     //初始相机在世界中的坐标
    double mNBV_Angle_correct;
    int mCandidate_num_topub; //展示的候选观测点的数量
    int mBackgroudObjectNum;    //背景物体的数量。先验
    double mPitch;  //相机的俯仰角
    //double mgreat_angle = 0;
    //std::mutex mMutexMamAngle;
    //double getMamGreadAngle();
    string mstrSettingPath;
    float mReward_dis;

//NBV MAM
private:
    double mGreat_angle = 0;
    float mDivide;
    int MAM_neckdelay;
    int mMAM_turn;
    int MAM_isused;
    float mLocalNeckRange;
    std::mutex mMutexMamAngle;
    float  mMax_dis;
    float  mMin_dis;
    ros::Publisher publisher_mam;
    ros::Publisher publisher_mam_rviz;
    void publishLocalNBV();
    float mTfDuration;
    void publishNeckAngle(double angle);
};

}


#endif //ACTIVE_EAO_NEW_NBVGENERATOR_H
