//
// Created by zhjd on 11/8/22.
//

#ifndef ACTIVE_EAO_NEW_OBJECT_H
#define ACTIVE_EAO_NEW_OBJECT_H


#include "MapPoint.h"
#include <mutex>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <eigen3/Eigen/Dense>
#include <algorithm>
//数值计算
#include <numeric>
#include <math.h>
#include <assert.h>
#include <iostream>
#include "YOLOv3SE.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"
#include "isolation_forest.h"


extern std::string WORK_SPACE_PATH;
extern bool MotionIou_flag;
extern bool NoPara_flag;
extern bool ProIou_flag;
extern bool Ttest_flag;
extern bool iforest_flag;
extern bool little_mass_flag;
extern bool ProIou_only30_flag;

namespace ORB_SLAM2
{
class Frame;
class MapPoint;
class KeyFrame;
class Map;
class Object_Map;

class Object_2D;
enum eAssociateFlag{
        MotionIou=1,
        NoPara=2,
        t_test=3,
        ProIou=4 //Debug=4
    };


class Object_2D {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    public:
        Object_2D();

        //Object_2D(const Object_2D* obj2d){
        //    std::cout<<"Object_2D  construct 2   ";
        //    mclass_id = obj2d->mclass_id;
        //    mScore = obj2d->mScore;
        //    mleft = obj2d->mleft;
        //    mright = obj2d->mright;
        //    mtop = obj2d->mtop;
        //    mbottom = obj2d->mbottom;
        //    mWidth = obj2d->mWidth;
        //    mHeight = obj2d->mHeight;
        //    // 2d center.
        //    mBoxCenter_2d = obj2d->mBoxCenter_2d;
        //    // opencv Rect format.
        //    mBox_cvRect = obj2d->mBox_cvRect;
        //    mBox_cvRect_FeaturePoints = obj2d->mBox_cvRect_FeaturePoints;
        //    this->mpCurrentFrame = obj2d->mpCurrentFrame;
        //    this->mpMap = obj2d->mpMap;
        //
        //    //位姿
        //    this->sum_pos_3d = obj2d->sum_pos_3d;
        //    this->mPos_world = obj2d->mPos_world;
        //    this->mvMapPonits = obj2d->mvMapPonits;
        //    mReIdAndIou = obj2d->mReIdAndIou;
        //
        //    bad = false;
        //    std::cout<<">>>   End"<<std::endl;
        //};
        Object_2D(Map* mpMap, Frame *CurrentFrame, const BoxSE &box);
    public:
        //yolo
        int mclass_id;                                                  // class id.
        float mScore;                                                   // Probability.
        float mleft;                                                    // size.
        float mright;
        float mtop;
        float mbottom;
        float mWidth;
        float mHeight;
        cv::Rect mBox_cvRect;                   // cv::Rect format.
        cv::Rect mBox_cvRect_FeaturePoints;     // the bounding box constructed by object feature points.
                                                // 在tracker处理object2d时，生成的mBox_cvRect_FeaturePoints。用途： 用在2d和3d数据关联中的ProjectIou
        //BoxSE mBox_yoloSE;                      // BoxSE
        cv::Point2f mBoxCenter_2d;              // 2D center.

        // Frame and Map
        Frame* mpCurrentFrame;                  // 提取Object_2D的frame。可从中获取T和R
        Map* mpMap;

        // coordinate
        cv::Mat sum_pos_3d;                         // Summation of points observed in the current frame.
        cv::Mat mPos_world;                         // current object center (3d, world). 通过当前帧的观测框中的point，计算而来
        //cv::Mat mCenter_ofAssMapObj;                // map object center.   通过数据关联map中的物体的中心坐标
        //float mStandar_x, mStandar_y, mStandar_z; // standard deviation  注释的原因： 2d中心的坐标在计算3d坐标的偏差时有用。2d中心的偏差有什么用？

        //mappoint
        vector<MapPoint*>  mvMapPonits;             // object points in current frame. 存储的是

        //object在map中的属性
        int mnId;                                   // object ID.
        int confidence;
        bool bad = false;

        // 待确定
        int nMayRepeat = 0;
        std::map<int, float> mReIdAndIou;           // potential objects.

        void AddYoloBoxes(const BoxSE &box);            // copy box to object_2d.
        void ComputeMeanAndDeviation();                 // compute the mean and standard deviation of object points in current frame.
        void RemoveOutlier_ByHeightorDepth();                  // remove outliers by boxplot.
        void MergeTwo_Obj2D(Object_2D *Old_Object2D);
        int Object2D_DataAssociationWith_Object3D();    // data association.
        int creatObject();
        int  NoParaDataAssociation(Object_Map* ObjectMapSingle); // NP.

        void AddObjectPoint(MapPoint *pMP);
        void AddPotentialAssociatedObjects( vector<Object_Map*> obj3ds, int AssoId, int beAssoId);

    protected:
        std::mutex mMutexObjMapPoints;   //对特征点 处理时
        std::mutex mMutexPos;    // 对特征点簇的中心 处理时
    public:
        static std::mutex mGlobalMutex; //未来会用在后端优化中,当前无用

    // line.
    //public:
    //    Eigen::MatrixXd mObjLinesEigen;   //用来保存2d中的line





};







struct Cuboid3D{
    //     7------6
    //    /|     /|
    //   / |    / |
    //  4------5  |
    //  |  3---|--2
    //  | /    | /
    //  0------1
    // lenth ：corner_2[0] - corner_1[0]
    // width ：corner_2[1] - corner_3[1]
    // height：corner_2[2] - corner_6[2]

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
    // 8 vertices.
    Eigen::Vector3d corner_1;
    Eigen::Vector3d corner_2;
    Eigen::Vector3d corner_3;
    Eigen::Vector3d corner_4;
    Eigen::Vector3d corner_5;
    Eigen::Vector3d corner_6;
    Eigen::Vector3d corner_7;
    Eigen::Vector3d corner_8;

    // 8 vertices (without rotation).
    Eigen::Vector3d corner_1_w;
    Eigen::Vector3d corner_2_w;
    Eigen::Vector3d corner_3_w;
    Eigen::Vector3d corner_4_w;
    Eigen::Vector3d corner_5_w;
    Eigen::Vector3d corner_6_w;
    Eigen::Vector3d corner_7_w;
    Eigen::Vector3d corner_8_w;

    Eigen::Vector3d cuboidCenter;       // the center of the Cube, not the center of mass of the object
    float x_min, x_max, y_min, y_max, z_min, z_max;     // the boundary in XYZ direction.

    float lenth;
    float width;
    float height;
    float mfRMax;      // 中心点与角点的最大半径

    //g2o::SE3Quat pose ;                      // 6 dof pose.
    cv::Mat pose_mat = cv::Mat::eye(4, 4, CV_32F);   //cv::mat形式的 物体在世界坐标系下的位姿
    //g2o::SE3Quat pose_without_yaw;          // 6 dof pose without rotation.
    cv::Mat pose_noyaw_mat = cv::Mat::eye(4, 4, CV_32F);
    // angle.
    float rotY = 0.0;
    float rotP = 0.0;
    float rotR = 0.0;

    // line.
    float mfErrorParallel;
    float mfErroeYaw;
};




class Object_Map{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
    Object_Map();

public:
    std::vector<Object_2D* > mvObject_2ds;  //在各frame中的object_2d  原mObjectFrame , Eigen::aligned_allocator<Object_2D*>

    //yolo检测框和观测帧的id, 用于iou数据关联
    cv::Rect mLastRect;
    cv::Rect mLastLastRect;
    //cv::Rect mPredictRect;
    cv::Rect mRect_byProjectPoints;    //投影到当前帧的投影框, 根据tTrackMotion中的obj3d->ComputeProjectRectFrameTo(mCurrentFrame),没获取一帧, 地图中物体的投影框就会重新计算
    int mnAddedID_nouse;
    int mnLastAddID;
    int mnLastLastAddID;

public:
    //物体在map中的属性
    int mnId;                   //全局的id
    int mnClass;
    bool bad_3d = false;     //zhangjiadong  用途：（1）如果为true，则不view  （2）在localMapping、 等地方，应用
    int mnConfidence_foractive;
    Cuboid3D mCuboid3D;         // cuboid.
    cv::Mat mSumPointsPos;
    cv::Mat mAveCenter3D;       //点云簇的平均坐标。而非cube的中心坐标
    float mStandar_x, mStandar_y, mStandar_z;
    float mCenterStandar_x, mCenterStandar_y, mCenterStandar_z;
    float mCenterStandar;
    //int nMayRepeat = 0;                 // maybe a repeat object.


    //物体中的特征点
    vector< MapPoint*> mvpMapObjectMappoints;
    vector< MapPoint*> mvpMapObjectMappoints_NewForActive;
    //vector< MapPoint*> mvpMapCurrentNewMappoints;  //物体中新添加的特征点，用于更新占据概率地图


    //NoPara数据关联. 潜在的关联对象
    std::map<int, int> mReObj;          // potential associated objects.  记录潜在的
    std::map<int, int> mmAppearSametime;// 共视关系: 两个物体在同一帧中被观测到, 则认为两个物体被共视. 第二项代表被共视的次数. 次数越多, 越有可能是同一个物体

    std::vector<Eigen::Matrix<float,5,1>, Eigen::aligned_allocator<Eigen::Matrix<float,5,1>> > mvAngleTimesAndScore;    // Score of sampling angle.

    void ComputeMeanAndDeviation_3D();
    void IsolationForestDeleteOutliers();
    bool UpdateToObject3D(Object_2D* Object_2d, Frame &mCurrentFrame, int Flag);
    void Update_Twobj();      // 原UpdateObjPose  更新物体在世界下的坐标
    void ComputeProjectRectFrameTo(Frame &Frame);  //将obj3d中的point投影到目标frame中，计算obj3d在目标frame中的投影边界框.从而查看obje3d,能够关联目标frame中物体

    bool WhetherOverlap(Object_Map *CompareObj);

//   void UpdateObjScale(Eigen::Vector3d Scale);    // for optimization.
    void UpdateCoView(Object_Map *Obj_CoView);

protected:
    std::mutex mMutexMapPoints;


//localmap 部分   fll指forlocalmap
public:
    static std::mutex mMutex_front_back;
    void SearchAndMergeMapObjs_fll(Map *mpMap);
    void MergeTwoMapObjs_fll(Object_Map *RepeatObj);
    bool DoubleSampleTtest_fll(Object_Map *RepeatObj);
    void DealTwoOverlapObjs_fll(Object_Map *OverlapObj, float overlap_x, float overlap_y, float overlap_z);
    void BigToSmall_fll(Object_Map *SmallObj, float overlap_x, float overlap_y, float overlap_z);
    void DivideEquallyTwoObjs_fll(Object_Map *AnotherObj, float overlap_x, float overlap_y, float overlap_z);


//信息熵 部分
public:
    void init_information_entroy();

    int threshold = 3;  //用于判定, 某列grid是否有效

    Eigen::Vector3d mMainDirection;  //通过特征点计算的主方向,用于view的 ie

    //传感器模型
    double P_occ=0.6;
    double P_free=0.4;
    double P_prior=0.5;

    vector<vector<double> > vInforEntroy;  // 没用.  用于存储18*18个栅格的信息熵
    vector<vector<double> > vgrid_prob;  //用于存储18*18个栅格的占据概率
    vector<vector<int> > vpointnum_eachgrid;   //存储18*18个栅格中,各自的grid数量

    void grid_index(const Eigen::Vector3d &zero_vec, const Eigen::Vector3d &point_vec, int& x, int& y);

    void compute_pointnum_eachgrid();

    void compute_occupied_prob();

    double information_entroy(const double &p);

    void compute_information_entroy();

    double get_information_entroy();

    vector<MapPoint* >  GetObjectMappoints();

    vector<MapPoint* >  GetNewObjectMappoints();
protected:
    std::mutex mMutexPose;
    std::mutex mMutexObj2d;
public:
    void AddObj2d(Object_2D* Object_2d){
        unique_lock<mutex> lock(mMutexObj2d);
        this->mvObject_2ds.push_back(Object_2d);
    }
};


}

#endif //ACTIVE_EAO_NEW_OBJECT_H
