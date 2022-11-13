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
        Object_2D();
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

    //crash bug
    protected:
        std::mutex mMutexObjMapPoints;   //对特征点 处理时
        std::mutex mMutexPos;    // 对特征点簇的中心 处理时
    public:
        static std::mutex mGlobalMutex; //未来会用在后端优化中,当前无用
        //cv::Mat GetWorldPos();
        //cv::Mat Get_Sum_Points_Pos();
        //void SetWorldPos(const cv::Mat &Pos);

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

    //// 8 vertices.
    //vector<double>  corner_1 {0, 0, 0};
    //vector<double>  corner_2 {0, 0, 0};
    //vector<double>  corner_3 {0, 0, 0};
    //vector<double>  corner_4 {0, 0, 0};
    //vector<double>  corner_5 {0, 0, 0};
    //vector<double>  corner_6 {0, 0, 0};
    //vector<double>  corner_7 {0, 0, 0};
    //vector<double>  corner_8 {0, 0, 0};
    //
    //// 8 vertices (without rotation).
    //vector<double>  corner_1_w {0, 0, 0};
    //vector<double>  corner_2_w {0, 0, 0};
    //vector<double>  corner_3_w {0, 0, 0};
    //vector<double>  corner_4_w {0, 0, 0};
    //vector<double>  corner_5_w {0, 0, 0};
    //vector<double>  corner_6_w {0, 0, 0};
    //vector<double>  corner_7_w {0, 0, 0};
    //vector<double>  corner_8_w {0, 0, 0};
    //
    //float x_min, x_max, y_min, y_max, z_min, z_max;     // the boundary in XYZ direction.
    //vector<double>  cuboidCenter {0, 0, 0};      // the center of the Cube, not the center of mass of the object
    //float  cuboidCenter0,  cuboidCenter1,  cuboidCenter2;


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
    Object_Map(){
        //init_information_entroy();
    }

public:
    std::vector<Object_2D*> mvObject_2ds;  //在各frame中的object_2d

    //yolo检测框和观测帧的id, 用于iou数据关联
    cv::Rect mLastRect;
    cv::Rect mLastLastRect;
    //cv::Rect mPredictRect;
    cv::Rect mRect_byProjectPoints;
    int mnAddedID;
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

    //std::vector<Vector5f> mvAngleTimesAndScore;    // Score of sampling angle.

    void ComputeMeanAndDeviation_3D();
    void IsolationForestDeleteOutliers();
    bool UpdateToObject3D(Object_2D* Object_2d, Frame &mCurrentFrame, int Flag);
    void Update_Twobj();      // update object pose.
//
    void ComputeProjectRectFrame(Frame &Frame);  //将obj3d中的point投影到目标frame中，计算obj3d在目标frame中的投影边界框.从而查看obje3d,能够关联目标frame中物体
//    void WhetherMergeTwoMapObjs_forlocalmap(Map *mpMap);
//    void MergeTwoMapObjs_forlocalmap(Object_Map *RepeatObj);
//    bool DoubleSampleTtest(Object_Map *RepeatObj);
//    void DealTwoOverlapObjs_forlocalmap(Object_Map *OverlapObj, float overlap_x, float overlap_y, float overlap_z);
    bool WhetherOverlap(Object_Map *CompareObj);
//    void BigToSmall_forlocalmap(Object_Map *SmallObj, float overlap_x, float overlap_y, float overlap_z);
//    void DivideEquallyTwoObjs_forlocalmap(Object_Map *AnotherObj, float overlap_x, float overlap_y, float overlap_z);
//
//    // void UpdateObjScale(Eigen::Vector3d Scale);    // for optimization.

    void UpdateCoView(Object_Map *Obj_CoView);
protected:
    std::mutex mMutexMapPoints;
    std::mutex mMutex;

//public:
//    void init_information_entroy();

    //vector< MapPoint*> mvpMapObjectMappoints;
    //Cuboid3D mCuboid3D;

    int threshold = 3;  //用于判定, 某列grid是否有效

    //Vector3D mMainDirection;  //通过特征点计算的主方向,用于view的 ie

    //传感器模型
    double P_occ=0.6;
    double P_free=0.4;
    double P_prior=0.5;

    //占据状态值

    //
    vector<vector<double> > vInforEntroy;  // 没用.  用于存储18*18个栅格的信息熵
    vector<vector<double> > vgrid_prob;  //用于存储18*18个栅格的占据概率
    vector<vector<int> > vpointnum_eachgrid;   //存储18*18个栅格中,各自的grid数量

    //void grid_index(const Vector3d &zero_vec, const Vector3d &point_vec, int& x, int& y);

    //void compute_pointnum_eachgrid();
    //
    //void compute_occupied_prob();
    //
    //double information_entroy(const double &p);
    //
    //void compute_information_entroy();
    //
    //double get_information_entroy();


protected:
    std::mutex mMutexPose;


};


}

#endif //ACTIVE_EAO_NEW_OBJECT_H
