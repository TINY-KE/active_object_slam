//
// Created by zhjd on 11/8/22.
//

#ifndef ACTIVE_EAO_NEW_OBJECT_H
#define ACTIVE_EAO_NEW_OBJECT_H


#include "MapPoint.h"
#include "MapPlane.h"
#include <mutex>
#include <vector>
#include <string>
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

//ros rviz
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

//yolo label
#include "yolo_label.h"

//pcl
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/ModelCoefficients.h>

extern std::string WORK_SPACE_PATH;
extern std::string yamlfile_object;
extern bool MotionIou_flag;
extern bool NoPara_flag;
extern bool ProIou_flag;
extern bool Ttest_flag;
extern bool iforest_flag;
extern bool little_mass_flag;
extern bool ProIou_only30_flag;



namespace ORB_SLAM2
{
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;

class Frame;
class MapPoint;
class KeyFrame;
class Map;
class Object_Map;
class MapPlane;
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
        std::vector<MapPoint*>  mvMapPonits;             // object points in current frame. 存储的是

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
        void AddPotentialAssociatedObjects( std::vector<Object_Map*> obj3ds, int AssoId, int beAssoId);

    protected:
        std::mutex mMutexObjMapPoints;   //对特征点 处理时
        std::mutex mMutexPos;    // 对特征点簇的中心 处理时
    public:
        static std::mutex mGlobalMutex; //未来会用在后端优化中,当前无用

    // line.
    public:
        Eigen::MatrixXd mObjLinesEigen;   //用来保存2d中的line





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

    //g2o::SE3Quat pose ;                               // 6 dof pose.
    cv::Mat pose_mat = cv::Mat::eye(4, 4, CV_32F);      //cv::mat形式的 物体在世界坐标系下的位姿
    //g2o::SE3Quat pose_without_yaw;                    // 6 dof pose without rotation.
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
    cv::Rect mRect_byProjectPoints;    // zhang推测： 这应该是投影到最新一帧的rect
                                // 原mPredictRect. 投影到当前帧的投影框, 根据tTrackMotion中的obj3d->ComputeProjectRectFrameTo(mCurrentFrame),没获取一帧, 地图中物体的投影框就会重新计算
    int mnAddedID_nouse;
    int mnLastAddID;
    int mnLastLastAddID;

public:
    //物体在map中的属性
    int mnId;                           //全局的id
    int mnClass;
    bool bad_3d = false;                //用途：（1）如果为true，则不view  （2）在localMapping、 等地方，应用
    bool backgroud_object = false;      //用途：如果为true，则被参与位姿更新和物体优化
    bool end_build = false;             // 当NBV generator模块认为物体已经建立完毕,就不用再添加新的点了.但是还是可以用于数据关联.
    int mnConfidence_foractive;
    Cuboid3D mCuboid3D;                 // cuboid.
    cv::Mat mSumPointsPos;
    cv::Mat mAveCenter3D;               //点云簇的平均坐标。而非cube的中心坐标
    float mStandar_x, mStandar_y, mStandar_z;
    float mCenterStandar_x, mCenterStandar_y, mCenterStandar_z;
    float mCenterStandar;
    float mIForest_thresh;
    //物体中的特征点
    std::vector< MapPoint*> mvpMapObjectMappoints;
    std::vector< MapPoint*> mvpMapObjectMappoints_NewForActive;     //物体中新添加的特征点，用于更新占据概率地图

    //潜在的可以融合的物体
    std::map<int, int> mReObj;                                      //potential associated objects.  记录潜在的关联对象，会在localmap中进行融合
    std::map<int, int> mmAppearSametime;                            // 共视关系: 两个物体在同一帧中被观测到, 则认为两个物体被共视. 第二项代表被共视的次数. 次数越多, 越有可能是同一个物体

    //物体yaw估计中，各角度的评分
    std::vector<Eigen::Matrix<float,5,1>, Eigen::aligned_allocator<Eigen::Matrix<float,5,1>> > mvAngleTimesAndScore;    // Score of sampling angle.

// ************************************
// object3d 通用函数部分 *
    void ComputeMeanAndDeviation_3D();
    void IsolationForestDeleteOutliers();
    void Update_Twobj();                                                // 原UpdateObjPose  更新物体在世界下的坐标
    void ComputeProjectRectFrameToCurrentFrame(Frame &Frame);           //将obj3d中的point投影到目标frame中，计算obj3d在目标frame中的投影边界框.从而查看obje3d,能够关联目标frame中物体
    bool WhetherOverlap(Object_Map *CompareObj);
    void UpdateCoView(Object_Map *Obj_CoView);
    void AddObj2d(Object_2D* Object_2d){
        std::unique_lock<std::mutex> lock(mMutexObj2d);
        this->mvObject_2ds.push_back(Object_2d);
    }

// ************************************
// object3d track部分 *
    bool UpdateToObject3D(Object_2D* Object_2d, Frame &mCurrentFrame, int Flag);

protected:
    std::mutex mMutexMapPoints;

// ************************************
// object3d localmap部分 *
public:
    static std::mutex mMutex_front_back;                                //用于前端和后端对obj2d操作时,不冲突
    void SearchAndMergeMapObjs_fll(Map *mpMap);
    void MergeTwoMapObjs_fll(Object_Map *RepeatObj);
    bool DoubleSampleTtest_fll(Object_Map *RepeatObj);
    void DealTwoOverlapObjs_fll(Object_Map *OverlapObj, float overlap_x, float overlap_y, float overlap_z);
    void BigToSmall_fll(Object_Map *SmallObj, float overlap_x, float overlap_y, float overlap_z);
    void DivideEquallyTwoObjs_fll(Object_Map *AnotherObj, float overlap_x, float overlap_y, float overlap_z);


// ************************************
// object3d 信息熵计算部分 *
public:
    int mIE_rows, mIE_cols;

    void IE_RecoverInit();
    int mIEThresholdPointNum;                                       //用于判定, 某列grid是否有效
    Eigen::Vector3d mMainDirection;                         //通过特征点计算的主方向,用于view的 ie
    double mStatistics = 1;                                 //检验统计量

    //传感器模型
    double mP_occ;
    double mP_free;
    double mP_prior;

    double mIE;
    //std::vector<std::vector<double> > mvInforEntroy;                   // 没用.  用于存储18*18个栅格的信息熵
    cv::Mat mvInforEntroy_mat;
    std::vector<double> mvInforEntroy_vector;   //一列一列得存储
    //std::vector<std::vector<double> > mvGridProb;                     //用于存储18*18个栅格的占据概率
    cv::Mat mvGridProb_mat;
    std::vector<double> mvGridProb_vector;
    //std::vector<std::vector<int> > mvPointNum;                //存储18*18个栅格中,各自的grid数量
    cv::Mat mvObserveNum_mat;
    std::vector<double> mvPointNum_vector;   //一列一列得存储

    void compute_grid_xy(const Eigen::Vector3d &zero_vec, const Eigen::Vector3d &point_vec, int& x, int& y);
    cv::Mat compute_pointnum_eachgrid();
    void compute_perceptionNum_eachgrid();
    float log2(float x);
    float mIEThresholdEndMapping;
    void compute_occupied_prob_eachgrid();
    double IE(const double &p);
    void ComputeIE();
    void PublishIE();
    int mbPublishIEwheel;
    ros::NodeHandle nh;
    ros::Publisher publisher_IE;
    void ComputeMainDirection();
    double get_information_entroy();
    std::vector<MapPoint* >  GetObjectMappoints();
    std::vector<MapPoint* >  GetNewObjectMappoints();
protected:
    std::mutex mMutexPose;
    std::mutex mMutexObj2d;

// ************************************
// object3d 筛选候选点 *
public:
    bool WheatherInRectFrameOf(const cv::Mat &Tcw, const float &fx, const float &fy, const float &cx, const float &cy, const float &ImageWidth, const float &ImageHeight);  //计算到任意帧的投影


private:
    float mnViewField;

//fake GBV
    int mbFakeGBVs;
    void FakeObjectModel(double x, double y, double z, double length, double width, double height );
};

void cmpute_corner(Object_Map* object) ;

void ReadLocalObjects( const std::string& filePath, std::vector<Object_Map*>& vObjects);



class BackgroudObject {

public:
    enum eState{
        UnExplored=0,
        UnEnd=1,
        End=2
    };

public:
    BackgroudObject();
    ~BackgroudObject();

public:
    int mnId;
    int mnClass = 60;
    int mState = UnExplored;
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr
    PointCloud::Ptr mPlane;         //支撑面
    double mean_x,mean_y,mean_z;	//物体中心
    double max_x,max_y,max_z;
    double min_x,min_y,min_z;
    double length,width,height;
    //bool end_activemapping = false;
    double IEvalue;
    double FO_num, FO_num_not_end ;
    cv::Mat pose_mat = cv::Mat::eye(4, 4, CV_32F);
    std::vector<Object_Map*> mvFOs;
    int mnObserveMaxNum = 10;
    int mnObserveNum = 0;

public:
    bool include(Object_Map* fo);
    bool AllInclude(std::vector<Object_Map*> fos);
    void IncludeFOs_and_WheatherEndActive(const std::vector<Object_Map*> &FOs);

    void computePose(double yaw = 0.0);

    bool return_end_ASLAM();


};

}








#endif //ACTIVE_EAO_NEW_OBJECT_H
