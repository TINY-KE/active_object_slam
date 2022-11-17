

// 该文件作为基本ros模板.

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
// darknet_ros_msgs
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/ObjectCount.h>
#include <std_msgs/Float64.h>
using namespace sensor_msgs;

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

// 多帧数据同步
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
using namespace message_filters;


// #include <tf/Quat>
using namespace geometry_msgs;

int i = 0;
#include <Eigen/Core>
// #include <Eigen>
#include <System.h>
#include "Global.h"
#include <Eigen/Dense>
#include <eigen3/Eigen/Dense>

// imu的相关工具
#include "tf/transform_datatypes.h"//转换函数头文件
#include <sensor_msgs/Imu.h>//imu数据信息
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>

using namespace std;
std::string WORK_SPACE_PATH = "";
ORB_SLAM2::System* system_ptr;

//NBV MAM
ros::Publisher pub_mam;
int loop = 0;

//std::string DatasetType;
Eigen::Matrix4d INIT_POSE = Eigen::Matrix4d::Identity();
std::vector<BoxSE> darknetRosMsgToBoxSE(std::vector<darknet_ros_msgs::BoundingBox>& boxes);

void GrabImage(const ImageConstPtr& image, const ImageConstPtr& depth, const darknet_ros_msgs::BoundingBoxesConstPtr& bbox);

void GrabIMU(const sensor_msgs::ImuConstPtr& imu_msg_ptr);

bool InitIMU();

int main(int argc, char **argv)
{
    WORK_SPACE_PATH = "/home/zhjd/ws_active/src/kinect/EAO-Fusion/";//ros::package::getPath("ros_evo") + "/../";

//(1) ROS
    ros::init(argc, argv, "EllipsoidSLAM");
    ros::NodeHandle nh;

//(1.1) subscribe
//tum_bag:     /camera/rgb/image_color    /camera/depth/image
//kinect_dk和rosbag_correct:  /rgb/image_raw   /depth_to_rgb/image_raw
    message_filters::Subscriber<Image> image_sub(nh, "/rgb/image_raw", 1);
    message_filters::Subscriber<Image> depth_sub(nh, "/depth_to_rgb/image_raw", 1);
    message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> bbox_sub(nh, "/darknet_ros/bounding_boxes", 1);
//    ros::Subscriber sub = nh.subscribe<darknet_ros_msgs::BoundingBoxes>( "/darknet_ros/bounding_boxes" , 10 , GrabBbox );
    typedef message_filters::sync_policies::ApproximateTime
            <Image, Image, darknet_ros_msgs::BoundingBoxes> sync_pol;
    typedef message_filters::sync_policies::ApproximateTime
            <Image, Image> sync_pol_only_image;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), image_sub, depth_sub, bbox_sub);
    message_filters::Synchronizer<sync_pol_only_image> sync_only_image(sync_pol_only_image (10), image_sub, depth_sub);
    
    ros::Subscriber sub_imu = nh.subscribe<sensor_msgs::Imu>( "/imu/data" , 100 , GrabIMU );

//(1.2) publisher
    pub_mam = nh.advertise<std_msgs::Float64>("/mam_angle", 10);


//(2)SYSTEM
    string yamlfile, sensor, method; bool semanticOnline, rosBagFlag;
    const std::string VocFile = WORK_SPACE_PATH + "/Vocabulary/ORBvoc.bin";
    // const std::string YamlFile = WORK_SPACE_PATH + "/ros_test/config/D435i.yaml";
    ros::param::param<std::string>("~yamlfile", yamlfile, "TUM3_ros.yaml"); /*kinectdk.yaml  TUM3.yaml TUM3_ros.yaml kinectdk_720.yaml*/
    ros::param::param<std::string>("~method", method, "Full");
    const std::string YamlFile = WORK_SPACE_PATH + "/ros_test/config/" + yamlfile;
    // 读取launch文件中的参数
    ros::param::param<std::string>("~sensor", sensor, "RGBD");
    
    ros::param::param<bool>("~rosbag", rosBagFlag, "false");  //  这是做什么的???/
    ros::param::param<bool>("~online", semanticOnline, "true");
//    slam_ptr_ = std::make_shared<ORB_SLAM2::System>(VocFile, YamlFile, "Full", ORB_SLAM2::System::RGBD, true, semanticOnline);
    std::cout<<"The method: "<<method <<std::endl ;
    system_ptr = new ORB_SLAM2::System(VocFile, YamlFile, method, ORB_SLAM2::System::RGBD, true, semanticOnline);


    sync.registerCallback(boost::bind(&GrabImage,_1,_2,_3));
    //    sync_only_image.registerCallback(boost::bind(&GrabImage,_1,_2));
    // 显示窗口
    // cv::namedWindow("rgb", cv::WINDOW_NORMAL);
    // cv::namedWindow("depth", cv::WINDOW_NORMAL);
    std::cout << "Waiting for comming frames..." << std::endl;






    
    ros::spin();
    ros::shutdown();
    // Save camera trajectory
    system_ptr->SaveKeyFrameTrajectoryTUM("/home/zhjd/ws_active/src/kinect/EAO-Fusion/eval/CameraTrajectory.txt");
    system_ptr->SavePlaneFeatures("/home/zhjd/ws_active/src/kinect/EAO-Fusion/eval/PlaneFeature.txt");
    system_ptr->SaveObjects("/home/zhjd/ws_active/src/kinect/EAO-Fusion/eval/Objects.txt",
                            "/home/zhjd/ws_active/src/kinect/EAO-Fusion/eval/Objects_with_points.txt");
    return 0;
}
std::vector<BoxSE> darknetRosMsgToBoxSE(std::vector<darknet_ros_msgs::BoundingBox>& boxes){
        //修改
        if (boxes.size() == 0)
        {
            std::cout << "[WARNNING] OBJECTS SIZE IS ZERO" << std::endl;
        }
        /* darknet_ros_msgs::BoundingBox
         * float64 probability
         * int64 xmin
         * int64 ymin
         * int64 xmax
         * int64 ymax
         * int16 id
         * string Class
         * */
        std::vector<BoxSE> boxes_online;
        for (auto &objInfo : boxes)
        {
            if (objInfo.probability < 0.5)
                continue;
            // TODO: 检测和yolo3ros 相同.
            // 0: person; 24: handbag?24应该是backpack背包,26是handbag手提包; 28: suitcase; 39: bottle; 56: chair;
            // 57: couch; 58:potted plant; 59: bed; 60: dining table; 62: tv;
            // 63: laptop; 66: keyboard; 67: phone; 73: book;
            if (objInfo.id != 0 && objInfo.id != 24 && objInfo.id != 28 && objInfo.id != 39 && objInfo.id != 56 && objInfo.id != 57 && objInfo.id != 58 && objInfo.id != 59 && objInfo.id != 60 && objInfo.id != 62 && objInfo.id != 63 && objInfo.id != 66 && objInfo.id != 67 && objInfo.id != 73
            /* 自己添加 */ && objInfo.id != 72 /* refrigerator */  && objInfo.id != 11 /* stop sign */
            )
                continue;
            BoxSE box;
            box.m_class = objInfo.id;
            box.m_score = objInfo.probability;
            box.x = objInfo.xmin;
            box.y = objInfo.ymin;
            box.width = (objInfo.xmax - objInfo.xmin);
            box.height = (objInfo.ymax - objInfo.ymin );
            // box.m_class_name = "";
            boxes_online.push_back(box);
        }
        std::sort(boxes_online.begin(), boxes_online.end(), [](BoxSE a, BoxSE b) -> bool { return a.m_score > b.m_score; });
        return boxes_online;
//    liaoziwei版本
//    Eigen::MatrixXd boxMat; boxMat.resize(boxes.size(), 7);
//    for( int i=0;i<boxMat.rows();i++ )
//    {
//        auto box = boxes[i];
//        Eigen::Matrix<double, 7, 1> boxVec;
//        boxVec << i,box.xmin,box.ymin,box.xmax,box.ymax,box.id,box.probability;
//        boxMat.row(i) = boxVec.transpose();
//    }
//    return boxMat;
    }

void GrabImage(const ImageConstPtr& msgImage, const ImageConstPtr& msgDepth, const darknet_ros_msgs::BoundingBoxes::ConstPtr & msgBbox)
{
    //  i++;
    //  std::cout << "OpenCV version: "
    //      << CV_MAJOR_VERSION << "."
    //      << CV_MINOR_VERSION << "."
    //      << CV_SUBMINOR_VERSION << "."
    //      << i
    //      << std::endl;

   cv::Mat imRGB, imDepth, imDepth32F;
    double current_time = msgImage->header.stamp.toSec();
    std::cout << std::endl << std::endl;
    std::cout << "[Get a Frame with bbox] Timestamp:" << current_time << std::endl;

    // // 数据处理 简易版
    //  cvImgPtr = cv_bridge::toCvCopy(current_image_color_data_, sensor_msgs::image_encodings::BGR8);
    // cvDepthPtr = cv_bridge::toCvCopy(current_image_depth_data_, sensor_msgs::image_encodings::TYPE_16UC1);
    // bboxes = current_image_bbox_data_.bounding_boxes;  //此时还是ros_vector版本,还需要转化为eigen mat版本


    // 彩色图像
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvCopy(msgImage);
        if(!cv_ptrRGB->image.empty())
            imRGB = cv_ptrRGB->image.clone();
        else
        {
            std::cout << "Empty RGB!" << std::endl;
            return;
        }
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // 深度图像
    cv_bridge::CvImageConstPtr cv_ptrDepth;
    try
    {
        cv_ptrDepth = cv_bridge::toCvCopy(msgDepth);
        if(!cv_ptrDepth->image.empty())
            imDepth = cv_ptrDepth->image.clone();
        else
        {
            std::cout << "Empty Depth!" << std::endl;
            return;
        }
        // imDepth32F.convertTo(imDepth, CV_16UC1, 1000);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    std::vector<darknet_ros_msgs::BoundingBox> boxes = msgBbox->bounding_boxes;
    if( boxes.size() == 0 )
    {
        std::cerr << "No detection. " << std::endl;
        // return;  //zhangjiadong  不能return,否则orbslam无法运行.
    }
    std::vector<BoxSE> BboxVector = darknetRosMsgToBoxSE(boxes);

    // mpSLAM->TrackWithObjects(timestamp , cam_pose, bboxMatFiltered, imDepth, imRGB, false);   // 其中 imRGB只用作可视化.
    system_ptr -> TrackRGBD(imRGB, imDepth, current_time, BboxVector);

    if( loop == 20){
        std_msgs::Float64 msg;
        msg.data =  system_ptr -> getMamGreadAngle();
        pub_mam.publish(msg);
        loop = 0;
    }else{
        loop ++;
    }
}

void GrabIMU(const sensor_msgs::ImuConstPtr& imu_msg_ptr) {

    // eigen四元数
    Eigen::Quaterniond q;
    q.x() = imu_msg_ptr->orientation.x;
    q.y() = imu_msg_ptr->orientation.y;
    q.z() = imu_msg_ptr->orientation.z;
    q.w() = imu_msg_ptr->orientation.w;

    // 转为eigen的旋转矩阵 --先归一化再转为旋转矩阵.  IMU输出的位姿orientation,是imu坐标系在世界坐标系下的姿态, 即:世界坐标系到imu坐标系的旋转变换关系..
    Eigen::Matrix3d R_world_to_imu = q.normalized().toRotationMatrix();

    // EDGUG:
    // //定义一个四元数quadf
    // tf::Quaternion quat;
    // //把msg形式的四元数转化为tf形式,得到quat的tf形式
    // tf::quaternionMsgToTF(imu_msg_ptr->orientation, quat);
    // //定义存储r\p\y的容器
    // double roll, pitch, yaw;
    // //进行转换得到RPY欧拉角
    // tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);        
    // double Pi = 3.1415926;
    // std::cout<<"roll="<<(roll*180/ Pi)<<"\t pitch="<<(pitch*180/ Pi)<<"\t yaw="<<(yaw*180/ Pi)<<std::endl;


    Eigen::Matrix3d R_camera_to_imu;
    R_camera_to_imu<< 0, -1, 0, 
                    0, 0, 1, 
                    -1, 0, 0;
    Eigen::Matrix3d R_imu_to_camera = R_camera_to_imu.inverse();
    // 旋转矩阵的右乘: 
    INIT_POSE.block<3, 3>(0, 0) =   R_world_to_imu * R_imu_to_camera;
}

