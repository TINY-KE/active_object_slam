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

#include "MapPublisher.h"
#include "MapPoint.h"
#include "KeyFrame.h"

namespace ORB_SLAM2
{


MapPublisher::MapPublisher(Map* pMap, const string &strSettingPath):mpMap(pMap), mbCameraUpdated(false)
{

    //Configure MapPoints
    fPointSize=0.01;
    mPoints.header.frame_id = MAP_FRAME_ID;
    mPoints.ns = POINTS_NAMESPACE;
    mPoints.id=0;
    mPoints.type = visualization_msgs::Marker::POINTS;
    mPoints.scale.x=fPointSize;
    mPoints.scale.y=fPointSize;
    mPoints.pose.orientation.w=1.0;
    mPoints.action=visualization_msgs::Marker::ADD;
    mPoints.color.a = 1.0;

    //Configure KeyFrames
    fCameraSize=0.04;
    mKeyFrames.header.frame_id = MAP_FRAME_ID;
    mKeyFrames.ns = KEYFRAMES_NAMESPACE;
    mKeyFrames.id=1;
    mKeyFrames.type = visualization_msgs::Marker::LINE_LIST;
    mKeyFrames.scale.x=0.005;
    mKeyFrames.pose.orientation.w=1.0;
    mKeyFrames.action=visualization_msgs::Marker::ADD;

    mKeyFrames.color.b=1.0f;
    mKeyFrames.color.a = 1.0;

    //Configure Covisibility Graph 共视图
    mCovisibilityGraph.header.frame_id = MAP_FRAME_ID;
    mCovisibilityGraph.ns = GRAPH_NAMESPACE;
    mCovisibilityGraph.id=2;
    mCovisibilityGraph.type = visualization_msgs::Marker::LINE_LIST;
    mCovisibilityGraph.scale.x=0.002;
    mCovisibilityGraph.pose.orientation.w=1.0;
    mCovisibilityGraph.action=visualization_msgs::Marker::ADD;
    mCovisibilityGraph.color.b=0.7f;
    mCovisibilityGraph.color.g=0.7f;
    mCovisibilityGraph.color.a = 0.3;

    //Configure KeyFrames Spanning Tree  关键帧中心的连线
    mMST.header.frame_id = MAP_FRAME_ID;
    mMST.ns = GRAPH_NAMESPACE;
    mMST.id=3;
    mMST.type = visualization_msgs::Marker::LINE_LIST;
    mMST.scale.x=0.005;
    mMST.pose.orientation.w=1.0;
    mMST.action=visualization_msgs::Marker::ADD;
    mMST.color.b=0.0f;
    mMST.color.g=1.0f;
    mMST.color.a = 1.0;

    //Configure Current Camera
    mCurrentCamera.header.frame_id = MAP_FRAME_ID;
    mCurrentCamera.ns = CAMERA_NAMESPACE;
    mCurrentCamera.id=4;
    mCurrentCamera.type = visualization_msgs::Marker::LINE_LIST;
    mCurrentCamera.scale.x=0.01;//0.2; 0.03
    mCurrentCamera.pose.orientation.w=1.0;
    mCurrentCamera.action=visualization_msgs::Marker::ADD;
    mCurrentCamera.color.g=1.0f;
    mCurrentCamera.color.a = 1.0;

    //Configure Reference MapPoints
    mReferencePoints.header.frame_id = MAP_FRAME_ID;
    mReferencePoints.ns = POINTS_NAMESPACE;
    mReferencePoints.id=6;
    mReferencePoints.type = visualization_msgs::Marker::POINTS;
    mReferencePoints.scale.x=fPointSize;
    mReferencePoints.scale.y=fPointSize;
    mReferencePoints.pose.orientation.w=1.0;
    mReferencePoints.action=visualization_msgs::Marker::ADD;
    mReferencePoints.color.r =1.0f;
    mReferencePoints.color.a = 1.0;

    //Configure IE text
    mIEtext.header.frame_id = "map";
    mIEtext.header.stamp = ros::Time::now();
    mIEtext.ns = "text_marker";
    mIEtext.id = 7;
    mIEtext.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    mIEtext.action = visualization_msgs::Marker::ADD;
    mIEtext.pose.position.x = -2.0;
    mIEtext.pose.position.y = 0.0;
    mIEtext.pose.position.z = 2.0;
    mIEtext.pose.orientation.x = 0.0;
    mIEtext.pose.orientation.y = 0.0;
    mIEtext.pose.orientation.z = 0.0;
    mIEtext.pose.orientation.w = 1.0;
    mIEtext.scale.x = 1.0;
    mIEtext.scale.y = 1.0;
    mIEtext.scale.z = 1.0;
    mIEtext.color.r = 1.0;
    mIEtext.color.g = 0.0;
    mIEtext.color.b = 0.0;
    mIEtext.color.a = 1.0;
    mIEtext.text = "Hello, world!";

    //Configure IE grid
    IE_id = 8;

    //Configure MapObjects
    object_id_init = 46;


    //Configure Publisher
    publisher = nh.advertise<visualization_msgs::Marker>("Point", 1000);
    publisher_curframe = nh.advertise<visualization_msgs::Marker>("CurFrame", 1000);
    publisher_KF = nh.advertise<visualization_msgs::Marker>("KeyFrame", 1000);
    publisher_CoView = nh.advertise<visualization_msgs::Marker>("CoView", 1000);
    publisher_object = nh.advertise<visualization_msgs::Marker>("objectmap", 1000);
    publisher_object_points = nh.advertise<visualization_msgs::Marker>("objectPoints", 1000);
    publisher_IE_maindirection = nh.advertise<visualization_msgs::Marker>("object_ie_maindirection", 1000);
    publisher_IE_cylinder = nh.advertise<visualization_msgs::Marker>("object_ie_cylinder", 1000);
    publisher_IE_ellipse = nh.advertise<visualization_msgs::Marker>("object_ie_ellipse", 1000);
    publisher_IE_half = nh.advertise<visualization_msgs::Marker>("object_ie_half", 1000);
    publisher_MainDirection = nh.advertise<visualization_msgs::Marker>("object_MainDirection", 1000);
    publisher_SumMainDirection = nh.advertise<visualization_msgs::Marker>("object_SumMainDirection", 1000);
    //publisher_robotpose = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1000);
    //publisher_mam_rviz = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/local_nbv", 1000);
    publisher_IEtext = nh.advertise<visualization_msgs::Marker>("IEtext", 1);

    publisher.publish(mPoints);
    publisher.publish(mReferencePoints);
    publisher_CoView.publish(mCovisibilityGraph);
    publisher_KF.publish(mKeyFrames);
    publisher_curframe.publish(mCurrentCamera);

    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    //坐标关系,用于生成
    float qx = fSettings["Trobot_camera.qx"], qy = fSettings["Trobot_camera.qy"], qz = fSettings["Trobot_camera.qz"], qw = fSettings["Trobot_camera.qw"],
                tx = fSettings["Trobot_camera.tx"], ty = fSettings["Trobot_camera.ty"], tz = fSettings["Trobot_camera.tz"];
    mT_body_cam = Converter::Quation2CvMat(qx, qy, qz, qw, tx, ty, tz );
    mQuaternion_robot_camera.x = qx;
    mQuaternion_robot_camera.y = qy;
    mQuaternion_robot_camera.z = qz;
    mQuaternion_robot_camera.w = qw;
    mTranslation_robot_camera.x = tx;
    mTranslation_robot_camera.y = ty;
    mTranslation_robot_camera.z = tz;

    //ros duration
    mObject_Duration = fSettings["Viewer.object_Duration"];
    mDirection_Duration = fSettings["Viewer.direction_Duration"];
    mIE_Duration = fSettings["Viewer.IE_Duration"];
}

void MapPublisher::Refresh()
{
    {
        PublishCurrentCamera(mCameraPose);
    }
    {
        vector<KeyFrame*> vKeyFrames = mpMap->GetAllKeyFrames();
        vector<MapPoint*> vMapPoints = mpMap->GetAllMapPoints();
        vector<MapPoint*> vRefMapPoints = mpMap->GetReferenceMapPoints();
        //vector<MapPlane*> vMapPlanes = mpMap ->GetAllMapPlanes();
        vector<Object_Map*> vMapObjects = mpMap -> GetObjects();

        PublishMapPoints(vMapPoints, vRefMapPoints);   
        PublishKeyFrames(vKeyFrames);
        //PublishPlane(vMapPlanes);
        PublishObject(vMapObjects);
        //PublishIE(vMapObjects);

    }
}

void MapPublisher::PublishMapPoints(const vector<MapPoint*> &vpMPs, const vector<MapPoint*> &vpRefMPs)
{
    mPoints.points.clear();
    mReferencePoints.points.clear();

    set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
    {
        if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
            continue;
        geometry_msgs::Point p;
        cv::Mat pos = vpMPs[i]->GetWorldPos();
        p.x=pos.at<float>(0);
        p.y=pos.at<float>(1);
        p.z=pos.at<float>(2);

        mPoints.points.push_back(p);
    }

    for(set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
    {
        if((*sit)->isBad())
            continue;
        geometry_msgs::Point p;
        cv::Mat pos = (*sit)->GetWorldPos();
        p.x=pos.at<float>(0);
        p.y=pos.at<float>(1);
        p.z=pos.at<float>(2);

        mReferencePoints.points.push_back(p);
    }

    mPoints.header.stamp = ros::Time::now();
    mReferencePoints.header.stamp = ros::Time::now();
    publisher.publish(mPoints);
    publisher.publish(mReferencePoints);
}

void MapPublisher::PublishKeyFrames(const vector<KeyFrame*> &vpKFs)
{
    mKeyFrames.points.clear();
    mCovisibilityGraph.points.clear();
    mMST.points.clear();

    float d = fCameraSize;

    //Camera is a pyramid. Define in camera coordinate system
    cv::Mat o = (cv::Mat_<float>(4,1) << 0, 0, 0, 1);
    cv::Mat p1 = (cv::Mat_<float>(4,1) << d, d*0.8, d*0.5, 1);
    cv::Mat p2 = (cv::Mat_<float>(4,1) << d, -d*0.8, d*0.5, 1);
    cv::Mat p3 = (cv::Mat_<float>(4,1) << -d, -d*0.8, d*0.5, 1);
    cv::Mat p4 = (cv::Mat_<float>(4,1) << -d, d*0.8, d*0.5, 1);

    for(size_t i=0, iend=vpKFs.size() ;i<iend; i++)
    {
        cv::Mat Tcw = vpKFs[i]->GetPose();
//        std::cout<<"[rviz debug] frame id： "<< vpKFs[i]->mnId <<", Tcw:"<<Tcw<< std::endl;
        cv::Mat Twc = Tcw.inv();
        cv::Mat ow = vpKFs[i]->GetCameraCenter();
        cv::Mat p1w = Twc*p1;
        cv::Mat p2w = Twc*p2;
        cv::Mat p3w = Twc*p3;
        cv::Mat p4w = Twc*p4;

        geometry_msgs::Point msgs_o,msgs_p1, msgs_p2, msgs_p3, msgs_p4;
        msgs_o.x=ow.at<float>(0);
        msgs_o.y=ow.at<float>(1);
        msgs_o.z=ow.at<float>(2);
        msgs_p1.x=p1w.at<float>(0);
        msgs_p1.y=p1w.at<float>(1);
        msgs_p1.z=p1w.at<float>(2);
        msgs_p2.x=p2w.at<float>(0);
        msgs_p2.y=p2w.at<float>(1);
        msgs_p2.z=p2w.at<float>(2);
        msgs_p3.x=p3w.at<float>(0);
        msgs_p3.y=p3w.at<float>(1);
        msgs_p3.z=p3w.at<float>(2);
        msgs_p4.x=p4w.at<float>(0);
        msgs_p4.y=p4w.at<float>(1);
        msgs_p4.z=p4w.at<float>(2);

        mKeyFrames.points.push_back(msgs_o);
        mKeyFrames.points.push_back(msgs_p1);
        mKeyFrames.points.push_back(msgs_o);
        mKeyFrames.points.push_back(msgs_p2);
        mKeyFrames.points.push_back(msgs_o);
        mKeyFrames.points.push_back(msgs_p3);
        mKeyFrames.points.push_back(msgs_o);
        mKeyFrames.points.push_back(msgs_p4);
        mKeyFrames.points.push_back(msgs_p1);
        mKeyFrames.points.push_back(msgs_p2);
        mKeyFrames.points.push_back(msgs_p2);
        mKeyFrames.points.push_back(msgs_p3);
        mKeyFrames.points.push_back(msgs_p3);
        mKeyFrames.points.push_back(msgs_p4);
        mKeyFrames.points.push_back(msgs_p4);
        mKeyFrames.points.push_back(msgs_p1);

        // Covisibility Graph
        vector<KeyFrame*> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
        if(!vCovKFs.empty())
        {
            for(vector<KeyFrame*>::iterator vit=vCovKFs.begin(), vend=vCovKFs.end(); vit!=vend; vit++)
            {
                if((*vit)->mnId<vpKFs[i]->mnId)
                    continue;
                cv::Mat Ow2 = (*vit)->GetCameraCenter();
                geometry_msgs::Point msgs_o2;
                msgs_o2.x=Ow2.at<float>(0);
                msgs_o2.y=Ow2.at<float>(1);
                msgs_o2.z=Ow2.at<float>(2);
                mCovisibilityGraph.points.push_back(msgs_o);
                mCovisibilityGraph.points.push_back(msgs_o2);
            }
        }

        // MST
        KeyFrame* pParent = vpKFs[i]->GetParent();
        if(pParent)
        {
            cv::Mat Owp = pParent->GetCameraCenter();
            geometry_msgs::Point msgs_op;
            msgs_op.x=Owp.at<float>(0);
            msgs_op.y=Owp.at<float>(1);
            msgs_op.z=Owp.at<float>(2);
            mMST.points.push_back(msgs_o);
            mMST.points.push_back(msgs_op);
        }
        set<KeyFrame*> sLoopKFs = vpKFs[i]->GetLoopEdges();
        for(set<KeyFrame*>::iterator sit=sLoopKFs.begin(), send=sLoopKFs.end(); sit!=send; sit++)
        {
            if((*sit)->mnId<vpKFs[i]->mnId)
                continue;
            cv::Mat Owl = (*sit)->GetCameraCenter();
            geometry_msgs::Point msgs_ol;
            msgs_ol.x=Owl.at<float>(0);
            msgs_ol.y=Owl.at<float>(1);
            msgs_ol.z=Owl.at<float>(2);
            mMST.points.push_back(msgs_o);
            mMST.points.push_back(msgs_ol);
        }
    }

    mKeyFrames.header.stamp = ros::Time::now();
    mCovisibilityGraph.header.stamp = ros::Time::now();
    mMST.header.stamp = ros::Time::now();

    publisher_KF.publish(mKeyFrames);
    publisher_CoView.publish(mCovisibilityGraph);
    publisher_KF.publish(mMST);
}

void MapPublisher::PublishCurrentCamera(const cv::Mat &Tcw) {
    if (Tcw.empty())
        return;

    mCurrentCamera.points.clear();

    float d = fCameraSize;
    //(1) 相机的几何模型
    //Camera is a pyramid. Define in camera coordinate system
    cv::Mat o = (cv::Mat_<float>(4, 1) << 0, 0, 0, 1);
    cv::Mat p1 = (cv::Mat_<float>(4, 1) << d, d * 0.8, d * 0.5, 1);
    cv::Mat p2 = (cv::Mat_<float>(4, 1) << d, -d * 0.8, d * 0.5, 1);
    cv::Mat p3 = (cv::Mat_<float>(4, 1) << -d, -d * 0.8, d * 0.5, 1);
    cv::Mat p4 = (cv::Mat_<float>(4, 1) << -d, d * 0.8, d * 0.5, 1);

    cv::Mat Twc = Tcw.inv();
    cv::Mat ow = Twc * o;
    cv::Mat p1w = Twc * p1;
    cv::Mat p2w = Twc * p2;
    cv::Mat p3w = Twc * p3;
    cv::Mat p4w = Twc * p4;

    geometry_msgs::Point msgs_o, msgs_p1, msgs_p2, msgs_p3, msgs_p4;
    msgs_o.x = ow.at<float>(0);
    msgs_o.y = ow.at<float>(1);
    msgs_o.z = ow.at<float>(2);
    msgs_p1.x = p1w.at<float>(0);
    msgs_p1.y = p1w.at<float>(1);
    msgs_p1.z = p1w.at<float>(2);
    msgs_p2.x = p2w.at<float>(0);
    msgs_p2.y = p2w.at<float>(1);
    msgs_p2.z = p2w.at<float>(2);
    msgs_p3.x = p3w.at<float>(0);
    msgs_p3.y = p3w.at<float>(1);
    msgs_p3.z = p3w.at<float>(2);
    msgs_p4.x = p4w.at<float>(0);
    msgs_p4.y = p4w.at<float>(1);
    msgs_p4.z = p4w.at<float>(2);

    mCurrentCamera.points.push_back(msgs_o);
    mCurrentCamera.points.push_back(msgs_p1);
    mCurrentCamera.points.push_back(msgs_o);
    mCurrentCamera.points.push_back(msgs_p2);
    mCurrentCamera.points.push_back(msgs_o);
    mCurrentCamera.points.push_back(msgs_p3);
    mCurrentCamera.points.push_back(msgs_o);
    mCurrentCamera.points.push_back(msgs_p4);
    mCurrentCamera.points.push_back(msgs_p1);
    mCurrentCamera.points.push_back(msgs_p2);
    mCurrentCamera.points.push_back(msgs_p2);
    mCurrentCamera.points.push_back(msgs_p3);
    mCurrentCamera.points.push_back(msgs_p3);
    mCurrentCamera.points.push_back(msgs_p4);
    mCurrentCamera.points.push_back(msgs_p4);
    mCurrentCamera.points.push_back(msgs_p1);

    mCurrentCamera.header.stamp = ros::Time::now();

    publisher_curframe.publish(mCurrentCamera);

    //（2）相机的坐标和四元数
    //cv::Mat T_w_cam = Tcw.inv();   //初始的相机在世界的坐标
    //cv::Mat T_w_body = cv::Mat::eye(4, 4, CV_32F);
    //T_w_body = T_w_cam * mT_body_cam.inv();  //初始的机器人底盘在世界的坐标
    //geometry_msgs::PoseWithCovarianceStamped robotpose;
    //robotpose.pose.pose.position.x = T_w_body.at<float>(0, 3);
    //robotpose.pose.pose.position.y = T_w_body.at<float>(1, 3);
    //robotpose.pose.pose.position.z = 0.0;
    ////（2.1）gazebo中的四元数和rviz的不同，需要绕着z轴转90度
    ////Eigen::AngleAxisd rotation_vector (-M_PI/2.0, Eigen::Vector3d(0,0,1));
    ////Eigen::Quaterniond q_y_x = Eigen::Quaterniond(rotation_vector);
    ////Eigen::Quaterniond q_w_body = Converter::toQuaterniond(T_w_body);
    ////Eigen::Quaterniond q = q_y_x * q_w_body;
    ////（2.2）不饶z轴转90度
    //Eigen::Quaterniond q_w_body = Converter::ExtractQuaterniond(T_w_body);
    //Eigen::Quaterniond q = q_w_body;
    //
    //robotpose.pose.pose.orientation.w = q.w();
    //robotpose.pose.pose.orientation.x = q.x();
    //robotpose.pose.pose.orientation.y = q.y();
    //robotpose.pose.pose.orientation.z = q.z();
    //robotpose.header.frame_id= "odom";
    //robotpose.header.stamp=ros::Time::now();
    //publisher_robotpose.publish(robotpose);

    //（3）发布tf树
    //发布机器人底盘和odom的tf变换
    //geometry_msgs::TransformStamped odom_trans;
    //ros::Time current_time = ros::Time::now();
    //odom_trans.header.stamp = current_time;
    //odom_trans.header.frame_id = "odom";
    //odom_trans.child_frame_id = "base_link";
    //odom_trans.transform.translation.x = robotpose.pose.pose.position.x;
    //odom_trans.transform.translation.y = robotpose.pose.pose.position.y;
    //odom_trans.transform.translation.z = 0.0;
    //odom_trans.transform.rotation = robotpose.pose.pose.orientation; //Quaternion_odom_robot;
    //odom_broadcaster.sendTransform(odom_trans);

    //发布camera_depth_optical_frame和base_link的tf变换
    //geometry_msgs::TransformStamped camera_trans;
    //camera_trans.header.stamp = current_time;
    //camera_trans.header.frame_id = "base_link";
    //camera_trans.child_frame_id = "camera_depth_optical_frame";
    //camera_trans.transform.translation.x = mTranslation_robot_camera.x;
    //camera_trans.transform.translation.y = mTranslation_robot_camera.y;
    //camera_trans.transform.translation.z = mTranslation_robot_camera.z;
    //camera_trans.transform.rotation = mQuaternion_robot_camera;
    //camera_broadcaster.sendTransform(camera_trans);

    //(4)发布localnbv的箭头   //NBV MAM
    //if(mbMAMUpdated){
    //    geometry_msgs::PoseWithCovarianceStamped mampose;
    //    mampose.pose.pose.position.x = T_w_body.at<float>(0, 3);
    //    mampose.pose.pose.position.y = T_w_body.at<float>(1, 3);
    //    mampose.pose.pose.position.z = 0.0;
    //
    //    Eigen::Quaterniond q_w_body = Converter::ExtractQuaterniond(T_w_body);
    //    Eigen::Quaterniond q_body_rotate = Eigen::Quaterniond( Eigen::AngleAxisd( mMAM_angle*M_PI/180.0, Eigen::Vector3d ( 0,0,1 ) )  );     //沿 Z 轴旋转 45 度
    //    Eigen::Quaterniond q = q_w_body * q_body_rotate;
    //    mampose.pose.pose.orientation.w = q.w();
    //    mampose.pose.pose.orientation.x = q.x();
    //    mampose.pose.pose.orientation.y = q.y();
    //    mampose.pose.pose.orientation.z = q.z();
    //    mampose.header.frame_id= "map";
    //    mampose.header.stamp=ros::Time::now();
    //
    //    publisher_mam_rviz.publish(mampose);
    //}
}

//void MapPublisher::PublishPlane(const vector<MapPlane *> &vpMPls ){
//    mPlanes.points.clear();
//
//    for(size_t i=0, iend=vpMPls.size(); i<iend;i++)
//    {
//        if(vpMPls[i]->isBad() )
//            continue;
//        geometry_msgs::Point p;
//        cv::Mat pos = vpMPls[i]->GetWorldPos();
//        p.x=pos.at<float>(0);
//        p.y=pos.at<float>(1);
//        p.z=pos.at<float>(2);
//
//        mPoints.points.push_back(p);
//    }
//
//
//    mPoints.header.stamp = ros::Time::now();
//    mReferencePoints.header.stamp = ros::Time::now();
//    //publisher.publish(mPlanes);
//}

geometry_msgs::Point MapPublisher::corner_to_marker(Eigen::Vector3d& v){
    geometry_msgs::Point point;
    point.x = v[0];
    point.y = v[1];
    point.z = v[2];
    return point;
}

geometry_msgs::Point MapPublisher::corner_to_marker(const std::vector<float>& v){
    geometry_msgs::Point point;
    point.x = v[0];
    point.y = v[1];
    point.z = v[2];
    return point;
}

geometry_msgs::Point MapPublisher::corner_to_marker(const std::vector<double>& v){
    geometry_msgs::Point point;
    point.x = v[0];
    point.y = v[1];
    point.z = v[2];
    return point;
}

void MapPublisher::PublishObject(const vector<Object_Map*> &vObjs ){


    for(size_t i=0; i< vObjs.size(); i++)
    {

        // color.
        std::vector<vector<float> > colors_bgr{ {135,0,248},  {255,0,253},  {4,254,119},  {255,126,1},  {0,112,255},  {0,250,250}   };
        vector<float> color = colors_bgr[ (vObjs[i]->mnClass +4) % 6];  //+1还是粉色/红色    +2绿色  +3蓝色  +4橙色   +5黄色


        //用于物体的颜色随机
        std::default_random_engine e;
        std::uniform_real_distribution<double>  random(0.5,1);
        float r = random(e); float g = random(e); float b = random(e);

        //(1)物体
        visualization_msgs::Marker marker;
        marker.id = vObjs[i]->mnId;//++object_id_init;//object_id_init + i;
        marker.lifetime = ros::Duration(mObject_Duration);
        marker.header.frame_id= MAP_FRAME_ID;
        marker.header.stamp=ros::Time::now();

        //if(!vObjs[i]->backgroud_object)
        {
            if(/*(vObjs[i]->mvpMapObjectMappoints.size() < 10) ||*/ (vObjs[i]->bad_3d == true) || vObjs[i]->backgroud_object  )
            {
                continue;
            }
        }

        marker.type = visualization_msgs::Marker::LINE_LIST; //LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.color.r = color[2]/255.0; marker.color.g = color[1]/255.0; marker.color.b = color[0]/255.0; marker.color.a = 1.0;
        marker.scale.x = 0.01;
        //     8------7
        //    /|     /|
        //   / |    / |
        //  5------6  |
        //  |  4---|--3
        //  | /    | /
        //  1------2
        // lenth ：corner_2[0] - corner_1[0]
        // width ：corner_2[1] - corner_3[1]
        // height：corner_2[2] - corner_6[2]

        marker.points.push_back(corner_to_marker(vObjs[i]->mCuboid3D.corner_1));
        marker.points.push_back(corner_to_marker(vObjs[i]->mCuboid3D.corner_2));
        marker.points.push_back(corner_to_marker(vObjs[i]->mCuboid3D.corner_2));
        marker.points.push_back(corner_to_marker(vObjs[i]->mCuboid3D.corner_3));
        marker.points.push_back(corner_to_marker(vObjs[i]->mCuboid3D.corner_3));
        marker.points.push_back(corner_to_marker(vObjs[i]->mCuboid3D.corner_4));
        marker.points.push_back(corner_to_marker(vObjs[i]->mCuboid3D.corner_4));
        marker.points.push_back(corner_to_marker(vObjs[i]->mCuboid3D.corner_1));

        marker.points.push_back(corner_to_marker(vObjs[i]->mCuboid3D.corner_5));
        marker.points.push_back(corner_to_marker(vObjs[i]->mCuboid3D.corner_1));
        marker.points.push_back(corner_to_marker(vObjs[i]->mCuboid3D.corner_6));
        marker.points.push_back(corner_to_marker(vObjs[i]->mCuboid3D.corner_2));
        marker.points.push_back(corner_to_marker(vObjs[i]->mCuboid3D.corner_7));
        marker.points.push_back(corner_to_marker(vObjs[i]->mCuboid3D.corner_3));
        marker.points.push_back(corner_to_marker(vObjs[i]->mCuboid3D.corner_8));
        marker.points.push_back(corner_to_marker(vObjs[i]->mCuboid3D.corner_4));

        marker.points.push_back(corner_to_marker(vObjs[i]->mCuboid3D.corner_5));
        marker.points.push_back(corner_to_marker(vObjs[i]->mCuboid3D.corner_6));
        marker.points.push_back(corner_to_marker(vObjs[i]->mCuboid3D.corner_6));
        marker.points.push_back(corner_to_marker(vObjs[i]->mCuboid3D.corner_7));
        marker.points.push_back(corner_to_marker(vObjs[i]->mCuboid3D.corner_7));
        marker.points.push_back(corner_to_marker(vObjs[i]->mCuboid3D.corner_8));
        marker.points.push_back(corner_to_marker(vObjs[i]->mCuboid3D.corner_8));
        marker.points.push_back(corner_to_marker(vObjs[i]->mCuboid3D.corner_5));

        publisher_object.publish(marker);

        //(2)物体中的点
        visualization_msgs::Marker marker1;
        marker1.header.frame_id = MAP_FRAME_ID;
        marker1.ns = "ObjectPoints";
        marker1.lifetime = ros::Duration(mObject_Duration);
        marker1.id= ++object_id_init;
        marker1.type = visualization_msgs::Marker::POINTS;
        marker1.scale.x=0.01;
        marker1.scale.y=0.01;
        marker1.pose.orientation.w=1.0;  //????
        marker1.action=visualization_msgs::Marker::ADD;
        marker1.color.r = color[2]/255.0; marker1.color.g = color[1]/255.0; marker1.color.b = color[0]/255.0; marker1.color.a = 0.5;
        vector<MapPoint* > vpMPs = vObjs[i]->GetObjectMappoints();
        for(size_t i=0, iend=vpMPs.size();  i<iend;  i++)
        {
            //if(vpMPs[i]->isBad() )
            //    continue;
            geometry_msgs::Point p;
            cv::Mat pos = vpMPs[i]->GetWorldPos();
            //std::cout<<"[nbv debug:1 ]"<<pos <<std::endl;
            p.x=pos.at<float>(0);
            p.y=pos.at<float>(1);
            p.z=pos.at<float>(2);
            //std::cout<<"[nbv debug:2 ]"<<p.x<<" "<<p.y<<" "<<p.z <<std::endl;
            marker1.points.push_back(p);
        }
        publisher_object_points.publish(marker1);

        //(3)物体中新的点
        vector<float> color_new = colors_bgr[(vObjs[i]->mnClass+1) % 6];
        visualization_msgs::Marker marker2;
        marker2.header.frame_id = MAP_FRAME_ID;
        marker2.ns = "NewObjectPoints";
        marker2.lifetime = ros::Duration(mObject_Duration);
        marker2.id= ++object_id_init;
        marker2.type = visualization_msgs::Marker::POINTS;
        marker2.scale.x=0.02;
        marker2.scale.y=0.02;
        marker2.pose.orientation.w=1.0;  //????
        marker2.action=visualization_msgs::Marker::ADD;
        //marker2.color.r = color_new[2]/255.0; marker2.color.g = color_new[1]/255.0; marker2.color.b = color_new[0]/255.0; marker2.color.a = 1.0;
        marker2.color.r = 0.0; marker2.color.g = 0.0; marker2.color.b = 1.0;
        marker2.color.a = 1.0;
        vector<MapPoint* > vpMPs_new = vObjs[i]->GetNewObjectMappoints();
        for(size_t i=0, iend=vpMPs_new.size();  i<iend;  i++)
        {
            //if(vpMPs[i]->isBad() )
            //    continue;
            geometry_msgs::Point p;
            cv::Mat pos = vpMPs_new[i]->GetWorldPos();
            //std::cout<<"[nbv debug:1 ]"<<pos <<std::endl;
            p.x=pos.at<float>(0);
            p.y=pos.at<float>(1);
            p.z=pos.at<float>(2);
            //std::cout<<"[nbv debug:2 ]"<<p.x<<" "<<p.y<<" "<<p.z <<std::endl;
            marker2.points.push_back(p);
        }
        //std::cout<<"[rviz debug:2 ]"<<marker2.points.size() <<std::endl;
        publisher_object_points.publish(marker2);

        //(4)main direction
        visualization_msgs::Marker marker3;
        marker3.header.frame_id = MAP_FRAME_ID;
        marker3.header.stamp = ros::Time::now();
        marker3.ns = "vector";
        marker3.lifetime = ros::Duration(mDirection_Duration);
        marker3.id = marker.id;
        marker3.type = visualization_msgs::Marker::ARROW;
        marker3.scale.x = 0.02;
        marker3.scale.y = 0.02;
        marker3.scale.z = 0.05;
        marker3.pose.orientation.w = 1.0;
        marker3.points.resize(2);
        marker3.points[0].x = vObjs[i]->mCuboid3D.cuboidCenter.x();
        marker3.points[0].y = vObjs[i]->mCuboid3D.cuboidCenter.y();
        marker3.points[0].z = vObjs[i]->mCuboid3D.cuboidCenter.z();
        marker3.points[1].x = vObjs[i]->mMainDirection.x()/10.0*2.0 + vObjs[i]->mCuboid3D.cuboidCenter.x();
        marker3.points[1].y = vObjs[i]->mMainDirection.y()/10.0*2.0 + vObjs[i]->mCuboid3D.cuboidCenter.y();
        marker3.points[1].z = vObjs[i]->mMainDirection.z()/10.0*2.0 + vObjs[i]->mCuboid3D.cuboidCenter.z();
        marker3.action = visualization_msgs::Marker::ADD;
        marker3.color.r = 1.0;
        marker3.color.g = 1.0;
        marker3.color.b = 0.0;
        marker3.color.a = 1.0;
        publisher_MainDirection.publish(marker3);

        //(5)IE text
        mIEtext.lifetime = ros::Duration(mDirection_Duration);
        mIEtext.id = vObjs[i]->mnId;
        mIEtext.pose.position.x = vObjs[i]->mCuboid3D.cuboidCenter.x();
        mIEtext.pose.position.y = vObjs[i]->mCuboid3D.cuboidCenter.y();
        mIEtext.pose.position.z = 2.0;
        mIEtext.scale.x = 0.2;
        mIEtext.scale.y = 0.2;
        mIEtext.scale.z = 0.2;
        mIEtext.color.r = color[2]/255.0;
        mIEtext.color.g = color[1]/255.0;
        mIEtext.color.b = color[0]/255.0;
        mIEtext.color.a = 1.0;
        //ROS_ERROR("IEtext:%f",vObjs[i]->mIE );
        //ROS_ERROR_STREAM(to_string(double(vObjs[i]->mIE)));
        mIEtext.text = to_string(double(vObjs[i]->mIE) );
        //std::cout<<"PointNum_mat:"<<vObjs[i]->mvPointNum_mat<<std::endl;
        //std::cout<<"GridProb_mat:"<<vObjs[i]->mvGridProb_mat<<std::endl;
        //std::cout<<"InforEntroy_mat:"<<vObjs[i]->mvInforEntroy_mat<<std::endl;
        publisher_IEtext.publish(mIEtext);
    }

    //5.全部的观测主方向
    int object_viewed_num = 0;
    Eigen::Vector3d direction_all = Eigen::Vector3d::Zero();
    Eigen::Vector3d object_viewed_center_all = Eigen::Vector3d::Zero();
    for (int i = 0; i < (int)vObjs.size(); i++) {
        Object_Map *obj3d = vObjs[i];
            direction_all += obj3d->mMainDirection;
    }
    direction_all = direction_all.normalized()/10.0*3.0;
    visualization_msgs::Marker marker;
    marker.header.frame_id = MAP_FRAME_ID;
    marker.header.stamp = ros::Time::now();
    marker.ns = "vector";
    marker.lifetime = ros::Duration(mDirection_Duration);
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.scale.x = 0.02;
    marker.scale.y = 0.02;
    marker.scale.z = 0.05;
    marker.pose.orientation.w = 1.0;
    marker.points.resize(2);
    marker.points[0].x = -2.0;
    marker.points[0].y = 0.0;
    marker.points[0].z = 0.0;
    marker.points[1].x = direction_all.x() - 2.0;
    marker.points[1].y = direction_all.y() + 0.0;
    marker.points[1].z = direction_all.z() + 0.0;
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    publisher_SumMainDirection.publish(marker);

}

void MapPublisher::PublishIE(const vector<Object_Map*> &vObjs ){
    // color.
    std::vector<vector<float> > colors_bgr{ {135,0,248},  {255,0,253},  {4,254,119},  {255,126,1},  {0,112,255},  {0,250,250}   };
    vector<float> color;

    //生成 rviz marker _cylinder
    visualization_msgs::Marker marker_cylinder;
    marker_cylinder.header.frame_id = MAP_FRAME_ID;
    marker_cylinder.ns = "InformationEntroy";
    //marker.lifetime = ros::Duration(5.0);
    marker_cylinder.lifetime = ros::Duration(mIE_Duration);
    marker_cylinder.id= IE_id ;  //TODO:绝对数字
    marker_cylinder.type = visualization_msgs::Marker::POINTS;
    marker_cylinder.scale.x=0.03;
    marker_cylinder.scale.y=0.1;
    marker_cylinder.pose.orientation.w=1.0;  //????
    marker_cylinder.action=visualization_msgs::Marker::ADD;

    //生成 rviz marker _ellipse
    visualization_msgs::Marker marker_ellipse;
    marker_ellipse.header.frame_id = MAP_FRAME_ID;
    marker_ellipse.ns = "InformationEntroy";
    //marker.lifetime = ros::Duration(5.0);
    marker_ellipse.lifetime = ros::Duration(mIE_Duration);
    marker_ellipse.id= IE_id ;  //TODO:绝对数字
    marker_ellipse.type = visualization_msgs::Marker::POINTS;
    marker_ellipse.scale.x=0.03;
    marker_ellipse.scale.y=0.1;
    marker_ellipse.pose.orientation.w=1.0;  //????
    marker_ellipse.action=visualization_msgs::Marker::ADD;

    //生成 rviz marker _half
    visualization_msgs::Marker marker_half;
    marker_half.header.frame_id = MAP_FRAME_ID;
    marker_half.ns = "InformationEntroy";
    //marker.lifetime = ros::Duration(5.0);
    marker_half.lifetime = ros::Duration(mIE_Duration);
    marker_half.id= IE_id ;  //TODO:绝对数字
    marker_half.type = visualization_msgs::Marker::POINTS;
    marker_half.scale.x=0.03;
    marker_half.scale.y=0.1;
    marker_half.pose.orientation.w=1.0;  //????
    marker_half.action=visualization_msgs::Marker::ADD;

    for(size_t i=0; i< vObjs.size(); i++){
        Object_Map* obj = vObjs[i];

        if((obj->mvpMapObjectMappoints.size() < 10) || (obj->bad_3d == true)  || obj->backgroud_object )
        {
            continue;
        }

        color = colors_bgr[(obj->mnClass) % 6];  //+1还是粉色/红色    +2绿色  +3蓝色  +4橙色   +5黄色
        double diameter_init = sqrt(obj->mCuboid3D.width * obj->mCuboid3D.width   +   obj->mCuboid3D.lenth * obj->mCuboid3D.lenth )/2.0;
        double a_aix_ellipse = diameter_init * 1.2;  //水平方向
        double b_aix_ellipse = obj->mCuboid3D.height / 2.0 * 1.2;   //垂直方向
        double a_aix_half = diameter_init * 1.2;  //水平方向
        double b_aix_half = obj->mCuboid3D.height ;   //垂直方向
        for(int x=0; x<obj->mIE_rows; x++){

            for(int y=0; y<obj->mIE_cols; y++){

                //version:  圆柱
                double h_divide =  obj->mCuboid3D.height/obj->mIE_rows;
                //z
                double p_z_cylinder = h_divide * (y+0.5) - obj->mCuboid3D.height/2.0;  //纵坐标
                //xy
                double angle_divide_xy = 2 * M_PI / obj->mIE_cols;
                double angle = angle_divide_xy * (x + 0.5 );
                double p_x_cylinder = cos(angle) * diameter_init;
                double p_y_cylinder = sin(angle) * diameter_init;
                // 物体坐标系 -> 世界坐标系
                cv::Mat cvMat4 = obj->mCuboid3D.pose_mat.clone();
                Eigen::Matrix4f eigenMat4f;
                cv::cv2eigen(cvMat4, eigenMat4f);
                //Eigen::Matrix4d T = ORB_SLAM2::Converter::cvMattoMatrix4d(obj->mCuboid3D.pose_mat);
                Eigen::Matrix4d T = eigenMat4f.cast<double>();
                Eigen::Matrix3d R = T.block<3, 3>(0, 0);
                Eigen::Vector3d p_world_cylinder = R * Eigen::Vector3d(p_x_cylinder, p_y_cylinder, p_z_cylinder);
                geometry_msgs::Point p_cylinder;
                p_cylinder.x= p_world_cylinder[0] + T(0, 3);
                p_cylinder.y= p_world_cylinder[1] + T(1, 3);
                p_cylinder.z= p_world_cylinder[2] + T(2, 3);
                marker_cylinder.points.push_back(p_cylinder);

                //version: 椭圆
                double angle_divide_z =  M_PI / obj->mIE_rows;
                double theta = angle_divide_z * (y+0.5) - M_PI/2.0;
                double xy_project = a_aix_ellipse * cos(theta);  //水平面的投影
                //z
                double p_z_ellipse = b_aix_ellipse * sin(theta);  //纵坐标
                double diameter = xy_project;
                //xy
                double p_x_ellipse = cos(angle) * diameter;
                double p_y_ellipse = sin(angle) * diameter;
                // 物体坐标系 -> 世界坐标系
                geometry_msgs::Point p_ellipse;
                Eigen::Vector3d p_world_ellipse = R * Eigen::Vector3d(p_x_ellipse, p_y_ellipse, p_z_ellipse);
                p_ellipse.x= p_world_ellipse[0] + T(0, 3);
                p_ellipse.y= p_world_ellipse[1] + T(1, 3);
                p_ellipse.z= p_world_ellipse[2] + T(2, 3);
                marker_ellipse.points.push_back(p_ellipse);

                //version: 半椭圆
                angle_divide_z =  M_PI / obj->mIE_rows /2.0;
                theta = angle_divide_z * (y) ;
                double xy_project_half = a_aix_half * cos(theta);  //水平面的投影
                //z
                double p_z_half = b_aix_half * sin(theta);  //纵坐标
                diameter = xy_project_half;
                //xy
                double p_x_half = cos(angle) * diameter;
                double p_y_half = sin(angle) * diameter;
                // 物体坐标系 -> 世界坐标系
                geometry_msgs::Point p_half;
                Eigen::Vector3d p_world_half = R * Eigen::Vector3d(p_x_half, p_y_half, p_z_half);
                p_half.x= p_world_half[0] + T(0, 3);
                p_half.y= p_world_half[1] + T(1, 3);
                p_half.z= p_world_half[2] + T(2, 3) - obj->mCuboid3D.height/2.0;
                marker_half.points.push_back(p_half);


                std_msgs::ColorRGBA c;
                if(obj->mvGridProb_mat.at<float>(x,y) > 0.5){
                    //marker.color.r =1.0; marker.color.g = 1.0; marker.color.b = 1.0; marker.color.a = 1.0;
                    //marker.color.r =color[2]/255.0; marker.color.g = color[1]/255.0; marker.color.b = color[0]/255.0; marker.color.a = 0.7;
                    //marker.color.r =255.0; marker.color.g = 255.0; marker.color.b = 255.0; marker.color.a = 0.7;
                    //marker.color.r =0.0; marker.color.g = 0.0; marker.color.b = 0.0; marker.color.a = 0.7;
                    c.r =color[2]/255.0; c.g = color[1]/255.0; c.b = color[0]/255.0; c.a = 0.7;
                }
                else if(obj->mvGridProb_mat.at<float>(x,y) < 0.5){
                    //marker.color.r =0.0; marker.color.g = 0.0; marker.color.b = 0.0; marker.color.a = 1.0;
                    //marker.color.r =color[2]/255.0; marker.color.g = color[1]/255.0; marker.color.b = color[0]/255.0; marker.color.a = 0.15;
                    //marker.color.r =255.0; marker.color.g = 255.0; marker.color.b = 255.0; marker.color.a = 0.7;
                    c.r =color[2]/255.0; c.g = color[1]/255.0; c.b = color[0]/255.0; c.a = 0.15;
                }
                else {
                    //marker.color.r =0.0; marker.color.g = 0.0; marker.color.b = 0.0; marker.color.a = 0.5;
                    c.r =0.0; c.g = 0.0; c.b = 0.0; c.a = 0.1;
                }


                marker_cylinder.colors.push_back(c);
                marker_ellipse.colors.push_back(c);
                marker_half.colors.push_back(c);
                //usleep(100);
            }
        }
    }
    //version all_publish:
    //marker.id= 0;
    publisher_IE_cylinder.publish(marker_cylinder);
    publisher_IE_ellipse.publish(marker_ellipse);
    publisher_IE_half.publish(marker_half);
}

void MapPublisher::PublishMainDirection(const vector<Object_Map*> &vObjs ){
    // color.
    std::vector<vector<float> > colors_bgr{ {135,0,248},  {255,0,253},  {4,254,119},  {255,126,1},  {0,112,255},  {0,250,250}   };
    vector<float> color;

    //生成 rviz marker
    visualization_msgs::Marker marker;
    marker.header.frame_id = MAP_FRAME_ID;
    marker.ns = "MainDirection";
    //marker.lifetime = ros::Duration(5.0);
    marker.lifetime = ros::Duration(mDirection_Duration);
    marker.type = visualization_msgs::Marker::POINTS;
    marker.scale.x=0.03;
    marker.scale.y=0.08;
    marker.pose.orientation.w=1.0;  //????
    marker.action=visualization_msgs::Marker::ADD;


    for(size_t i=0; i< vObjs.size(); i++){
        Object_Map* obj = vObjs[i];

        if((obj->mvpMapObjectMappoints.size() < 10) || (obj->bad_3d == true)  || obj->backgroud_object )
        {
            continue;
        }

        color = colors_bgr[obj->mnClass % 6];
        double diameter = sqrt(obj->mCuboid3D.width * obj->mCuboid3D.width   +   obj->mCuboid3D.lenth * obj->mCuboid3D.lenth )/2.0;
        for(int x=0; x<obj->mIE_rows; x++){
            double angle_divide = 2*M_PI/obj->mIE_rows;
            double angle = angle_divide * ( x + 0.5 );
            double p_x = cos(angle) * diameter;
            double p_y = sin(angle) * diameter;

            double h_divide =  obj->mCuboid3D.height/obj->mIE_cols;
            for(int y=0; y<obj->mIE_cols; y++){
                //计算纵坐标
                double p_z = h_divide * (y+0.5) - obj->mCuboid3D.height/2.0;

                // 物体坐标系 -> 世界坐标系
                cv::Mat cvMat4 = obj->mCuboid3D.pose_mat.clone();
                Eigen::Matrix4f eigenMat4f;
                cv::cv2eigen(cvMat4, eigenMat4f);
                //Eigen::Matrix4d T = ORB_SLAM2::Converter::cvMattoMatrix4d(obj->mCuboid3D.pose_mat);
                Eigen::Matrix4d T = eigenMat4f.cast<double>();
                Eigen::Matrix3d R = T.block<3, 3>(0, 0);
                Eigen::Vector3d p_world = R * Eigen::Vector3d(p_x, p_y, p_z);
                geometry_msgs::Point p;
                p.x= p_world[0] + T(0, 3);
                p.y= p_world[1] + T(1, 3);
                p.z= p_world[2] + T(2, 3);

                if(obj->mvGridProb_mat.at<float>(x,y) > 0.5){
                    //marker.color.r =1.0; marker.color.g = 1.0; marker.color.b = 1.0; marker.color.a = 1.0;
                    marker.color.r =color[2]/255.0; marker.color.g = color[1]/255.0; marker.color.b = color[0]/255.0; marker.color.a = 0.7;
                }
                else if(obj->mvGridProb_mat.at<float>(x,y) < 0.5){
                    //marker.color.r =0.0; marker.color.g = 0.0; marker.color.b = 0.0; marker.color.a = 1.0;
                    marker.color.r =color[2]/255.0; marker.color.g = color[1]/255.0; marker.color.b = color[0]/255.0; marker.color.a = 0.15;
                }
                else {
                    marker.color.r =1.0; marker.color.g = 1.0; marker.color.b = 1.0; marker.color.a = 0.2;
                }
                marker.id= ++IE_id;
                marker.points.push_back(p);
                publisher_IE_maindirection.publish(marker);
                //usleep(100);
            }
        }
    }

}


void MapPublisher::SetCurrentCameraPose(const cv::Mat &Tcw)    //zhangjiadong  用在map.cc中
{
    unique_lock<mutex> lock(mMutexCamera);
    mCameraPose = Tcw.clone();
    mbCameraUpdated = true;
}

void MapPublisher::SetMAM(const double &angle){         //zhangjiadong  用在mam的发布中
    mMAM_angle = angle;
    mbMAMUpdated = true;
}

cv::Mat MapPublisher::GetCurrentCameraPose()
{
    unique_lock<mutex> lock(mMutexCamera);
    return mCameraPose.clone();
}

bool MapPublisher::isCamUpdated()
{
    unique_lock<mutex> lock(mMutexCamera);
    return mbCameraUpdated;
}

void MapPublisher::ResetCamFlag()
{
    unique_lock<mutex> lock(mMutexCamera);
    mbCameraUpdated = false;
}


} //namespace ORB_SLAM
