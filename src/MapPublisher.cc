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


MapPublisher::MapPublisher(Map* pMap):mpMap(pMap), mbCameraUpdated(false)
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

    //Configure MapPlanes 平面的中心法线
    mPlanes.header.frame_id = MAP_FRAME_ID;
    mPlanes.ns = POINTS_NAMESPACE;
    mPlanes.id=7;
    mPlanes.type = visualization_msgs::Marker::POINTS;
    mPlanes.scale.x=10*fPointSize;
    mPlanes.scale.y=10*fPointSize;
    mPlanes.pose.orientation.w=1.0;
    mPlanes.action=visualization_msgs::Marker::ADD;

    //
    mCandidate.header.frame_id = MAP_FRAME_ID;
    mCandidate.ns = CANDIDATE_NAMESPACE;
    mCandidate.id=8;
    mCandidate.type = visualization_msgs::Marker::POINTS;
    mCandidate.pose.orientation.w=1.0;
    mCandidate.action=visualization_msgs::Marker::ADD;


    //Configure MapObjectPoints

    //Configure MapObjects
    object_id_init = 9;


    //Configure Publisher
    publisher = nh.advertise<visualization_msgs::Marker>("objectmap", 1000);
    publisher_IE = nh.advertise<visualization_msgs::Marker>("object_ie", 1000);
    pubCloud = nh.advertise<sensor_msgs::PointCloud2>("plane", 1000);
    publisher_candidate = nh.advertise<visualization_msgs::Marker>("candidate", 1000);

    publisher.publish(mPoints);
    publisher.publish(mReferencePoints);
    publisher.publish(mCovisibilityGraph);
    publisher.publish(mKeyFrames);
    publisher.publish(mCurrentCamera);

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
        vector<MapPlane *> vpMPlanes = mpMap->GetAllMapPlanes();
        PublishMapPoints(vMapPoints, vRefMapPoints);   
        PublishKeyFrames(vKeyFrames);
        //PublishPlane(vMapPlanes);
        PublishObject(vMapObjects);
        //PublishIE(vMapObjects);
        PublishMapPlanes(vpMPlanes);
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

    publisher.publish(mKeyFrames);
    publisher.publish(mCovisibilityGraph);
    publisher.publish(mMST);
}

void MapPublisher::PublishCurrentCamera(const cv::Mat &Tcw)
{
    if(Tcw.empty())
        return ;

    mCurrentCamera.points.clear();

    float d = fCameraSize;

    //Camera is a pyramid. Define in camera coordinate system
    cv::Mat o = (cv::Mat_<float>(4,1) << 0, 0, 0, 1);
    cv::Mat p1 = (cv::Mat_<float>(4,1) << d, d*0.8, d*0.5, 1);
    cv::Mat p2 = (cv::Mat_<float>(4,1) << d, -d*0.8, d*0.5, 1);
    cv::Mat p3 = (cv::Mat_<float>(4,1) << -d, -d*0.8, d*0.5, 1);
    cv::Mat p4 = (cv::Mat_<float>(4,1) << -d, d*0.8, d*0.5, 1);

    cv::Mat Twc = Tcw.inv();
    cv::Mat ow = Twc*o;
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

    publisher.publish(mCurrentCamera);
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

    //for(size_t i=vObjs.size(); i<object_num_last ; i++)
    //{
    //    visualization_msgs::Marker marker;
    //    marker.id = object_id_init + i;
    //    marker.header.frame_id= MAP_FRAME_ID;
    //    marker.header.stamp=ros::Time::now();
    //    marker.action = visualization_msgs::Marker::DELETE;
    //    publisher.publish(marker);
    //}
    //object_num_last = vObjs.size();

    for(size_t i=0; i< vObjs.size(); i++)
    {

        // color.
        std::vector<vector<float> > colors_bgr{ {135,0,248},  {255,0,253},  {4,254,119},  {255,126,1},  {0,112,255},  {0,250,250}   };
        vector<float> color = colors_bgr[vObjs[i]->mnClass % 6];


        //用于物体的颜色随机
        std::default_random_engine e;
        std::uniform_real_distribution<double>  random(0.5,1);
        float r = random(e); float g = random(e); float b = random(e);

        //物体
        visualization_msgs::Marker marker;
        marker.id = ++object_id_init;//object_id_init + i;
        marker.lifetime = ros::Duration(0.1);
        marker.header.frame_id= MAP_FRAME_ID;
        marker.header.stamp=ros::Time::now();

        if((vObjs[i]->mvpMapObjectMappoints.size() < 10) || (vObjs[i]->bad_3d == true))
        {
            continue;
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

        publisher.publish(marker);


        visualization_msgs::Marker marker1;
        marker1.header.frame_id = MAP_FRAME_ID;
        marker1.ns = "ObjectPoints";
        marker1.lifetime = ros::Duration(0.2);
        marker1.id= ++object_id_init;
        marker1.type = visualization_msgs::Marker::POINTS;
        marker1.scale.x=0.02;
        marker1.scale.y=0.02;
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
        publisher.publish(marker1);

        vector<float> color_new = colors_bgr[(vObjs[i]->mnClass+1) % 6];
        visualization_msgs::Marker marker2;
        marker2.header.frame_id = MAP_FRAME_ID;
        marker2.ns = "NewObjectPoints";
        marker2.lifetime = ros::Duration(0.2);
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
        publisher.publish(marker2);
    }

}

void MapPublisher::PublishIE(const vector<Object_Map*> &vObjs ){
    // color.
    std::vector<vector<float> > colors_bgr{ {135,0,248},  {255,0,253},  {4,254,119},  {255,126,1},  {0,112,255},  {0,250,250}   };
    vector<float> color;

    for(size_t i=0; i< vObjs.size(); i++){
        double  pie = 3.1415926;
        Object_Map* obj = vObjs[i];

        if((obj->mvpMapObjectMappoints.size() < 10) || (obj->bad_3d == true))
        {
            continue;
        }

        color = colors_bgr[obj->mnClass % 6];
        double diameter = sqrt(obj->mCuboid3D.width * obj->mCuboid3D.width   +   obj->mCuboid3D.lenth * obj->mCuboid3D.lenth )/2.0;
        for( int x=0; x<obj->vInforEntroy.size(); x++){
            double angle_divide = 2*pie/obj->vInforEntroy.size();
            double angle = angle_divide * ( x + 0.5 );
            double p_x = cos(angle) * diameter;
            double p_y = sin(angle) * diameter;

            double h_divide =  obj->mCuboid3D.height/obj->vInforEntroy[x].size();
            for( int y=0; y<obj->vInforEntroy[x].size(); y++){
                //计算纵坐标
                double p_z = h_divide * (y+0.5) - obj->mCuboid3D.height/2.0;

                // 物体坐标系 -> 世界坐标系
                Eigen::Vector3d p_world = Converter::toSE3Quat(obj->mCuboid3D.pose_mat) * Eigen::Vector3d(p_x, p_y, p_z);
                geometry_msgs::Point p;
                p.x=p_world[0];
                p.y=p_world[1];
                p.z=p_world[2];

                //生成 rviz marker
                visualization_msgs::Marker marker;
                marker.header.frame_id = MAP_FRAME_ID;
                marker.ns = "InformationEntroy";
                marker.lifetime = ros::Duration(5.0);
                marker.id= (x*obj->vInforEntroy.size() + y) + obj->mnId*18*18  ;  //TODO:绝对数字
                marker.type = visualization_msgs::Marker::POINTS;
                marker.scale.x=0.03;
                marker.scale.y=0.08;
                marker.pose.orientation.w=1.0;  //????
                marker.action=visualization_msgs::Marker::ADD;
                //double color = obj->vInforEntroy[x][y];  marker.color.r = color; marker.color.g = color; marker.color.b = color; marker.color.a = 1.0;
                //std::cout<< "x" <<x << ", y" <<y << ", prob" <<obj->vgrid_prob[x][y] <<std::endl;
                if(obj->vgrid_prob[x][y]>0.5){
                    //marker.color.r =1.0; marker.color.g = 1.0; marker.color.b = 1.0; marker.color.a = 1.0;
                    marker.color.r =color[2]/255.0; marker.color.g = color[1]/255.0; marker.color.b = color[0]/255.0; marker.color.a = 0.7;
                }
                else if(obj->vgrid_prob[x][y]<0.5){
                    //marker.color.r =0.0; marker.color.g = 0.0; marker.color.b = 0.0; marker.color.a = 1.0;
                    marker.color.r =color[2]/255.0; marker.color.g = color[1]/255.0; marker.color.b = color[0]/255.0; marker.color.a = 0.15;
                }
                else {
                    marker.color.r =1.0; marker.color.g = 1.0; marker.color.b = 1.0; marker.color.a = 0.2;
                }
                marker.points.push_back(p);
                publisher_IE.publish(marker);
            }
        }
        //std::cout<<std::endl<<std::endl<<std::endl;


    }
}
void MapPublisher::SetCurrentCameraPose(const cv::Mat &Tcw)    //zhangjiadong  用在map.cc中
{
    unique_lock<mutex> lock(mMutexCamera);
    mCameraPose = Tcw.clone();
    mbCameraUpdated = true;
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


void MapPublisher::PublishMapPlanes(const vector<MapPlane *> &vpMPs)
{
    mPlanes.points.clear();
    mPlanes.lifetime = ros::Duration(0.2);
    mCandidate.points.clear();

    if (vpMPs.empty())
        return;
    //降维过滤器
    pcl::VoxelGrid<PointT> voxel;
    voxel.setLeafSize(0.002, 0.002, 0.002);

    //带颜色的pcl point
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr  colored_pcl_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    colored_pcl_ptr->points.clear();
    // color.
    std::vector<vector<float> > colors_bgr{ {135,0,248},  {255,0,253},  {4,254,119},  {255,126,1},  {0,112,255},  {0,250,250}   };
    int num = 0;

    for (auto pMP : vpMPs)  //对vpMPs中每个平面pMP分别进行处理,
    {
        // 计算平面与地面的夹角(cos值), 如果夹角很小,则认为水平面. 可以显示
        cv::Mat groud = (cv::Mat_<float>(3, 1) << 0, 0, 1);  ;
        cv::Mat pMP_normal = pMP->GetWorldPos();
        float angle = groud.at<float>(0, 0) * pMP_normal.at<float>(0, 0) +
                      groud.at<float>(1, 0) * pMP_normal.at<float>(1, 0) +
                      groud.at<float>(2, 0) * pMP_normal.at<float>(2, 0);
        if ((angle < 0.2) && (angle > -0.2))
            continue;

        //颜色
        float ir = pMP->mRed;
        float ig = pMP->mGreen;
        float ib = pMP->mBlue;

        //计算当前平面,在各关键帧中对应的平面
        map<KeyFrame *, int> observations = pMP->GetObservations();  //std::map<KeyFrame*, int> mObservations;

        //将各关键帧中的平面,融合为一个allCloudPoints
        PointCloud::Ptr allCloudPoints(new PointCloud);
        float x=0, y=0, z=0;
        for (auto mit = observations.begin(), mend = observations.end(); mit != mend; mit++)
        {
            KeyFrame *frame = mit->first;
            int id = mit->second;
            if (id >= frame->mnRealPlaneNum)
            {
                continue;
            }
            Eigen::Isometry3d T = ORB_SLAM2::Converter::toSE3Quat(frame->GetPose());
            PointCloud::Ptr cloud(new PointCloud);
            pcl::transformPointCloud(frame->mvPlanePoints[id], *cloud, T.inverse().matrix());
            *allCloudPoints += *cloud;
            //center += frame->mvPlanePoints[id];
        }

        //对allCloudPoints降维成tmp
        PointCloud::Ptr tmp(new PointCloud());
        voxel.setInputCloud(allCloudPoints);
        voxel.filter(*tmp);


        // 计算allCloudPoint的中心点
        vector<float> vec_x,vec_y,vec_z;
        for (size_t i = 0; i < allCloudPoints->points.size(); i++)
        {
            vec_x.push_back(allCloudPoints->points[i].x);
            vec_y.push_back(allCloudPoints->points[i].y);
            vec_z.push_back(allCloudPoints->points[i].z);
        }
        double mean_x,mean_y,mean_z;	//点云均值
	    double stddev_x,stddev_y,stddev_z;	//点云标准差
        pcl::getMeanStd(vec_z, mean_z, stddev_z);
        cout<< "mean z1:"<<mean_z<<std::endl;
        if(mean_z>1.7 || mean_z<0.5)
            continue;
        cout<< "mean z2:"<<mean_z<<std::endl;
        pcl::getMeanStd(vec_x, mean_x, stddev_x);
        pcl::getMeanStd(vec_y, mean_y, stddev_y);

        // 将中心点, 转为rviz marker
        geometry_msgs::Point p;
        p.x = mean_x;//center.at<float>(0,0);
        p.y = mean_y;//center.at<float>(1,0);
        p.z = mean_z;//center.at<float>(2,0);
        mPlanes.points.push_back(p);


        //如果能投影到当前帧，在处理
        //为什么会有平面没有处理掉？
        //计算下时间
        //ComputePlanesFromOrganizedPointCloud

         //tmp转为pcl::PointXYZRGB, 才能显示颜色
        vector<float> color = colors_bgr[pMP->mnId % 6];
        for (int i = 0; i <  tmp->points.size(); i++)
        {
          pcl::PointXYZRGB  p;
          p.x=tmp->points[i].x;
          p.y=tmp->points[i].y;
          p.z=tmp->points[i].z;
          //p.r = 255*ir;//color[2];
          p.r = color[2];
          //p.g = 255*ig;//color[1];
          p.g = color[1];
          //p.b = 255*ib;//color[0];
          p.b = color[0];
          colored_pcl_ptr->points.push_back(p);
        }


        //计算tmp的边缘点
        //(1)将tmp转为no_color
        pcl::PointCloud<pcl::PointXYZ>::Ptr  nocolored_pcl_ptr (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*tmp, *nocolored_pcl_ptr);
        ////（2）计算法向量
        //pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normEst;
        //pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        //normEst.setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr(nocolored_pcl_ptr));
        //normEst.setRadiusSearch(0.1);
        //normEst.compute(*normals);
        ////（3）计算边缘
        //pcl::PointCloud<pcl::Boundary> boundaries;
        //pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> boundEst;
        //boundEst.setInputCloud( nocolored_pcl_ptr  );
        //boundEst.setInputNormals(normals);
        //boundEst.setRadiusSearch(0.1);
        //boundEst.setAngleThreshold(M_PI / 4);
        //boundEst.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));
        //boundEst.compute(boundaries);

        ////(4)将boundaries转为colored_pcl_ptr
        //int boundary_points_num = 0;
        //for (int i = 0; i < tmp->points.size(); i++)
        //{
        //
        //    if (boundaries[i].boundary_point > 0)
        //    {
        //        pcl::PointXYZRGB  p;
        //        p.x=tmp->points[i].x;
        //        p.y=tmp->points[i].y;
        //        p.z=tmp->points[i].z;
        //        p.r = 255.0;
        //        p.g = 0;
        //        p.b = 0;
        //        colored_pcl_ptr->push_back(p);
        //        boundary_points_num ++;
        //    }
        //}

        //(新2)经纬线扫描法
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boundary(new pcl::PointCloud<pcl::PointXYZ>);
        BoundaryExtraction(nocolored_pcl_ptr, cloud_boundary, 200);

        //(3)cloud_boundary转为colored_pcl_ptr
        vector<float> color2 = colors_bgr[(pMP->mnId+2) % 6];
        for (int i = 0; i <  cloud_boundary->points.size(); i++)
        {
          pcl::PointXYZ pno = cloud_boundary->points[i];
          pcl::PointXYZRGB  p;
          p.x= pno.x;
          p.y= pno.y;
          p.z= pno.z;
          //p.r = 255*ir;//color[2];
          p.r = color2[2];
          //p.g = 255*ig;//color[1];
          p.g = color2[1];
          //p.b = 255*ib;//color[0];
          p.b = color2[0];
          colored_pcl_ptr->points.push_back(p);
        }
        //(4)cloud_boundary转为candidates
        double safe_radius = 0.5;
        int divide = 20;
        int step = floor( cloud_boundary->points.size() / divide);
        for(int i=0; i< divide; i++){
            int index = i*step+ divide/2 ;
            double x,y;
            double d = sqrt(    (cloud_boundary->points[index].x-mean_x)*(cloud_boundary->points[index].x-mean_x)
                            +   (cloud_boundary->points[index].y-mean_y)*(cloud_boundary->points[index].y-mean_y)
                            )
                       + safe_radius;
            double k = (cloud_boundary->points[index].y-mean_y) /
                     (cloud_boundary->points[index].x-mean_x);

            double x_positive = sqrt(d * d / (1 + k * k) ) + mean_x;
            double x_negative = -sqrt(d * d / (1 + k * k) ) + mean_x;
            if( (x_positive-mean_x)*(cloud_boundary->points[index].x-mean_x) > 0 )
                x = x_positive;
            else
                x = x_negative;
            y = k*(x - mean_x) + mean_y;
            geometry_msgs::Point p;
            p.x=x;
            p.y=y;
            p.z=mean_z;
            mCandidate.points.push_back(p);
        }

        // 桌面高度的 水平面的数量
        num++;

    }
    cout << "-------" << endl << endl;
    cout << "桌面高度的水平面的数量: " <<num << endl;

    //发布中心点的rviz marker
    mPlanes.header.stamp = ros::Time::now();
    mPlanes.color.r = 0.0; mPlanes.color.g = 0.0; mPlanes.color.b = 1.0;  mPlanes.color.a = 1.0;
    publisher.publish(mPlanes);

    //发布中心店
    mCandidate.header.stamp = ros::Time::now();
    mCandidate.color.r=0.0f;
    mCandidate.color.b=0.0f;
    mCandidate.color.g=1.0f;
    mCandidate.color.a=1.0f;
    mCandidate.scale.x=10*fPointSize;
    mCandidate.scale.y=10*fPointSize;
    publisher_candidate.publish(mCandidate);

    // 发布平面的点云
    sensor_msgs::PointCloud2 colored_msg;
    colored_pcl_ptr->width = 1;
    colored_pcl_ptr->height = colored_pcl_ptr->points.size();
    pcl::toROSMsg( *colored_pcl_ptr,  colored_msg);  //将点云转化为消息才能发布
    colored_msg.header.frame_id = MAP_FRAME_ID;//帧id改成和velodyne一样的
    pubCloud.publish( colored_msg); //发布调整之后的点云数据，主题为/adjustd_cloud

}


} //namespace ORB_SLAM
