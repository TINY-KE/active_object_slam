#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <iostream>

#include <ignition/math/Vector3.hh>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


#include <iostream>
#include <Eigen/Dense>

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>
#include<opencv2/core/eigen.hpp>

#include<Converter.h>

#include <fstream>

using namespace std;

ros::Publisher publisher_nbv;
visualization_msgs::Marker mReferenceKeyFrames;
//visualization_msgs::Marker mCovisibilityGraph;
visualization_msgs::Marker mMST;
visualization_msgs::Marker mNBVs;

void PublishNBVs(const vector<cv::Mat> &nbvs)
{

    mNBVs.points.clear();
    //mCovisibilityGraph.points.clear();
    mMST.points.clear();

    float d = 0.04;

    //Camera is a pyramid. Define in camera coordinate system
    cv::Mat o = (cv::Mat_<float>(4,1) << 0, 0, 0, 1);
    cv::Mat p1 = (cv::Mat_<float>(4,1) << d, d*0.8, d*0.5, 1);
    cv::Mat p2 = (cv::Mat_<float>(4,1) << d, -d*0.8, d*0.5, 1);
    cv::Mat p3 = (cv::Mat_<float>(4,1) << -d, -d*0.8, d*0.5, 1);
    cv::Mat p4 = (cv::Mat_<float>(4,1) << -d, d*0.8, d*0.5, 1);

    for(size_t i=0, iend=nbvs.size() ; i < iend; i++)
    {

        cv::Mat Twc = nbvs[i];
        //根据 Ow.copyTo(Twc.rowRange(0,3).col(3));
        cv::Mat ow = nbvs[i].rowRange(0,3).col(3).clone();//->GetCameraCenter();
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

        mNBVs.points.push_back(msgs_o);
        mNBVs.points.push_back(msgs_p1);
        mNBVs.points.push_back(msgs_o);
        mNBVs.points.push_back(msgs_p2);
        mNBVs.points.push_back(msgs_o);
        mNBVs.points.push_back(msgs_p3);
        mNBVs.points.push_back(msgs_o);
        mNBVs.points.push_back(msgs_p4);
        mNBVs.points.push_back(msgs_p1);
        mNBVs.points.push_back(msgs_p2);
        mNBVs.points.push_back(msgs_p2);
        mNBVs.points.push_back(msgs_p3);
        mNBVs.points.push_back(msgs_p3);
        mNBVs.points.push_back(msgs_p4);
        mNBVs.points.push_back(msgs_p4);
        mNBVs.points.push_back(msgs_p1);

        //// Covisibility Graph
        //vector<KeyFrame*> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
        //if(!vCovKFs.empty())
        //{
        //    for(vector<KeyFrame*>::iterator vit=vCovKFs.begin(), vend=vCovKFs.end(); vit!=vend; vit++)
        //    {
        //        if((*vit)->mnId<vpKFs[i]->mnId)
        //            continue;
        //        cv::Mat Ow2 = (*vit)->GetCameraCenter();
        //        geometry_msgs::Point msgs_o2;
        //        msgs_o2.x=Ow2.at<float>(0);
        //        msgs_o2.y=Ow2.at<float>(1);
        //        msgs_o2.z=Ow2.at<float>(2);
        //        mCovisibilityGraph.points.push_back(msgs_o);
        //        mCovisibilityGraph.points.push_back(msgs_o2);
        //    }
        //}

        // MST
        if(i>0)
        {
            cv::Mat Owp = nbvs[i-1].rowRange(0,3).col(3).clone();//->GetCameraCenter();;
            geometry_msgs::Point msgs_op;
            msgs_op.x=Owp.at<float>(0);
            msgs_op.y=Owp.at<float>(1);
            msgs_op.z=Owp.at<float>(2);
            mMST.points.push_back(msgs_o);
            mMST.points.push_back(msgs_op);
        }
        //set<KeyFrame*> sLoopKFs = nbvs[i]->GetLoopEdges();
        //for(set<KeyFrame*>::iterator sit=sLoopKFs.begin(), send=sLoopKFs.end(); sit!=send; sit++)
        //{
        //    if((*sit)->mnId < nbvs[i]->mnId)
        //        continue;
        //    cv::Mat Owl = (*sit)->GetCameraCenter();
        //    geometry_msgs::Point msgs_ol;
        //    msgs_ol.x=Owl.at<float>(0);
        //    msgs_ol.y=Owl.at<float>(1);
        //    msgs_ol.z=Owl.at<float>(2);
        //    mMST.points.push_back(msgs_o);
        //    mMST.points.push_back(msgs_ol);
        //}
    }

    mNBVs.header.stamp = ros::Time::now();
    //mCovisibilityGraph.header.stamp = ros::Time::now();
    mMST.header.stamp = ros::Time::now();

    publisher_nbv.publish(mNBVs);
    //publisher_CoView.publish(mCovisibilityGraph);
    publisher_nbv.publish(mMST);
}


int main(int argc, char **argv)
{   std::string filePath;
    if(argc != 2){
        std::cout<<"【注意】：没有输入被读取的nbv文件地址"<<std::endl;
        std::cout<<"读取默认文件：/home/zhjd/active_eao/src/active_eao/eval/GlobalNBV.txt"<<std::endl;
        filePath = "/home/zhjd/active_eao/src/active_eao/eval/GlobalNBV.txt";
    }
    else
        filePath = argv[1];

    const char* MAP_FRAME_ID = "map"; //  odom   imu_link   /ORB_SLAM/World    map
    const char* POINTS_NAMESPACE = "MapPoints";
    const char* KEYFRAMES_NAMESPACE = "KeyFrames";
    const char* PLANES_NAMESPACE = "MapPlanes";
    const char* OBJECTS_NAMESPACE = "MapObjects";
    const char* GRAPH_NAMESPACE = "Graph";
    const char* CAMERA_NAMESPACE = "Camera";

    //Configure KeyFrames
    mNBVs.header.frame_id = MAP_FRAME_ID;
    mNBVs.ns = KEYFRAMES_NAMESPACE;
    mNBVs.id=1;
    mNBVs.type = visualization_msgs::Marker::LINE_LIST;
    mNBVs.scale.x=0.005;
    mNBVs.pose.orientation.w=1.0;
    mNBVs.action=visualization_msgs::Marker::ADD;

    mNBVs.color.r = 1.0f;
    mNBVs.color.a = 1.0;

    //Configure KeyFrames Spanning Tree  关键帧中心的连线
    mMST.header.frame_id = MAP_FRAME_ID;
    mMST.ns = GRAPH_NAMESPACE;
    mMST.id=3;
    mMST.type = visualization_msgs::Marker::LINE_LIST;
    mMST.scale.x=0.005;
    mMST.pose.orientation.w=1.0;
    mMST.action=visualization_msgs::Marker::ADD;
    mMST.color.r=0.0f;
    mMST.color.b=0.0f;
    mMST.color.g=0.0f;
    mMST.color.a = 1.0;

    std::vector<cv::Mat> NBVs;
    ros::init ( argc, argv, "gazebo_world_parser" );
    ros::NodeHandle nh;
    publisher_nbv = nh.advertise<visualization_msgs::Marker>("nbv_trajectory", 1000);



    std::ifstream infile(filePath, std::ios::in);
    if (!infile.is_open())
    {
        std::cout << "open fail: "<< filePath <<" " << endl;
        exit(233);
    }
    else
    {
        std::cout << "read NBVs.txt" << std::endl;
    }

    std::vector<double> row;

    cv::Mat cam_pose_mat;
    int mnid_current = -1;
    //string s0;
    //getline(infile, s0);  注销掉无用的line
    NBVs.clear();
    string line;

    while (getline(infile, line))
    {
        istringstream istr(line);
        double num,tx,ty,tz,qx,qy,qz,qw;

        //存nbv
        // 四元数   v[0] = q.x();
        //v[1] = q.y();
        //v[2] = q.z();
        //v[3] = q.w();
        //<< i << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
        //  << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

        //存物体< q[0] << " " << q[1] << " " << q[2] << " " << q[3]
        //f_point     << "1 "  //物体
        //            << object->mnId << "   "
        //            << object->mnClass << " "
        //            << object->mnConfidence_foractive << " "
        //            << object->mvpMapObjectMappoints.size() << "     "
        //            << pose.translation().x() << " "
        //            << pose.translation().y() << " "
        //            << pose.translation().z()<< "     "
        //            << pose.rotation().x() << " "
        //            << pose.rotation().y() << " "
        //            << pose.rotation().z() << " "
        //            << pose.rotation().w() << "     "
        //            << object->mCuboid3D.lenth << " "
        //            << object->mCuboid3D.width << " "
        //            << object->mCuboid3D.height << " "
        //            << endl;

        //说明四元数的顺序都是xyzw

        istr >> num;

        double temp;
        Eigen::MatrixXd object_poses(1, 8); ;
        istr >> temp;  object_poses(0) = temp;  //obj->mCuboid3D.cuboidCenter0 = temp;
        istr >> temp;  object_poses(1) = temp;  //obj->mCuboid3D.cuboidCenter1 = temp;
        istr >> temp;  object_poses(2) = temp;  //obj->mCuboid3D.cuboidCenter2 = temp;
        istr >> temp;  object_poses(3) = temp;
        istr >> temp;  object_poses(4) = temp;
        istr >> temp;  object_poses(5) = temp;
        istr >> temp;  object_poses(6) = temp;
        g2o::SE3Quat cam_pose_se3(object_poses.row(0).head(7));

        cv::Mat nbv_pose = ORB_SLAM2::Converter::toCvMat(cam_pose_se3);

        NBVs.push_back(nbv_pose);

        row.clear();
        istr.clear();
        line.clear();
    }

    std::cout<<"NBV size:"<<NBVs.size()<<std::endl;

    ros::Rate rate(10);
    while (nh.ok()){
        PublishNBVs(NBVs);
    }  


    return 0;

}
