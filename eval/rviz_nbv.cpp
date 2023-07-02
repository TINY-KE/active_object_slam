/*
* 修改/home/zhjd/fabo_gazebo/src/fabo_moveit_gazebo/ASLAM_gazebo_world/urdf/robot_hokuyo_kinectv1neck.urdf.xacro中
* <xacro:kinectv1_neck parent="base_footprint"  中的
* camera_offset_z
* */

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
visualization_msgs::Marker mKFs;
visualization_msgs::Marker mCandidates;
visualization_msgs::Marker mCandidates_1;

void PublishNBVs(const vector<cv::Mat> &VIEWs, const int type)
{
    if(type == 1){   //KeyFrame
        mKFs.points.clear();

        float d = 0.05;

        //Camera is a pyramid. Define in camera coordinate system
        cv::Mat o = (cv::Mat_<float>(4, 1) << 0, 0, 0, 1);
        cv::Mat p1 = (cv::Mat_<float>(4, 1) << d, d * 0.8, d * 0.5, 1);
        cv::Mat p2 = (cv::Mat_<float>(4, 1) << d, -d * 0.8, d * 0.5, 1);
        cv::Mat p3 = (cv::Mat_<float>(4, 1) << -d, -d * 0.8, d * 0.5, 1);
        cv::Mat p4 = (cv::Mat_<float>(4, 1) << -d, d * 0.8, d * 0.5, 1);

        for (size_t i = 0, iend = VIEWs.size(); i < iend; i++) {

            cv::Mat Twc = VIEWs[i];
            //根据 Ow.copyTo(Twc.rowRange(0,3).col(3));
            cv::Mat ow = VIEWs[i].rowRange(0, 3).col(3).clone();//->GetCameraCenter();
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

            mKFs.points.push_back(msgs_o);
            mKFs.points.push_back(msgs_p1);
            mKFs.points.push_back(msgs_o);
            mKFs.points.push_back(msgs_p2);
            mKFs.points.push_back(msgs_o);
            mKFs.points.push_back(msgs_p3);
            mKFs.points.push_back(msgs_o);
            mKFs.points.push_back(msgs_p4);
            mKFs.points.push_back(msgs_p1);
            mKFs.points.push_back(msgs_p2);
            mKFs.points.push_back(msgs_p2);
            mKFs.points.push_back(msgs_p3);
            mKFs.points.push_back(msgs_p3);
            mKFs.points.push_back(msgs_p4);
            mKFs.points.push_back(msgs_p4);
            mKFs.points.push_back(msgs_p1);

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
            if (i > 0) {
                cv::Mat Owp = VIEWs[i - 1].rowRange(0, 3).col(3).clone();//->GetCameraCenter();;
                geometry_msgs::Point msgs_op;
                msgs_op.x = Owp.at<float>(0);
                msgs_op.y = Owp.at<float>(1);
                msgs_op.z = Owp.at<float>(2);
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

        mKFs.header.stamp = ros::Time::now();
        //mCovisibilityGraph.header.stamp = ros::Time::now();
        mMST.header.stamp = ros::Time::now();

        publisher_nbv.publish(mKFs);
        //publisher_CoView.publish(mCovisibilityGraph);
        publisher_nbv.publish(mMST);
    }
    else if (type == 0) //NBV
    {
        mNBVs.points.clear();
        //mCovisibilityGraph.points.clear();
        mMST.points.clear();

        float d = 0.1;

        //Camera is a pyramid. Define in camera coordinate system
        cv::Mat o = (cv::Mat_<float>(4, 1) << 0, 0, 0, 1);
        cv::Mat p1 = (cv::Mat_<float>(4, 1) << d, d * 0.8, d * 0.5, 1);
        cv::Mat p2 = (cv::Mat_<float>(4, 1) << d, -d * 0.8, d * 0.5, 1);
        cv::Mat p3 = (cv::Mat_<float>(4, 1) << -d, -d * 0.8, d * 0.5, 1);
        cv::Mat p4 = (cv::Mat_<float>(4, 1) << -d, d * 0.8, d * 0.5, 1);

        for (size_t i = 0, iend = VIEWs.size(); i < iend; i++) {

            cv::Mat Twc = VIEWs[i];
            //根据 Ow.copyTo(Twc.rowRange(0,3).col(3));
            cv::Mat ow = VIEWs[i].rowRange(0, 3).col(3).clone();//->GetCameraCenter();
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

        }

        mNBVs.header.stamp = ros::Time::now();

        publisher_nbv.publish(mNBVs);

    }
    else if (type == 2) //Candidates
    {
        mCandidates.points.clear();

        float d = 0.05;


        //Camera is a pyramid. Define in camera coordinate system
        cv::Mat bias = (cv::Mat_<float>(4, 1) << 0, 0, 0.3, 1);
        double b = 0.13;
        cv::Mat o = (cv::Mat_<float>(4, 1) << 0, 0, 0  +b, 1);
        cv::Mat p1 = (cv::Mat_<float>(4, 1) << d, d * 0.8, d * 0.5  +b, 1);
        cv::Mat p2 = (cv::Mat_<float>(4, 1) << d, -d * 0.8, d * 0.5  +b, 1);
        cv::Mat p3 = (cv::Mat_<float>(4, 1) << -d, -d * 0.8, d * 0.5  +b, 1);
        cv::Mat p4 = (cv::Mat_<float>(4, 1) << -d, d * 0.8, d * 0.5  +b, 1);
        //o += bias;
        //p1 += bias;
        //p2 += bias;
        //p3 += bias;
        //p4 += bias;

        for (size_t i = 1, iend = VIEWs.size(); i < iend; i++) {

            cv::Mat Twc = VIEWs[i];
            //根据 Ow.copyTo(Twc.rowRange(0,3).col(3));
            //cv::Mat ow = VIEWs[i].rowRange(0, 3).col(3).clone();//->GetCameraCenter();
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

            mCandidates.points.push_back(msgs_o);
            mCandidates.points.push_back(msgs_p1);
            mCandidates.points.push_back(msgs_o);
            mCandidates.points.push_back(msgs_p2);
            mCandidates.points.push_back(msgs_o);
            mCandidates.points.push_back(msgs_p3);
            mCandidates.points.push_back(msgs_o);
            mCandidates.points.push_back(msgs_p4);
            mCandidates.points.push_back(msgs_p1);
            mCandidates.points.push_back(msgs_p2);
            mCandidates.points.push_back(msgs_p2);
            mCandidates.points.push_back(msgs_p3);
            mCandidates.points.push_back(msgs_p3);
            mCandidates.points.push_back(msgs_p4);
            mCandidates.points.push_back(msgs_p4);
            mCandidates.points.push_back(msgs_p1);

        }
        mCandidates.header.stamp = ros::Time::now();


        publisher_nbv.publish(mCandidates);

        {
            cv::Mat Twc = VIEWs[0];
            //根据 Ow.copyTo(Twc.rowRange(0,3).col(3));
            //cv::Mat ow = VIEWs[i].rowRange(0, 3).col(3).clone();//->GetCameraCenter();
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

            mCandidates_1.points.push_back(msgs_o);
            mCandidates_1.points.push_back(msgs_p1);
            mCandidates_1.points.push_back(msgs_o);
            mCandidates_1.points.push_back(msgs_p2);
            mCandidates_1.points.push_back(msgs_o);
            mCandidates_1.points.push_back(msgs_p3);
            mCandidates_1.points.push_back(msgs_o);
            mCandidates_1.points.push_back(msgs_p4);
            mCandidates_1.points.push_back(msgs_p1);
            mCandidates_1.points.push_back(msgs_p2);
            mCandidates_1.points.push_back(msgs_p2);
            mCandidates_1.points.push_back(msgs_p3);
            mCandidates_1.points.push_back(msgs_p3);
            mCandidates_1.points.push_back(msgs_p4);
            mCandidates_1.points.push_back(msgs_p4);
            mCandidates_1.points.push_back(msgs_p1);

        }
        mCandidates_1.header.stamp = ros::Time::now();
        publisher_nbv.publish(mCandidates_1);

    }
}



void read_view(const std::string filePath, std::vector<cv::Mat>& views){
    std::ifstream infile(filePath, std::ios::in);
    if (!infile.is_open())
    {
        std::cout << "open fail: " << filePath << " " << endl;
        exit(233);
    }
    else
    {
        std::cout << "read VIEWs.txt" << std::endl;
    }

    std::vector<double> row;

    cv::Mat cam_pose_mat;
    int mnid_current = -1;
    //string s0;
    //getline(infile, s0);  注销掉无用的line
    views.clear();
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

        views.push_back(nbv_pose);

        row.clear();
        istr.clear();
        line.clear();
    }
}

int main(int argc, char **argv)
{
    std::string NBV_filePath, camera_filePath, candidate_filePath;
    if(argc != 3){
        std::cout<<"【注意】：没有输入被读取的nbv文件地址"<<std::endl;
        std::cout<<"读取默认文件：/home/zhjd/active_eao/src/active_eao/eval/GlobalNBV.txt"<<std::endl;
        NBV_filePath = "/home/zhjd/active_eao/src/active_eao/eval/GlobalNBV.txt";
        camera_filePath = "/home/zhjd/active_eao/src/active_eao/eval/KeyFrameTrajectory.txt";
        candidate_filePath = "/home/zhjd/active_eao/src/active_eao/eval/Candidates.txt";
    }
    else{
        NBV_filePath = argv[1];
        camera_filePath = argv[2];
        candidate_filePath = argv[3];
    }


    const char* MAP_FRAME_ID = "map"; //  odom   imu_link   /ORB_SLAM/World    map
    const char* POINTS_NAMESPACE = "MapPoints";
    const char* KEYFRAMES_NAMESPACE = "KeyFrames";
    const char* PLANES_NAMESPACE = "MapPlanes";
    const char* OBJECTS_NAMESPACE = "MapObjects";
    const char* GRAPH_NAMESPACE = "Graph";
    const char* CAMERA_NAMESPACE = "Camera";

    //Configure NBV
    mNBVs.header.frame_id = MAP_FRAME_ID;
    mNBVs.ns = KEYFRAMES_NAMESPACE;
    mNBVs.id=1;
    mNBVs.type = visualization_msgs::Marker::LINE_LIST;
    mNBVs.scale.x=0.01;
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
    mMST.color.g=1.0f;
    mMST.color.a = 1.0;

    //Configure KeyFrames
    mKFs.header.frame_id = MAP_FRAME_ID;
    mKFs.ns = KEYFRAMES_NAMESPACE;
    mKFs.id=4;
    mKFs.type = visualization_msgs::Marker::LINE_LIST;
    mKFs.scale.x=0.005;
    mKFs.pose.orientation.w=1.0;
    mKFs.action=visualization_msgs::Marker::ADD;
    mKFs.color.b = 1.0f;
    mKFs.color.a = 1.0;

    //Configure KeyFrames
    mCandidates.header.frame_id = MAP_FRAME_ID;
    mCandidates.ns = KEYFRAMES_NAMESPACE;
    mCandidates.id=5;
    mCandidates.type = visualization_msgs::Marker::LINE_LIST;
    mCandidates.scale.x=0.005;
    mCandidates.pose.orientation.w=1.0;
    mCandidates.action=visualization_msgs::Marker::ADD;
    mCandidates.color.r=0.0f;
    mCandidates.color.b=0.0f;
    mCandidates.color.g=0.0f;
    mCandidates.color.a = 1.0;

    mCandidates_1.header.frame_id = MAP_FRAME_ID;
    mCandidates_1.ns = KEYFRAMES_NAMESPACE;
    mCandidates_1.id=6;
    mCandidates_1.type = visualization_msgs::Marker::LINE_LIST;
    mCandidates_1.scale.x=0.005;
    mCandidates_1.pose.orientation.w=1.0;
    mCandidates_1.action=visualization_msgs::Marker::ADD;
    mCandidates_1.color.r=0.0f;
    mCandidates_1.color.b=1.0f;
    mCandidates_1.color.g=0.0f;
    mCandidates_1.color.a = 1.0;

    std::vector<cv::Mat> NBVs, cameras, candidates;
    ros::init ( argc, argv, "gazebo_world_parser" );
    ros::NodeHandle nh;
    publisher_nbv = nh.advertise<visualization_msgs::Marker>("nbv_trajectory", 1000);

    read_view(NBV_filePath, NBVs);
    read_view(camera_filePath, cameras);
    read_view(candidate_filePath, candidates);

    std::cout<<" size: NBV "<<NBVs.size()<<",  camera "<<cameras.size()<<std::endl;

    ros::Rate rate(10);
    while (nh.ok()){
        PublishNBVs(NBVs,0);
        PublishNBVs(cameras,1);
        //PublishNBVs(candidates,2);
    }  


    return 0;

}
