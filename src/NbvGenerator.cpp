//
// Created by zhjd on 11/17/22.
//

#include "NbvGenerator.h"

//NBV MAM
#include<camera.h>
#include <std_msgs/Float64.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>

namespace ORB_SLAM2
{

NbvGenerator::NbvGenerator(){}

NbvGenerator::NbvGenerator(Map* map, Tracking *pTracking, const string &strSettingPath):
mpMap(map), mpTracker(pTracking)
{
    publisher_centroid = nh.advertise<visualization_msgs::Marker>("centriod", 1000);
    pubCloud = nh.advertise<sensor_msgs::PointCloud2>("plane", 1000);
    publisher_candidate = nh.advertise<visualization_msgs::Marker>("candidate", 1000);
    publisher_candidate_unsort = nh.advertise<visualization_msgs::Marker>("candidate_unsort", 1000);
    publisher_nbv = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    publisher_mam = nh.advertise<std_msgs::Float64>("/neck/neck_controller/command", 10);
    publisher_mam_rviz = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/local_nbv", 1000);
    mActionlib = new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>("move_base", true);
    fPointSize=0.01;
    mCandidate.header.frame_id = MAP_FRAME_ID;
    mCandidate.ns = CANDIDATE_NAMESPACE;
    mCandidate.type = visualization_msgs::Marker::LINE_LIST;
    mCandidate.pose.orientation.w=1.0;
    mCandidate.action=visualization_msgs::Marker::ADD;


    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    mfx = fSettings["Camera.fx"];
    mfy = fSettings["Camera.fy"];
    mcx = fSettings["Camera.cx"];
    mcy = fSettings["Camera.cy"];
    mImageWidth = fSettings["Camera.width"];
    mImageHeight = fSettings["Camera.height"];
    mMax_dis = fSettings["Camera.max_dis"];
    mMin_dis = fSettings["Camera.min_dis"];
    mDivide = fSettings["MAM.divide"];   //NBV MAM,  也用在了global nbv中
    mLocalNeckRange = fSettings["MAM.NeckRange"];
    MAM_isused = fSettings["MAM.isused"];
    MAM_neckdelay = fSettings["MAM.neckdelay"];
    mMAM_turn = MAM_neckdelay;

    float qx = fSettings["Trobot_camera.qx"], qy = fSettings["Trobot_camera.qy"], qz = fSettings["Trobot_camera.qz"], qw = fSettings["Trobot_camera.qw"],
          tx = fSettings["Trobot_camera.tx"], ty = fSettings["Trobot_camera.ty"], tz = fSettings["Trobot_camera.tz"];
    mT_basefootprint_cam = Converter::Quation2CvMat(qx, qy, qz, qw, tx, ty, tz );

    qx = fSettings["Tworld_camera.qx"], qy = fSettings["Tworld_camera.qy"], qz = fSettings["Tworld_camera.qz"], qw = fSettings["Tworld_camera.qw"],
    tx = fSettings["Tworld_camera.tx"], ty = fSettings["Tworld_camera.ty"], tz = fSettings["Tworld_camera.tz"];
    mT_world_cam = Converter::Quation2CvMat(qx, qy, qz, qw, tx, ty, tz );

    mT_world_initbaselink  = mT_world_cam * mT_basefootprint_cam.inv();

    double factor = fSettings["NBV_Angle_correct_factor"];
    mNBV_Angle_correct = factor * M_PI;
    mCandidate_num_topub = fSettings["Candidate_num_topub"];
    down_nbv_height = fSettings["Trobot_camera.down_nbv_height"];
    mMaxPlaneHeight = fSettings["Plane.Height.Max"];
    mMinPlaneHeight = fSettings["Plane.Height.Min"];
    mbPubNavGoal = fSettings["PubNavGoal"];
    mTfDuration = fSettings["MAM.TfDuration"];

}

void NbvGenerator::Run() {
    while(1)
    {
        vector<MapPlane *> vpMPlanes = mpMap->GetAllMapPlanes();
        vector<Object_Map*> ObjectMaps = mpMap->GetObjects();
        //1. 计算全局候选点
        ExtractCandidates(vpMPlanes);

        //2. 旋转并评估 全局候选点
        cv::Mat BestCandidate;
        for(int i=0; i<mvGlobalCandidate.size(); i++ ){
            auto globalCandidate_source = mvGlobalCandidate[i];
            //旋转180度
            vector<Candidate> globalCandidate = RotateCandidates(globalCandidate_source);
            //计算视点的评价函数，对condidate筛选
            for(auto candidate : globalCandidate){
                computeReward(candidate, ObjectMaps);
            }
            //从大到小排序localCandidate
            std::sort(globalCandidate.begin(), globalCandidate.end(), [](Candidate a, Candidate b)->bool { return a.reward > b.reward; });
            //将最佳角度值, 修改回GlobalCandidate[i]
            mvGlobalCandidate[i].reward = globalCandidate.begin()->reward;
            mvGlobalCandidate[i].pose = globalCandidate.begin()->pose.clone();
        }

        //3. 将全局NBV发送给导航模块
        if( mvGlobalCandidate.size() != 0 )
        {
            //从大到小排序GlobalCandidate, 队首是NBV
            std::sort(mvGlobalCandidate.begin(), mvGlobalCandidate.end(),
                      [](Candidate a, Candidate b) -> bool { return a.reward > b.reward; });
            NBV = *mvGlobalCandidate.begin();

            //可视化
            PublishPlanes();

            //发布桌面边缘的候选点, 发送给movebase导航目标点和rviz可视化
            PublishGlobalNBVRviz(mvGlobalCandidate);

            //Global NBV:  将候选点的第一个,发送为movebase的目标点
            if(mbPubNavGoal){
                cv::Mat Twc = mvGlobalCandidate.front().pose;
                cv::Mat T_w_baselink = Twc * mT_basefootprint_cam.inv();

                while(!mActionlib->waitForServer(ros::Duration(5.0))){
                    ROS_INFO("Waiting for the move_base action server to come up");
                }
                move_base_msgs::MoveBaseGoal ActionGoal;
                ActionGoal.target_pose.header.frame_id = "map";
                ActionGoal.target_pose.header.stamp = ros::Time::now();
                ActionGoal.target_pose.pose.position.x = T_w_baselink.at<float>(0,3);
                ActionGoal.target_pose.pose.position.y = T_w_baselink.at<float>(1,3);
                ActionGoal.target_pose.pose.position.y = 0.0;
                Eigen::Quaterniond q = Converter::ExtractQuaterniond(T_w_baselink);
                ActionGoal.target_pose.pose.orientation.w = q.w();
                ActionGoal.target_pose.pose.orientation.x = q.x();
                ActionGoal.target_pose.pose.orientation.y = q.y();
                ActionGoal.target_pose.pose.orientation.z = q.z();
                mActionlib->sendGoal(ActionGoal);

                //4. 在导航过程中，生成Local NBV:
                while(!mActionlib->waitForResult(ros::Duration(0.5))){
                    publishLocalNBV();
                }

                //5.导航结束后，让机器人扭头到最佳角度
                if(mActionlib->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                    ROS_INFO("The robot successed to reach the Global goal!");
                else
                    ROS_INFO("The robot failed to reach the Global goal!");
                publishLocalNBV();
            }
            else{
                //if(mMAM_turn == MAM_neckdelay)
                //    publishLocalNBV();
                //else if(mMAM_turn == 0)
                //    mMAM_turn == MAM_neckdelay;
                //else
                //    mMAM_turn --;
                publishLocalNBV();
                usleep( 0.5 * 1000);
            }


        }

        usleep(10*1000);

    }
}

void  NbvGenerator::ExtractCandidates(const vector<MapPlane *> &vpMPs){
    mvGlobalCandidate.clear();
    mvPlanes_filter.clear();
    mvCloudBoundary.clear();

    if (vpMPs.empty())
        return;
    //降维过滤器
    pcl::VoxelGrid<PointT> voxel;
    voxel.setLeafSize(0.002, 0.002, 0.002);

    int num=0; //平面的数量
    for (auto pMP : vpMPs)  //对vpMPs中每个平面pMP分别进行处理,
    {
        // 计算平面与地面的夹角(cos值), 如果夹角很小,则认为水平面. 可以显示
        cv::Mat groud = (cv::Mat_<float>(3, 1) << 0, 0, 1);  ;
        cv::Mat pMP_normal = pMP->GetWorldPos();
        float angle = groud.at<float>(0, 0) * pMP_normal.at<float>(0, 0) +
                      groud.at<float>(1, 0) * pMP_normal.at<float>(1, 0) +
                      groud.at<float>(2, 0) * pMP_normal.at<float>(2, 0);
        //cout<< "垂直夹角angle:"<<angle<<std::endl;
        if ((angle < 0.2) && (angle > -0.2))
            continue;

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
        //cout<< "mean z1:"<<mean_z<<std::endl;
        // 桌面高度太小，忽略
        if(mean_z>mMaxPlaneHeight || mean_z<mMinPlaneHeight)
            continue;
        //cout<< "mean z2:"<<mean_z<<std::endl;
        pcl::getMeanStd(vec_x, mean_x, stddev_x);
        pcl::getMeanStd(vec_y, mean_y, stddev_y);

        //用于plane的展示
        mvPlanes_filter.push_back(tmp);

        //计算tmp的边缘点
        //(1)将tmp转为no_color
        pcl::PointCloud<pcl::PointXYZ>::Ptr  nocolored_pcl_ptr (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*tmp, *nocolored_pcl_ptr);
        //(2)经纬线扫描法
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boundary(new pcl::PointCloud<pcl::PointXYZ>);
        BoundaryExtraction(nocolored_pcl_ptr, cloud_boundary, 200);
        mvCloudBoundary.push_back(cloud_boundary);

        //计算candidates
        double safe_radius = 0.5;
        int divide = 20;
        int step = floor( cloud_boundary->points.size() / divide);

        for(int i=0; i< divide; i++){
            int index = i*step+ divide/2 ;
            //计算候选点的xy的坐标
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

            //计算xy指向中心的坐标
            cv::Mat view = (cv::Mat_<float>(3, 1) << mean_x-x, mean_y-y, 0);
            double angle = atan( (mean_y-y)/(mean_x-x) );
            if( (mean_x-x)<0 && (mean_y-y)>0 )
                angle = angle +  M_PI;
            if( (mean_x-x)<0 && (mean_y-y)<0 )
                angle = angle -  M_PI;
            Eigen::AngleAxisd rotation_vector (angle + mNBV_Angle_correct, Eigen::Vector3d(0,0,1));
            Eigen::Matrix3d rotation_matrix = rotation_vector.toRotationMatrix();
            cv::Mat rotate_mat = Converter::toCvMat(rotation_matrix);
            //cv::Mat t_mat = (cv::Mat_<float>(3, 1) << x, y, -1.0 * down_nbv_height);
            cv::Mat t_mat = (cv::Mat_<float>(3, 1) << x, y, 0);
            cv::Mat T_world_to_baselink = cv::Mat::eye(4, 4, CV_32F);
            rotate_mat.copyTo(T_world_to_baselink.rowRange(0, 3).colRange(0, 3));
            t_mat.copyTo(T_world_to_baselink.rowRange(0, 3).col(3));
            cv::Mat Camera_mat = T_world_to_baselink * mT_basefootprint_cam;

            Candidate candidate;
            candidate.pose = Camera_mat;
            mvGlobalCandidate.push_back(candidate);
        }
        // 桌面高度的 水平面的数量
        num++;
    }

    //cout << "-------" << endl;
    //cout << "桌面高度的水平面的数量: " <<num << endl << endl;
}

vector<Candidate>  NbvGenerator::RotateCandidates(Candidate& initPose){
    vector<Candidate> cands;
    cv::Mat T_w_cam = initPose.pose;   //初始的相机在世界的坐标
    cv::Mat T_w_body = cv::Mat::eye(4, 4, CV_32F); T_w_body = T_w_cam * mT_basefootprint_cam.inv() ;  //初始的机器人底盘在世界的坐标
    cv::Mat T_w_body_new;    //旋转之后的机器人位姿
    for(int i=0; i<=mDivide; i++){
            double angle = M_PI/mDivide * i - M_PI/2.0 ;
            //旋转
            Eigen::AngleAxisd rotation_vector (angle, Eigen::Vector3d(0,0,1));
            Eigen::Matrix3d rotation_matrix = rotation_vector.toRotationMatrix();  //分别加45度
            //Eigen::Isometry3d trans_matrix;
            //trans_matrix.rotate(rotation_vector);
            //trans_matrix.pretranslate(Vector3d(0,0,0));
            cv::Mat rotate_mat = Converter::toCvMat(rotation_matrix);

            //平移
            cv::Mat t_mat = (cv::Mat_<float>(3, 1) << 0, 0, 0);

            //总变换矩阵
            cv::Mat trans_mat = cv::Mat::eye(4, 4, CV_32F);
            rotate_mat.copyTo(trans_mat.rowRange(0, 3).colRange(0, 3));
            t_mat.copyTo(trans_mat.rowRange(0, 3).col(3));

            T_w_body_new = T_w_body * trans_mat;   //旋转之后的机器人位姿
            T_w_cam = T_w_body_new * mT_basefootprint_cam;   //旋转之后的相机位姿
            Candidate temp;
            temp.pose = T_w_cam;
            cands.push_back(temp);
        }
    return  cands;
}

void  NbvGenerator::PublishPlanes()
{
    // color.
    std::vector<vector<float> > colors_bgr{ {135,0,248},  {255,0,253},  {4,254,119},  {255,126,1},  {0,112,255},  {0,250,250}   };

    //plane
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr  colored_pcl_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    colored_pcl_ptr->points.clear();
    int index=0;
    for(auto tmp : mvPlanes_filter){
        index++;
        vector<float> color = colors_bgr[index%6];
        //tmp转为pcl::PointXYZRGB, 才能显示颜色
        for (int i = 0; i <  tmp->points.size(); i++)
        {
          pcl::PointXYZRGB  p;
          p.x=tmp->points[i].x;
          p.y=tmp->points[i].y;
          p.z=tmp->points[i].z;
          p.r = color[2];
          p.g = color[1];
          p.b = color[0];
          colored_pcl_ptr->points.push_back(p);
        }
    }

    //plane的边界
    index=2;
    for(auto cloud_boundary : mvCloudBoundary){
        index++;
        vector<float> color = colors_bgr[index % 6];
        for (int i = 0; i <  cloud_boundary->points.size(); i++)
        {
          pcl::PointXYZ pno = cloud_boundary->points[i];
          pcl::PointXYZRGB  p;
          p.x= pno.x;
          p.y= pno.y;
          p.z= pno.z;
          p.r = color[2];
          p.g = color[1];
          p.b = color[0];
          colored_pcl_ptr->points.push_back(p);
        }
    }
    // 发布平面点和边缘点
    sensor_msgs::PointCloud2 colored_msg;
    colored_pcl_ptr->width = 1;
    colored_pcl_ptr->height = colored_pcl_ptr->points.size();
    pcl::toROSMsg( *colored_pcl_ptr,  colored_msg);  //将点云转化为消息才能发布
    colored_msg.header.frame_id = MAP_FRAME_ID;//帧id改成和velodyne一样的
    pubCloud.publish( colored_msg); //发布调整之后的点云数据，主题为/adjustd_cloud
}


void NbvGenerator::BoundaryExtraction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boundary, int resolution)
    {   // BoundaryExtraction(cloud, cloud_boundary, 200);
        pcl::PointXYZ px_min = *std::min_element(cloud->begin(), cloud->end(), [](pcl::PointXYZ pt1, pcl::PointXYZ pt2) {return pt1.x < pt2.x; });
        pcl::PointXYZ px_max = *std::max_element(cloud->begin(), cloud->end(), [](pcl::PointXYZ pt1, pcl::PointXYZ pt2) {return pt1.x < pt2.x; });

        float delta_x = (px_max.x - px_min.x) / resolution;
        float min_y = INT_MAX, max_y = -INT_MAX;
        std::vector<int> indexs_x(2 * resolution);
        std::vector<std::pair<float, float>> minmax_x(resolution, { INT_MAX,-INT_MAX });
        for (size_t i = 0; i < cloud->size(); ++i)
        {
            int id = (cloud->points[i].x - px_min.x) / delta_x;
            if (cloud->points[i].y < minmax_x[id].first)
            {
                minmax_x[id].first = cloud->points[i].y;
                indexs_x[id] = i;
            }
            else if (cloud->points[i].y > minmax_x[id].second)
            {
                minmax_x[id].second = cloud->points[i].y;
                indexs_x[id + resolution] = i;
            }
        }

        pcl::PointXYZ py_min = *std::min_element(cloud->begin(), cloud->end(), [](pcl::PointXYZ pt1, pcl::PointXYZ pt2) {return pt1.y < pt2.y; });
        pcl::PointXYZ py_max = *std::max_element(cloud->begin(), cloud->end(), [](pcl::PointXYZ pt1, pcl::PointXYZ pt2) {return pt1.y < pt2.y; });

        float delta_y = (py_max.y - py_min.y) / resolution;
        float min_x = INT_MAX, max_x = -INT_MAX;
        std::vector<int> indexs_y(2 * resolution);
        std::vector<std::pair<float, float>> minmax_y(resolution, { INT_MAX,-INT_MAX });
        for (size_t i = 0; i < cloud->size(); ++i)
        {
            int id = (cloud->points[i].y - py_min.y) / delta_y;
            if (cloud->points[i].x < minmax_y[id].first)
            {
                minmax_y[id].first = cloud->points[i].x;
                indexs_y[id] = i;
            }
            else if (cloud->points[i].x > minmax_y[id].second)
            {
                minmax_y[id].second = cloud->points[i].x;
                indexs_y[id + resolution] = i;
            }
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xboundary(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*cloud, indexs_x, *cloud_xboundary);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_yboundary(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*cloud, indexs_y, *cloud_yboundary);
        *cloud_boundary = *cloud_xboundary + *cloud_yboundary;
    }



void NbvGenerator::PublishGlobalNBVRviz(const vector<Candidate> &candidates)
{
    if(candidates.empty())
        return ;

    // color.
    std::vector<vector<float> > colors_bgr{ {255,0,0},  {255,125,0},  {255,255,0 },  {0,255,0 },    {0,0,255},  {0,255,255},  {255,0,255},  {0,0,0}    };

    mCandidate.points.clear();

    for(int i=0; i < mCandidate_num_topub/*candidates.size()*/; i++)
    {
        //cv::Mat Tcw = Tcws[i];
        vector<float> color ;
        //if(i<7)
        //    color = colors_bgr[i];
        //else
        //    color = colors_bgr[7];

        float d = 0.1;
        //Camera is a pyramid. Define in camera coordinate system
        cv::Mat o = (cv::Mat_<float>(4, 1) << 0, 0, 0, 1);
        cv::Mat p1 = (cv::Mat_<float>(4, 1) << d, d * 0.8, d * 0.5, 1);
        cv::Mat p2 = (cv::Mat_<float>(4, 1) << d, -d * 0.8, d * 0.5, 1);
        cv::Mat p3 = (cv::Mat_<float>(4, 1) << -d, -d * 0.8, d * 0.5, 1);
        cv::Mat p4 = (cv::Mat_<float>(4, 1) << -d, d * 0.8, d * 0.5, 1);

        cv::Mat Twc = candidates[i].pose;//Tcw.inv();
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

        mCandidate.points.push_back(msgs_o);
        mCandidate.points.push_back(msgs_p1);
        mCandidate.points.push_back(msgs_o);
        mCandidate.points.push_back(msgs_p2);
        mCandidate.points.push_back(msgs_o);
        mCandidate.points.push_back(msgs_p3);
        mCandidate.points.push_back(msgs_o);
        mCandidate.points.push_back(msgs_p4);
        mCandidate.points.push_back(msgs_p1);
        mCandidate.points.push_back(msgs_p2);
        mCandidate.points.push_back(msgs_p2);
        mCandidate.points.push_back(msgs_p3);
        mCandidate.points.push_back(msgs_p3);
        mCandidate.points.push_back(msgs_p4);
        mCandidate.points.push_back(msgs_p4);
        mCandidate.points.push_back(msgs_p1);

        mCandidate.id= mtest ++;
        mCandidate.lifetime = ros::Duration(1);
        mCandidate.header.stamp = ros::Time::now();
        mCandidate.color.r=float(colors_bgr[i][0]/255.0);
        mCandidate.color.b=float(colors_bgr[i][1]/255.0);
        mCandidate.color.g=float(colors_bgr[i][2]/255.0);
        mCandidate.color.a=1.0f;
        mCandidate.scale.x=fPointSize;
        mCandidate.scale.y=fPointSize;
        publisher_candidate.publish(mCandidate);
        //mCandidate.color.r=0.0f;
        //mCandidate.color.b=0.0f;
        //mCandidate.color.g=0.0f;
        //publisher_candidate_unsort.publish(mCandidate);

    }
}

void NbvGenerator::publishLocalNBV(){

    //1.构建地图，并向mam中添加地图点。 TODO：也应该添加 物体。
    map_data MapData;
    double theta_interval;
    vector<MapPoint*> vpPts = mpMap->GetAllMapPoints();
    for(size_t i=0; i<vpPts.size(); i++){
        if(vpPts[i]->isBad())
            continue;

        float minDist = vpPts[i]->GetMinDistanceInvariance();  //根据金字塔计算出的最近可见距离
        float maxDist = vpPts[i]->GetMaxDistanceInvariance();  //根据金字塔计算出的最远可见距离
        float foundRatio = vpPts[i]->GetFoundRatio(); //地图点的查找率,如果过低,会被标记为bad point

        //float Dist = sqrt((Tsc_curr.at<float>(0,3) - vpPts[i]->GetWorldPos().at<float>(0))*(Tsc_curr.at<float>(0,3) - vpPts[i]->GetWorldPos().at<float>(0))+
        //(Tsc_curr.at<float>(1,3) - vpPts[i]->GetWorldPos().at<float>(1))*(Tsc_curr.at<float>(1,3) - vpPts[i]->GetWorldPos().at<float>(2))+
        //(Tsc_curr.at<float>(2,3) - vpPts[i]->GetWorldPos().at<float>(2))*(Tsc_curr.at<float>(2,3) - vpPts[i]->GetWorldPos().at<float>(2)));

        //if((Dist > maxDist)||(Dist < minDist))
        //    continue;

        MapData.MapPointsCoordinate.push_back(std::vector<double>{vpPts[i]->GetWorldPos().at<float>(0), vpPts[i]->GetWorldPos().at<float>(1), vpPts[i]->GetWorldPos().at<float>(2)});

        //观测角度(共视方向)的冗余量,论文中的alpha_max
        if(vpPts[i]->theta_std * 2.5 < 10.0/57.3){  //如果标准差小于..., 则设置冗余量为10弧度
            theta_interval = 10.0/57.3;
        }else{
            theta_interval = vpPts[i]->theta_std * 2.5;
        }

        MapData.UB.push_back(double(vpPts[i]->theta_mean + theta_interval));    //观测角度(共视方向)的最大值
        MapData.LB.push_back(double(vpPts[i]->theta_mean - theta_interval));    //观测角度(共视方向)的最小值
        MapData.maxDist.push_back(double(maxDist));         //最远可见距离
        MapData.minDist.push_back(double(minDist));         //最近可见距离
        MapData.foundRatio.push_back(double(foundRatio));   //地图点的查找率
    }

    //2.构建mam camera对象
    camera camera_model(MapData, 20);    // zhang 这里的阈值20对于我的实验环境是不是 有点高??
    camera_model.setCamera(mfx, mfy, mcx, mcy,  mImageWidth,  mImageHeight, mMax_dis, mMin_dis );

    //3.获取机器人在map下的坐标T_w_basefootprint
    tf::TransformListener listener;
    tf::StampedTransform transform;
    cv::Mat T_w_basefootprint = cv::Mat::eye(4,4,CV_32F);
    try
    {
        listener.waitForTransform("/map", "/base_footprint", ros::Time(0), ros::Duration(mTfDuration));
        listener.lookupTransform("/map", "/base_footprint", ros::Time(0), transform);
        T_w_basefootprint = Converter::Quation2CvMat(
                        transform.getRotation().x(),
                        transform.getRotation().y(),
                        transform.getRotation().z(),
                        transform.getRotation().w(),
                        transform.getOrigin().x(),
                        transform.getOrigin().y(),
                        0.0  //transform.getOrigin().x(),
                );
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s -->> lost tf from /map to /base_footprint",ex.what());
    }

    //4.计算扭头的最佳角度
    if(!T_w_basefootprint.empty()){

        if(MAM_isused){
            //visible_info VI;
            int visible_pts = 0;
            double great_angle = -5.0;

            //利用速度模型, 生成下一时刻的机器人位姿

            //在机器人位姿的基础上，计算最佳扭头的角度
            cv::Mat body_new;

            for(int i=0; i<=mDivide; i++){
                double angle = mLocalNeckRange/mDivide * i - mLocalNeckRange/2.0 ;
                //旋转
                Eigen::AngleAxisd rotation_vector (angle, Eigen::Vector3d(0,0,1));
                Eigen::Matrix3d rotation_matrix = rotation_vector.toRotationMatrix();  //分别加45度
                //Eigen::Isometry3d trans_matrix;
                //trans_matrix.rotate(rotation_vector);
                //trans_matrix.pretranslate(Vector3d(0,0,0));
                cv::Mat rotate_mat = Converter::toCvMat(rotation_matrix);

                //平移
                cv::Mat t_mat = (cv::Mat_<float>(3, 1) << 0, 0, 0);

                //总变换矩阵
                cv::Mat trans_mat = cv::Mat::eye(4, 4, CV_32F);
                rotate_mat.copyTo(trans_mat.rowRange(0, 3).colRange(0, 3));
                t_mat.copyTo(trans_mat.rowRange(0, 3).col(3));

                body_new = T_w_basefootprint * trans_mat;   //旋转之后的机器人位姿
                cv::Mat T_w_cam = body_new * mT_basefootprint_cam;   //旋转之后的相机位姿
                int num = camera_model.countVisible(T_w_cam);   //利用相机模型， 计算最佳的扭头角度
                cout<<"--agl:" << angle/M_PI*180<<",num:"<<num;
                if(num > visible_pts){
                    great_angle = angle;
                    visible_pts = num;
                }
                localCandidate lc;
                lc.angle = angle;
                lc.num = num;

                //double localreward = num + ;
            }
            //unique_lock<mutex> lock(mMutexMamAngle);
            mGreat_angle = great_angle;//   /M_PI*180 ;
            //cout << "----number of points predicted=" << visible_pts <<", angle:"<<mGreat_angle/M_PI*180 << endl;
        }
        else{
            //double desk_center_x = -2.0;
            //double desk_center_y = 0.0;
            //double direct_x = transform.getOrigin().x() - desk_center_x;
            //double direct_y = transform.getOrigin().y() - desk_center_y;
            //
            cv::Mat desk_center_w = cv::Mat::zeros(4,1,CV_32F);
            desk_center_w.at<float>(0,0) = -2.0;
            desk_center_w.at<float>(1,0) = 0.0;
            desk_center_w.at<float>(2,0) = 0.0;
            desk_center_w.at<float>(3,0) = 1.0;
            cv::Mat desk_center_robot = T_w_basefootprint.inv() * desk_center_w;
            double angle = atan2( desk_center_robot.at<float>(1,0),  desk_center_robot.at<float>(0,0) );  //与x轴的夹角
            mGreat_angle = angle;
            cout << "---- angle:" <<mGreat_angle/M_PI*180 << endl;

        }

        //5.发布脖子的角度
        publishNeckAngle(mGreat_angle);

        //6.可视化local nbv
        {
            geometry_msgs::PoseWithCovarianceStamped mampose;
            mampose.pose.pose.position.x = T_w_basefootprint.at<float>(0, 3);
            mampose.pose.pose.position.y = T_w_basefootprint.at<float>(1, 3);
            mampose.pose.pose.position.z = 0.0;

            Eigen::Quaterniond q_w_body = Converter::ExtractQuaterniond(T_w_basefootprint);
            Eigen::Quaterniond q_body_rotate = Eigen::Quaterniond( Eigen::AngleAxisd( mGreat_angle*M_PI/180.0, Eigen::Vector3d ( 0,0,1 ) )  );     //沿 Z 轴旋转 45 度
            Eigen::Quaterniond q = q_w_body * q_body_rotate;
            mampose.pose.pose.orientation.w = q.w();
            mampose.pose.pose.orientation.x = q.x();
            mampose.pose.pose.orientation.y = q.y();
            mampose.pose.pose.orientation.z = q.z();
            mampose.header.frame_id= "map";
            mampose.header.stamp=ros::Time::now();

            publisher_mam_rviz.publish(mampose);
        }
    }

}

void NbvGenerator::publishNeckAngle(double  angle){
    //if(angle==0)
    //    return;
    std_msgs::Float64 msg;
    msg.data =  angle;
    publisher_mam.publish(msg);
}
void NbvGenerator::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

double NbvGenerator::computeCosAngle(cv::Mat &candidate, cv::Mat &objectPose, Eigen::Vector3d &ie){
    Eigen::Vector3d view(   objectPose.at<float>(0,3)-candidate.at<float>(0,3),
                            objectPose.at<float>(1,3)-candidate.at<float>(1,3),
                            objectPose.at<float>(2,3)-candidate.at<float>(2,3)      );
    double cos = view.dot(ie) / (view.norm() * ie.norm()); //角度cos值
    return cos;
}

void NbvGenerator::computeReward(Candidate &candidate, vector<Object_Map*> obj3ds){
    double reward = 0;
    for (int i = 0; i < (int)obj3ds.size(); i++) {
        Object_Map *obj3d = obj3ds[i];
        bool viewed = obj3d->WheatherInRectFrameOf(candidate.pose, mfx, mfy, mcx, mcy, mImageWidth, mImageHeight);
        if(viewed){
            double CosAngle = computeCosAngle(candidate.pose, obj3d->mCuboid3D.pose_mat, obj3d->mMainDirection);
            // reward = 概率统计置信度 + 信息熵
            reward += obj3d->mStatistics * CosAngle * obj3d->mIE;
        }
    }
    candidate.reward = reward;
}


};

