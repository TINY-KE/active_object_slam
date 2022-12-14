//
// Created by zhjd on 11/17/22.
//

#include "NbvGenerator.h"

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
    mdivide = fSettings["MAM.divide"];

    //坐标关系,用于生成
    float qx = fSettings["Trobot_camera.qx"], qy = fSettings["Trobot_camera.qy"], qz = fSettings["Trobot_camera.qz"], qw = fSettings["Trobot_camera.qw"],
                tx = fSettings["Trobot_camera.tx"], ty = fSettings["Trobot_camera.ty"], tz = fSettings["Trobot_camera.tz"];
    //mT_body_cam = cv::Mat::eye(4, 4, CV_32F);
    //Eigen::Quaterniond quaternion(Eigen::Vector4d(qx, qy, qz, qw));
    //Eigen::AngleAxisd rotation_vector(quaternion);
    //Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    //T.rotate(rotation_vector);
    //T.pretranslate(Eigen::Vector3d(tx, ty, tz));
    //Eigen::Matrix4d GroundtruthPose_eigen = T.matrix();
    //cv::Mat cv_mat_32f;
    //cv::eigen2cv(GroundtruthPose_eigen, cv_mat_32f);
    //cv_mat_32f.convertTo(mT_body_cam, CV_32F);
    //新方法：
    mT_body_cam = Converter::Quation2CvMat(qx, qy, qz, qw, tx, ty, tz );
    down_nbv_height = fSettings["Trobot_camera.down_nbv_height"];
}

void NbvGenerator::Run() {
    while(1)
    {
        vector<MapPlane *> vpMPlanes = mpMap->GetAllMapPlanes();
        vector<Object_Map*> ObjectMaps = mpMap->GetObjects();
        //计算全局候选点
        ExtractCandidates(vpMPlanes);

        //计算局部候选点
        cv::Mat BestCandidate;
        for(int i=0; i<mvGlobalCandidate.size(); i++ ){
            auto globalCandidate = mvGlobalCandidate[i];
            //旋转180度
            vector<Candidate> localCandidate = RotateCandidates(globalCandidate);
            //计算视点的评价函数，对condidate筛选
            for(auto candidate : localCandidate){
                computeReward(candidate, ObjectMaps);
            }
            //从大到小排序localCandidate
            std::sort(localCandidate.begin(), localCandidate.end(), [](Candidate a, Candidate b)->bool { return a.reward > b.reward; });
            //将最佳角度值, 修改回GlobalCandidate[i]
            mvGlobalCandidate[i].reward = localCandidate.begin()->reward;
            mvGlobalCandidate[i].pose = localCandidate.begin()->pose.clone();
        }


        if( mvGlobalCandidate.size() != 0 )
        {
            //从大到小排序GlobalCandidate, 队首是NBV
            std::sort(mvGlobalCandidate.begin(), mvGlobalCandidate.end(),
                      [](Candidate a, Candidate b) -> bool { return a.reward > b.reward; });
            NBV = *mvGlobalCandidate.begin();

            //利用速度模型, 生成局部nbv


            //可视化
            PublishPlanesAndCamera();
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
        cout<< "mean z1:"<<mean_z<<std::endl;
        // 桌面高度太小，忽略
        if(mean_z>1.7 || mean_z<0.2)
            continue;
        cout<< "mean z2:"<<mean_z<<std::endl;
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
            Eigen::AngleAxisd rotation_vector (angle, Eigen::Vector3d(0,0,1));
            Eigen::Matrix3d rotation_matrix = rotation_vector.toRotationMatrix();
            cv::Mat rotate_mat = Converter::toCvMat(rotation_matrix);
            cv::Mat t_mat = (cv::Mat_<float>(3, 1) << x, y, -1.0 * down_nbv_height);
            cv::Mat PoseFootprint_mat = cv::Mat::eye(4, 4, CV_32F);
            rotate_mat.copyTo(PoseFootprint_mat.rowRange(0, 3).colRange(0, 3));
            t_mat.copyTo(PoseFootprint_mat.rowRange(0, 3).col(3));
            cv::Mat PoseCamera_mat = PoseFootprint_mat * mT_body_cam;

            Candidate candidate;
            candidate.pose = PoseCamera_mat;
            mvGlobalCandidate.push_back(candidate);
        }
        // 桌面高度的 水平面的数量
        num++;
    }

    cout << "-------" << endl;
    cout << "桌面高度的水平面的数量: " <<num << endl << endl;
}

vector<Candidate>  NbvGenerator::RotateCandidates(Candidate& initPose){
    vector<Candidate> cands;
    cv::Mat T_w_cam = initPose.pose;   //初始的相机在世界的坐标
    cv::Mat T_w_body = cv::Mat::eye(4, 4, CV_32F); T_w_body = T_w_cam * mT_body_cam.inv() ;  //初始的机器人底盘在世界的坐标
    cv::Mat T_w_body_new;    //旋转之后的机器人位姿
    for(int i=0; i<=mdivide; i++){
            double angle = M_PI/mdivide * i - M_PI/2.0 ;
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
            T_w_cam = T_w_body_new * mT_body_cam;   //旋转之后的相机位姿
            Candidate temp;
            temp.pose = T_w_cam;
            cands.push_back(temp);
        }
    return  cands;
}

void  NbvGenerator::PublishPlanesAndCamera()
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

    //发布桌面边缘的候选点
    PublishCamera(mvGlobalCandidate);
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



void NbvGenerator::PublishCamera(const vector<Candidate> &candidates)
{
    if(candidates.empty())
        return ;

    // color.
    std::vector<vector<float> > colors_bgr{ {255,0,0},  {255,125,0},  {255,255,0 },  {0,255,0 },    {0,0,255},  {0,255,255},  {255,0,255},  {0,0,0}    };

    mCandidate.points.clear();

    for(int i=0; i < 7/*candidates.size()*/; i++)
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

