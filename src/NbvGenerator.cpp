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
    publisher_object_backgroud = nh.advertise<visualization_msgs::Marker>("objectmap_backgroud", 1000);
    publisher_candidate = nh.advertise<visualization_msgs::Marker>("candidate", 1000);
    publisher_candidate_unsort = nh.advertise<visualization_msgs::Marker>("candidate_unsort", 1000);
    publisher_nbv = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    publisher_mam = nh.advertise<std_msgs::Float64>("/neck/neck_controller/command", 10);
    publisher_mam_rviz = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/local_nbv", 1000);
    mActionlib = new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>("move_base", true);
    sub_reachgoal = nh.subscribe("/reach_goal" , 1, &NbvGenerator::SubReachGoal, this);
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

    //mT_world_initbaselink  = mT_world_cam * mT_basefootprint_cam.inv();

    double factor = fSettings["NBV_Angle_correct_factor"];
    mNBV_Angle_correct = factor * M_PI;
    mCandidate_num_topub = fSettings["Candidate_num_topub"];
    down_nbv_height = fSettings["Trobot_camera.down_nbv_height"];
    mMaxPlaneHeight = fSettings["Plane.Height.Max"];
    mMinPlaneHeight = fSettings["Plane.Height.Min"];
    mMinPlaneSafeRadius = fSettings["Planle.Safe_radius"];
    mGobalCandidateNum = fSettings["Planle.Gobal_Candidate_Num"];
    mbPubGlobalGoal = fSettings["PubGlobalGoal"];
    mbPubLocalGoal = fSettings["PubLocalGoal"];
    mTfDuration = fSettings["MAM.TfDuration"];
    mReward_dis = fSettings["MAM.Reward_dis"];
    mReward_angle_cost = fSettings["MAM.Reward_angle_cost"];
    mBackgroudObjectNum = fSettings["IE.BackgroudObjectNum"];
    mbFakeBackgroudObjects = fSettings["FakeBackgroudObjects"];
    mnObserveMaxNumBackgroudObject = fSettings["ObserveMaxNumBackgroudObject"];

    Candidate origin_pose;
    origin_pose.pose = mT_world_cam;
    mNBVs_old.push_back(origin_pose);
    cv::Mat clonedMat = mT_world_cam.clone();
    mpMap->mvNBVs_pose.push_back(clonedMat);
}
void NbvGenerator::SubReachGoal(const std_msgs::Bool::ConstPtr & msg){
    mbReachGoalFlag = msg->data;
}
void NbvGenerator::Run() {
    while(1)
    {
        std::cerr << std::endl << "开始 一轮 NBV generator " << std::endl;
        //1.获取地图中的平面和物体。 并在每次global nbv计算前， 计算一遍所有物体的主方向
        vector<MapPlane *> vpMPlanes = mpMap->GetAllMapPlanes();
        vector<Object_Map*> ForegroundObjectMaps = mpMap->GetObjects();

        mvGlobalCandidate.clear();
        for(auto obj: ForegroundObjectMaps)
            obj->ComputeMainDirection();


        //2.
        //（1）筛选背景物体。并将他们加入到mvBackgroud_objects。
        //（2）计算背景物体的评价值, 并判断是否要  结束此背景物体的aive mapping
        //（3）从第一个Bo周围 挑选NBV
        //（4）可视化背景物体和NBV
        //v1:
        //Filter_BackgroudObjects_and_Extract_Candidates(vpMPlanes, ForegroundObjectMaps);
        //PublishBackgroudObjects();  //可视化 筛选后的平面
        //v2:
        std::cerr << "地图中平面的数量为" << vpMPlanes.size() << std::endl;
        if(mbFakeBackgroudObjects)
            Fake_BackgroudObjects(vpMPlanes, ForegroundObjectMaps, mbFakeBackgroudObjects);
        else
            Filter_BackgroudObjects(vpMPlanes, ForegroundObjectMaps);

        if (mvBackgroud_objects.empty()){
            std::cerr << "其中背景物体的数量为：0" << std::endl;
            continue;
        }
        else {
            std::cout << "其中背景物体的数量为："<< mvBackgroud_objects.size()  << std::endl;
            if(mvBackgroud_objects.size() > 1)
                std::sort(mvBackgroud_objects.begin(), mvBackgroud_objects.end(), [](const BackgroudObject* obj1, const BackgroudObject* obj2) {
                    return obj1->mnId < obj2->mnId;
                });

            BackgroudObject* bo_first = new BackgroudObject();
            bool init_bo_first = false;
            for(int i=0; i<mvBackgroud_objects.size(); i++){
                if(!mvBackgroud_objects[i]->return_end_ASLAM()){
                    bo_first = mvBackgroud_objects[i];
                    init_bo_first = true;
                    std::cerr << "选中背景物体【"<<i<<"】,开始生成NBV。x:"<<bo_first->mean_x<<", y:"<<bo_first->mean_y <<", ObserveNum:"<<bo_first->mnObserveNum<<", State:"<<bo_first->mState<< std::endl;
                    break;
                }
            }

            if(init_bo_first){
                //std::cerr << "等待Extract_Candidates" << std::endl;
                Extract_Candidates(bo_first);
            }
            else if( mvBackgroud_objects.size() == mBackgroudObjectNum){
                std::cerr << "结束建图，两个桌子都建立完毕了"<< std::endl;
                break;

            }
            else{
                std::cerr << "没有找到用于生成候选点的背景物体" << ", mvBackgroud_objects.size():" << mvBackgroud_objects.size()
                << ", mBackgroudObjectNum:" << mBackgroudObjectNum << std::endl;
            }
        }


        PublishBackgroudObjects_and_SupportingPlane();  //可视化 筛选后的平面


        //2. 旋转（旋转暂时停用）并评估 全局候选点
        cv::Mat BestCandidate;

        for(int i=0; i<mvGlobalCandidate.size(); i++ ){
            auto globalCandidate_source = mvGlobalCandidate[i];
            //version1: 旋转180
            ////旋转180度
            //vector<Candidate> globalCandidate = RotateCandidates(globalCandidate_source);
            ////计算视点的评价函数，对condidate筛选
            //for(auto candidate : globalCandidate){
            //    computeReward(candidate, ObjectMaps);
            //}
            ////从大到小排序localCandidate
            //std::sort(globalCandidate.begin(), globalCandidate.end(), [](Candidate a, Candidate b)->bool { return a.reward > b.reward; });
            ////将最佳角度值, 修改回GlobalCandidate[i]
            //mvGlobalCandidate[i].reward = globalCandidate.front().reward;
            //mvGlobalCandidate[i].pose = globalCandidate.front().pose.clone();

            //version2： 只处理面向中心的原位置
            computeReward(mvGlobalCandidate[i]); //  , ForegroundObjectMaps
            //ROS_INFO("final test reward:%f", mvGlobalCandidate[i].reward);
        }

        //3. 将全局NBV发送给导航模块
        if( mvGlobalCandidate.size() != 0 )
        {
            //从大到小排序GlobalCandidate, 队首是NBV
            std::sort(mvGlobalCandidate.begin(), mvGlobalCandidate.end(),
                      [](Candidate a, Candidate b) -> bool { return a.reward > b.reward; });
            //NBV = *mvGlobalCandidate.begin();

            //桌面边缘的候选点,rviz可视化
            PublishGlobalNBVRviz(mvGlobalCandidate);

            //Global NBV:  将候选点的第一个,发送为movebase的目标点
            if(mbPubGlobalGoal){
                auto Gnbv = mvGlobalCandidate.front();
                cv::Mat Twc = Gnbv.pose.clone();
                cv::Mat T_w_baselink = Twc * mT_basefootprint_cam.inv();

                while(!mActionlib->waitForServer(ros::Duration(5.0))){
                    ROS_INFO("Waiting for the move_base action server to come up");
                }
                move_base_msgs::MoveBaseGoal ActionGoal;
                ActionGoal.target_pose.header.frame_id = "map";
                ActionGoal.target_pose.header.stamp = ros::Time::now();
                ActionGoal.target_pose.pose.position.x = T_w_baselink.at<float>(0,3);
                ActionGoal.target_pose.pose.position.y = T_w_baselink.at<float>(1,3);
                ActionGoal.target_pose.pose.position.z = 0.0;
                Eigen::Quaterniond q = Converter::ExtractQuaterniond(T_w_baselink);
                ActionGoal.target_pose.pose.orientation.w = q.w();
                ActionGoal.target_pose.pose.orientation.x = q.x();
                ActionGoal.target_pose.pose.orientation.y = q.y();
                ActionGoal.target_pose.pose.orientation.z = q.z();
                mActionlib->sendGoal(ActionGoal);
                ROS_INFO("The robot try to reach the Global goal x:%f, y:%f",ActionGoal.target_pose.pose.position.x ,ActionGoal.target_pose.pose.position.y);

                //4. 在导航过程中，生成Local NBV:
                while(!mActionlib->waitForResult(ros::Duration(0.1))){
                    publishLocalNBV(Gnbv);
                    vector<Candidate> nbvs = {mvGlobalCandidate.front()};
                    PublishGlobalNBVRviz(nbvs);
                }

                //5.导航结束后，让机器人扭头到最佳角度。如果到达了nbv，则将它存入nbvs_old
                if(mActionlib->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                    ROS_INFO("The robot successed to reach the Global goal x:%f, y:%f",ActionGoal.target_pose.pose.position.x ,ActionGoal.target_pose.pose.position.y );
                    mNBVs_old.push_back(Gnbv);
                    cv::Mat clonedMat = Gnbv.pose.clone();
                    mpMap->mvNBVs_pose.push_back(clonedMat);
                }
                else
                    ROS_INFO("The robot failed to reach the Global goal x:%f, y:%f",ActionGoal.target_pose.pose.position.x ,ActionGoal.target_pose.pose.position.y);
                publishLocalNBV(Gnbv);
            }

            else{
                // v1: 等待键盘输入
                //while (true) {
                //    int reach_goal = std::cin.peek(); //非阻塞
                //    //int reach_goal = getchar(); //阻塞
                //
                //    //如果没有输入可用，std::cin.peek() 会返回 EOF（End-of-File）的值，通常是 -1。因此，在使用 std::cin.peek() 时，需要将返回值与 EOF 进行比较，以判断是否有输入可用。
                //    //如果输入流中有可用字符，std::cin.peek() 会返回该字符的 ASCII 值。您可以将其与具体的字符进行比较，或者将其转换为 char 类型进行处理。
                //    //使用 std::cin.peek() 时，需要注意输入缓冲区的状态。如果输入缓冲区中已经有字符，但尚未读取，那么 std::cin.peek() 会立即返回缓冲区中的下一个字符，而不会等待用户输入。但如果输入缓冲区为空，那么 std::cin.peek() 可能会等待用户输入。
                //    //if (reach_goal != EOF)
                //    if(reach_goal == '\n')
                //    {
                //        ROS_INFO("接收到回车" );
                //        break;
                //    }
                //    else{
                //        ROS_INFO("等待回车" );
                //        vector<Candidate> nbvs = {mvGlobalCandidate.front()};
                //        PublishGlobalNBVRviz(nbvs);
                //        usleep( 0.1 * 1000);
                //    }
                //
                //}

                //v2:
                ros::Rate loop_rate(2);
                while(true){
                    //用于my NBV方法。系统自动选择相机直视的背景物体。
                    if(mbPubLocalGoal==1){
                        if(mbReachGoalFlag){
                            ROS_INFO("到达NBV" );
                            mbReachGoalFlag = false;
                            mNBVs_old.push_back(mvGlobalCandidate.front());
                            publishLocalNBV(mvGlobalCandidate.front());
                            cv::Mat clonedMat = mvGlobalCandidate.front().pose.clone();
                            mpMap->mvNBVs_pose.push_back(clonedMat);
                            //记录当前BO生成NBV的数量，
                            std::cerr << "更新背景物体的观测次数，x:"<<mvGlobalCandidate.front().bo->mean_x<<", y:"<<mvGlobalCandidate.front().bo->mean_y <<", ObserveNum:"<<mvGlobalCandidate.front().bo->mnObserveNum<< std::endl;
                            mvGlobalCandidate.front().bo->mnObserveNum = mvGlobalCandidate.front().bo->mnObserveNum + 1;
                            std::cerr << "新ObserveNum:"<<mvGlobalCandidate.front().bo->mnObserveNum<< std::endl;
                            break;

                        } else{
                            ROS_INFO("未到达NBV" );
                            vector<Candidate> nbvs = {mvGlobalCandidate.front()};
                            PublishGlobalNBVRviz(nbvs);
                            publishLocalNBV(mvGlobalCandidate.front());
                            //usleep( 0.5 * 1000);
                            ros::spinOnce();
                            loop_rate.sleep();
                        }
                    }
                    //用于cover方法。相机直视 人指定的物体。
                    else if(mbPubLocalGoal==2){
                        if(mbReachGoalFlag){
                            ROS_INFO("到达NBV" );
                            mbReachGoalFlag = false;
                            Candidate Gnbv;
                            //说明当前BO已经观测关闭，因此Lnbv指向下一个BO。
                            ++ mFakeLocalNum;
                            Gnbv.bo = mvBackgroud_objects[mFakeLocalNum];
                            publishLocalNBV(Gnbv);
                            break;
                        } else{
                            ROS_INFO("未到达NBV" );
                            Candidate Gnbv;
                            Gnbv.bo = mvBackgroud_objects[mFakeLocalNum];
                            publishLocalNBV(Gnbv);
                            ros::spinOnce();
                            loop_rate.sleep();
                        }
                    }
                    // 用于 frontier exploration。相机不转。
                    //也可以用于GNBV的刷新与查看
                    else if(mbPubLocalGoal==0){
                        ROS_INFO("frontier exploration,不发布NBV" );
                        vector<Candidate> nbvs = {mvGlobalCandidate.front()};
                        PublishGlobalNBVRviz(nbvs);
                        ros::spinOnce();
                        loop_rate.sleep();
                    }
                }

            }

            //计算所属的背景物体的评价值


        }

        //usleep(10*1000);
        std::cerr << "一轮 NBV generator 结束" << std::endl;
    }
}


// 提取候选平面（mvPlanes_filter）
void  NbvGenerator::Filter_BackgroudObjects(const vector<ORB_SLAM2::MapPlane *> &vpMPls,  const vector<Object_Map*> &ForegroundObjectMaps ) {
    //清空 NBV提取器和地图中的 背景物体。通过本程序重新生成。
    mvPlanes_filter.clear();
    mvBackgroud_objects.clear();
    //mpMap->ClearBackgroudObjects();

    if (vpMPls.empty())
        return;

    for (auto pMP : vpMPls)  //对vpMPs中每个平面pMP分别进行处理,
    {
        // 计算平面与地面的夹角(cos值), 如果夹角很小,则认为水平面. 可以显示
        cv::Mat groud = (cv::Mat_<float>(3, 1) << 0, 0, 1);  ;
        cv::Mat pMP_normal = pMP->GetWorldPos();
        float angle = groud.at<float>(0, 0) * pMP_normal.at<float>(0, 0) +
                      groud.at<float>(1, 0) * pMP_normal.at<float>(1, 0) +
                      groud.at<float>(2, 0) * pMP_normal.at<float>(2, 0);
        //cout<< "垂直夹角angle:"<<angle<<std::endl;
        if ((angle < 0.2) && (angle > -0.2)){
            //cout<< "不是水平面:"<<angle<<std::endl;
            continue;
        }

        //计算当前平面,在各关键帧中对应的平面
        map<KeyFrame *, int> observations = pMP->GetObservations();  //std::map<KeyFrame*, int> mObservations;
        //std::cout << "Map size: " << observations.size() << std::endl;
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
        //cout<< "[debug] 将各关键帧中的平面,融合为一个allCloudPoints"<<std::endl;

        //对allCloudPoints降维成tmp
        PointCloud::Ptr tmp(new PointCloud());


        //计算点云的最小值和最大值    void getMinMax3D (const pcl::PointCloud<PointT> &cloud, PointT &min_pt, PointT &max_pt)

        // 计算allCloudPoint的中心点
        vector<double> vec_x,vec_y,vec_z;
        //cout<< "[debug] 计算allCloudPoint的中心点 0:"<<allCloudPoints->points.size()<<std::endl;
        for (size_t i = 0; i < allCloudPoints->points.size(); i++)
        {
            vec_x.push_back(allCloudPoints->points[i].x);
            vec_y.push_back(allCloudPoints->points[i].y);
            vec_z.push_back(allCloudPoints->points[i].z);
        }
        //cout<< "[debug] 计算allCloudPoint的中心点 1"<<std::endl;

        std::vector<float> vec_z_float(vec_z.size());
        std::transform(vec_z.begin(), vec_z.end(), vec_z_float.begin(),
                   [](double value) { return static_cast<float>(value); });
        //cout<< "[debug] 计算allCloudPoint的中心点 2"<<std::endl;

        //计算桌面高度
        double mean_z;	//点云均值
	    double stddev_z;	//点云标准差
        pcl::getMeanStd(vec_z_float, mean_z, stddev_z);
        //cout<< "mean z1:"<<mean_z<<std::endl;
        // 桌面高度太小，忽略
        if(mean_z>mMaxPlaneHeight || mean_z<mMinPlaneHeight)
            continue;
        //cout<< "mean z2:"<<mean_z<<std::endl;
        //cout<< "[debug] 计算allCloudPoint的中心点 3"<<std::endl;

        //计算点云的均值，但是没用上
        //double mean_x,mean_y,mean_z;	//点云均值
	    //double stddev_x,stddev_y,stddev_z;	//点云标准差
        //pcl::getMeanStd(vec_x, mean_x, stddev_x);
        //pcl::getMeanStd(vec_y, mean_y, stddev_y);


        // 支撑平面
        mvPlanes_filter.push_back(tmp);
        publishSupportingPlane(allCloudPoints);

        //  计算点云的最小值和最大值
        Eigen::Vector4f minPoint;
        Eigen::Vector4f maxPoint;
        pcl::PointXYZ minPt, maxPt;

        float x_min = std::numeric_limits<float>::max();
        float y_min = std::numeric_limits<float>::max();
        float z_min = std::numeric_limits<float>::max();
        float x_max = -std::numeric_limits<float>::max();
        float y_max = -std::numeric_limits<float>::max();
        float z_max = -std::numeric_limits<float>::max();

        for (const auto& pt : allCloudPoints->points) {
            if (pt.x < x_min) x_min = pt.x;
            if (pt.y < y_min) y_min = pt.y;
            if (pt.z < z_min) z_min = pt.z;
            if (pt.x > x_max) x_max = pt.x;
            if (pt.y > y_max) y_max = pt.y;
            if (pt.z > z_max) z_max = pt.z;
        }

        minPoint << x_min, y_min, z_min, 0.0f;
        maxPoint << x_max, y_max, z_max, 0.0f;

        BackgroudObject* bo = new BackgroudObject();
        //pcl::copyPointCloud( *allCloudPoints, *(bo->mPlane) );   //todo:  问题在哪？
        bo->max_x = x_max;
        bo->max_y = y_max;
        bo->max_z = z_max;

        bo->min_x = x_min;
        bo->min_y = y_min;
        bo->min_z = 0.0;

        bo->mean_x = (x_max + x_min) / 2;
        bo->mean_y = (y_max + y_min) / 2;
        bo->mean_z = (z_max + 0.0) / 2;

        bo->length = x_max - x_min;
        bo->width = y_max - y_min;
        bo->height = z_max - 0.0;

        // Rotation matrix.
        bo->computePose();
        //cout<< "[debug] 计算allCloudPoint的中心点 4"<<std::endl;

        bo->mnId = pMP->mnId;

        for(auto p: tmp->points){
            bo->mPlane->points.push_back(p);
        }

        bo->IncludeFOs_and_WheatherEndActive(ForegroundObjectMaps);

        mvBackgroud_objects.push_back(bo);
        mpMap->AddBackgroudObject(bo);
    }

}

void  NbvGenerator::Fake_BackgroudObjects(const vector<ORB_SLAM2::MapPlane *> &vpMPls,  const vector<Object_Map*> &ForegroundObjectMaps , const int fake_num  ) {

    if(mvBackgroud_objects.empty()){
        //清空 NBV提取器和地图中的 背景物体。通过本程序重新生成。
        mvPlanes_filter.clear();
        mvBackgroud_objects.clear();
        mpMap->ClearBackgroudObjects();
        double x,y,z,width,length,depth;
        if(fake_num==1)
        {
            BackgroudObject* bo = new BackgroudObject();
            //pcl::copyPointCloud( *allCloudPoints, *(bo->mPlane) );   //todo:  问题在哪？
            x= -1.936606,y=-0.0614115 ,z=0.250644;
            length=1.804500   ,width=1.600000 ,depth=0.721900;

            bo->max_x = x+length/2.0;
            bo->max_y = y+width/2.0;
            bo->max_z = z+depth/2.0;

            bo->min_x = x-length/2.0;
            bo->min_y = y-width/2.0;
            bo->min_z = 0.0;

            bo->mean_x = x;
            bo->mean_y = y;
            bo->mean_z = z;

            bo->length = length;
            bo->width = width;
            bo->height = depth;

            // Rotation matrix.
            bo->computePose();
            //cout<< "[debug] 计算allCloudPoint的中心点 4"<<std::endl;

            bo->mnId = 1;

            bo->mnObserveMaxNum = this->mnObserveMaxNumBackgroudObject;

            bo->IncludeFOs_and_WheatherEndActive(ForegroundObjectMaps);

            mvBackgroud_objects.push_back(bo);
            mpMap->AddBackgroudObject(bo);
        }
        if(fake_num==2)
        {
            //桌子1
            BackgroudObject* bo = new BackgroudObject();
            //pcl::copyPointCloud( *allCloudPoints, *(bo->mPlane) );   //todo:  问题在哪？
            x= -1.936606,y=-0.0614115 ,z=0.250644;
            length=1.804500   ,width=1.600000 ,depth=0.721900;

            bo->max_x = x+length/2.0;
            bo->max_y = y+width/2.0;
            bo->max_z = z+depth/2.0;

            bo->min_x = x-length/2.0;
            bo->min_y = y-width/2.0;
            bo->min_z = 0.0;

            bo->mean_x = x;
            bo->mean_y = y;
            bo->mean_z = z;

            bo->length = length;
            bo->width = width;
            bo->height = depth;

            // Rotation matrix.
            bo->computePose();
            //cout<< "[debug] 计算allCloudPoint的中心点 4"<<std::endl;

            bo->mnId = 1;

            bo->mnObserveMaxNum = this->mnObserveMaxNumBackgroudObject;

            bo->IncludeFOs_and_WheatherEndActive(ForegroundObjectMaps);

            mvBackgroud_objects.push_back(bo);
            mpMap->AddBackgroudObject(bo);


            //桌子2
            BackgroudObject* bo2 = new BackgroudObject();
            //pcl::copyPointCloud( *allCloudPoints, *(bo->mPlane) );   //todo:  问题在哪？
            //x=-2.28485+0.549443/2.0 ,y=1.90387+0.721389/2.0 ,z=0.447185;
            //width=0.721389 ,length=0.549443 ,depth=0.894369;
            x=-2.039965 ,y=2.194572 ,z=0.455522 ;
            length=1.096666 ,width=0.729113 ,depth=0.911044;
            //-2.039965 2.194572 0.455522     0.000000 0.000000 0.000000 1.000000     1.096666 0.729113 0.911044
            //Position: x=-2.28485 ,y=1.90387 ,z=0.447185
            //Orientation: roll=0 ,pitch=-0 ,yaw=0
            //Size: width=0.721389 ,length=0.549443 ,depth=0.894369

            bo2->max_x = x+length/2.0;
            bo2->max_y = y+width/2.0;
            bo2->max_z = z+depth/2.0;

            bo2->min_x = x-length/2.0;
            bo2->min_y = y-width/2.0;
            bo2->min_z = 0.0;

            bo2->mean_x = x;
            bo2->mean_y = y;
            bo2->mean_z = z;

            bo2->length = length;
            bo2->width = width;
            bo2->height = depth;

            // Rotation matrix.
            bo2->computePose();
            //cout<< "[debug] 计算allCloudPoint的中心点 4"<<std::endl;

            bo2->mnId = 2;

            bo2->mnObserveMaxNum = this->mnObserveMaxNumBackgroudObject;

            bo2->IncludeFOs_and_WheatherEndActive(ForegroundObjectMaps);

            mvBackgroud_objects.push_back(bo2);
            mpMap->AddBackgroudObject(bo2);
        }

        if(fake_num==3){
            //-1.617100 0.178400 0.381000     0.000000 0.000000 0.000000 1.000000     0.558800 1.828799 0.762000 #dining table餐桌  %desk_drawer
            //-3.752570 -0.653044 0.392500     0.000000 0.000000 0.000000 1.000000     0.913000 0.913000 0.785000 #dining table餐桌  %cafe_table
            //-3.744747 1.197421 0.433114     0.000000 0.000000 0.395462 0.918482     0.640972 0.635368 0.866227 #mouse鼠标  %chair_2
            //桌子1
            BackgroudObject* bo = new BackgroudObject();
            x=-1.617100,y= 0.178400  ,z=0.381000;
            length=0.558800   ,width=1.828799 ,depth=0.762000;

            bo->max_x = x+length/2.0;
            bo->max_y = y+width/2.0;
            bo->max_z = z+depth/2.0;

            bo->min_x = x-length/2.0;
            bo->min_y = y-width/2.0;
            bo->min_z = 0.0;

            bo->mean_x = x;
            bo->mean_y = y;
            bo->mean_z = z;

            bo->length = length;
            bo->width = width;
            bo->height = depth;

            // Rotation matrix.
            bo->computePose();
            //cout<< "[debug] 计算allCloudPoint的中心点 4"<<std::endl;

            bo->mnId = 1;

            bo->mnObserveMaxNum = this->mnObserveMaxNumBackgroudObject;

            bo->IncludeFOs_and_WheatherEndActive(ForegroundObjectMaps);

            mvBackgroud_objects.push_back(bo);
            mpMap->AddBackgroudObject(bo);


            //桌子2：
            BackgroudObject* bo2 = new BackgroudObject();
            //pcl::copyPointCloud( *allCloudPoints, *(bo->mPlane) );   //todo:  问题在哪？
            //x=-2.28485+0.549443/2.0 ,y=1.90387+0.721389/2.0 ,z=0.447185;
            //width=0.721389 ,length=0.549443 ,depth=0.894369;
            x=-3.752570 ,y=-0.653044 ,z=0.392500 ;
            length=0.913000 ,width=0.913000 ,depth=0.785000;
            //-2.039965 2.194572 0.455522     0.000000 0.000000 0.000000 1.000000     1.096666 0.729113 0.911044
            //Position: x=-2.28485 ,y=1.90387 ,z=0.447185
            //Orientation: roll=0 ,pitch=-0 ,yaw=0
            //Size: width=0.721389 ,length=0.549443 ,depth=0.894369

            bo2->max_x = x+length/2.0;
            bo2->max_y = y+width/2.0;
            bo2->max_z = z+depth/2.0;

            bo2->min_x = x-length/2.0;
            bo2->min_y = y-width/2.0;
            bo2->min_z = 0.0;

            bo2->mean_x = x;
            bo2->mean_y = y;
            bo2->mean_z = z;

            bo2->length = length;
            bo2->width = width;
            bo2->height = depth;

            // Rotation matrix.
            bo2->computePose();
            //cout<< "[debug] 计算allCloudPoint的中心点 4"<<std::endl;

            bo2->mnId = 2;

            bo2->mnObserveMaxNum = this->mnObserveMaxNumBackgroudObject;

            bo2->IncludeFOs_and_WheatherEndActive(ForegroundObjectMaps);

            mvBackgroud_objects.push_back(bo2);
            mpMap->AddBackgroudObject(bo2);

            //桌子3： //-3.744747 1.197421 0.433114     0.000000 0.000000 0.395462 0.918482     0.640972 0.635368 0.866227 #mouse鼠标  %chair_2
            BackgroudObject* bo3 = new BackgroudObject();
            //pcl::copyPointCloud( *allCloudPoints, *(bo->mPlane) );   //todo:  问题在哪？
            //x=-2.28485+0.549443/2.0 ,y=1.90387+0.721389/2.0 ,z=0.447185;
            //width=0.721389 ,length=0.549443 ,depth=0.894369;
            x=-3.744747 ,y=1.197421 ,z=0.433114 ;
            length=0.640972 ,width=0.635368 ,depth=0.866227;
            //-2.039965 2.194572 0.455522     0.000000 0.000000 0.000000 1.000000     1.096666 0.729113 0.911044
            //Position: x=-2.28485 ,y=1.90387 ,z=0.447185
            //Orientation: roll=0 ,pitch=-0 ,yaw=0
            //Size: width=0.721389 ,length=0.549443 ,depth=0.894369

            bo3->max_x = x+length/2.0;
            bo3->max_y = y+width/2.0;
            bo3->max_z = z+depth/2.0;

            bo3->min_x = x-length/2.0;
            bo3->min_y = y-width/2.0;
            bo3->min_z = 0.0;

            bo3->mean_x = x;
            bo3->mean_y = y;
            bo3->mean_z = z;

            bo3->length = length;
            bo3->width = width;
            bo3->height = depth;

            // Rotation matrix.
            bo3->computePose(-1*M_PI_4);
            //cout<< "[debug] 计算allCloudPoint的中心点 4"<<std::endl;

            bo3->mnId = 3;

            bo3->mnObserveMaxNum = this->mnObserveMaxNumBackgroudObject;

            bo3->IncludeFOs_and_WheatherEndActive(ForegroundObjectMaps);

            mvBackgroud_objects.push_back(bo3);
            mpMap->AddBackgroudObject(bo3);

        }
    }
    else{
        for(auto BO:mvBackgroud_objects){
            BO->IncludeFOs_and_WheatherEndActive(ForegroundObjectMaps);
        }
    }
}

// 提取候选平面
void  NbvGenerator::Extract_Candidates( BackgroudObject* bo_first ) {
    std::cerr << "开始Extract_Candidates" << std::endl;

    mvGlobalCandidate.clear();
    mvCloudBoundary.clear();

    //计算candidates
    //version1:
    //double safe_radius = mMinPlaneSafeRadius;
    //double divide = mGobalCandidateNum; //36
    //double step = 2* M_PI / divide;  //floor( 2* M_PI / divide); //10
    //for(int i=0; i< divide; i++){
    //    // 射线起点和方向
    //    //std::cout << "bo_first->mean_x, mean_y: "<<std::endl;
    //    //std::cout << bo_first->mean_x<<",  " << bo_first->mean_y  << std::endl;
    //    cv::Point2f rayOrigin(bo_first->mean_x, bo_first->mean_y);
    //    float rayAngle = ( i+0.5)*step /* 射线的角度，以弧度表示 */;
    //    //std::cout << "2 bo_first->mean_x, mean_y: "<<bo_first->mean_x<<",  " << bo_first->mean_y  << std::endl;
    //
    //    // 矩形边框信息
    //    cv::Point2f rectTopLeft(bo_first->mean_x - bo_first->length/2, bo_first->mean_y + bo_first->width/2);
    //    cv::Point2f rectBottomRight(bo_first->mean_x + bo_first->length/2, bo_first->mean_y - bo_first->width/2);
    //
    //    // 计算射线与矩形边框的交点
    //    cv::Point2f SafeNBVPoint_best;
    //    bool hasIntersection = false;
    //
    //    // 计算矩形的四条边
    //    cv::Point2f rectTopRight(rectBottomRight.x, rectTopLeft.y);
    //    cv::Point2f rectBottomLeft(rectTopLeft.x, rectBottomRight.y);
    //
    //    // 计算射线与每条边的交点
    //    cv::Point2f SafeNBVPoint;
    //    int type = 0;
    //    // 射线与右边相交
    //    if ( rayAngle>=0 && rayAngle<=M_PI/2.0 && computeIntersection(rayOrigin, cv::Point2f(rectBottomRight.x, rayOrigin.y+std::tan(rayAngle)*bo_first->length/2.0 ), rectBottomRight, rectTopRight, SafeNBVPoint))
    //    {
    //        type=1;
    //        SafeNBVPoint_best = SafeNBVPoint;
    //        hasIntersection = true;
    //    }
    //    if ( rayAngle>=M_PI*3.0/2.0 && rayAngle<=2.0*M_PI && computeIntersection(rayOrigin, cv::Point2f(rectBottomRight.x, rayOrigin.y+std::tan(rayAngle)*bo_first->length/2.0 ), rectBottomRight, rectTopRight, SafeNBVPoint))
    //    {
    //        type=1;
    //        SafeNBVPoint_best = SafeNBVPoint;
    //        hasIntersection = true;
    //    }
    //
    //    // 射线与上边相交
    //    else if (rayAngle>=0 && rayAngle<=M_PI && computeIntersection(rayOrigin, cv::Point2f(rayOrigin.x - std::tan(rayAngle-M_PI/2.0)*bo_first->width/2.0, rectTopRight.y), rectTopRight, rectTopLeft, SafeNBVPoint))
    //    {
    //        type=2;
    //        SafeNBVPoint_best = SafeNBVPoint;
    //        hasIntersection = true;
    //    }
    //    else if (rayAngle>=0 && rayAngle<=M_PI && computeIntersection(rayOrigin, cv::Point2f(rayOrigin.x - std::tan(rayAngle-M_PI/2.0)*bo_first->width/2.0, rectTopRight.y), rectTopRight, rectTopLeft, SafeNBVPoint))
    //    {
    //        type=2;
    //        SafeNBVPoint_best = SafeNBVPoint;
    //        hasIntersection = true;
    //    }
    //
    //    // 射线与左边相交
    //    else if (rayAngle>=M_PI/2.0 && rayAngle<=M_PI*3.0/2.0 && computeIntersection(rayOrigin, cv::Point2f( rectTopLeft.x, rayOrigin.y - std::tan(rayAngle-M_PI)*bo_first->length/2.0), rectTopLeft, rectBottomLeft, SafeNBVPoint))
    //    {
    //        type=3;
    //        SafeNBVPoint_best = SafeNBVPoint;
    //        hasIntersection = true;
    //    }
    //
    //    // 射线与下边相交
    //    else if (rayAngle>=M_PI && rayAngle<=M_PI*2 && computeIntersection(rayOrigin, cv::Point2f(rayOrigin.x + std::tan(rayAngle-3.0*M_PI/2.0)*bo_first->width/2.0, rectBottomLeft.y), rectBottomLeft, rectBottomRight, SafeNBVPoint))
    //    {
    //        type=4;
    //        SafeNBVPoint_best = SafeNBVPoint;
    //        hasIntersection = true;
    //    }
    //    //std::cout<<"[computeIntersection],num:"<<i<<", type:"<<type<<", angle:"<<rayAngle<<", coordinate:"<<SafeNBVPoint.x <<", "<<SafeNBVPoint.y<<std::endl;
    //    //// 打印交点坐标（如果存在交点）
    //    //if (hasIntersection)
    //    //{
    //    //    std::cout << "交点坐标：" << SafeNBVPoint_best.x << ", " << SafeNBVPoint_best.y << std::endl;
    //    //
    //    //}
    //    //else
    //    //{
    //    //    std::cout << "射线没有与矩形相交" << std::endl;
    //    //}
    //
    //
    //
    //
    //    ////计算候选点的xy的坐标
    //    double x = SafeNBVPoint_best.x;
    //    double y = SafeNBVPoint_best.y;
    //
    //    ////计算xy指向中心的坐标
    //    //cv::Mat view = (cv::Mat_<float>(3, 1) << bo_first->mean_x-x, bo_first->mean_y-y, 0);
    //    //double angle = atan( (bo_first->mean_y-y)/(bo_first->mean_x-x) );
    //    //if( (bo_first->mean_x-x)<0 && (bo_first->mean_y-y)>0 )
    //    //    angle = angle +  M_PI;
    //    //if( (bo_first->mean_x-x)<0 && (bo_first->mean_y-y)<0 )
    //    //    angle = angle -  M_PI;
    //    //Eigen::AngleAxisd rotation_vector (angle + mNBV_Angle_correct, Eigen::Vector3d(0,0,1));
    //    //Eigen::Matrix3d rotation_matrix = rotation_vector.toRotationMatrix();
    //    //cv::Mat rotate_mat = Converter::toCvMat(rotation_matrix);
    //    ////cv::Mat t_mat = (cv::Mat_<float>(3, 1) << x, y, -1.0 * down_nbv_height);
    //    //cv::Mat t_mat = (cv::Mat_<float>(3, 1) << x, y, 0);
    //    //cv::Mat T_world_to_baselink = cv::Mat::eye(4, 4, CV_32F);
    //    //rotate_mat.copyTo(T_world_to_baselink.rowRange(0, 3).colRange(0, 3));
    //    //t_mat.copyTo(T_world_to_baselink.rowRange(0, 3).col(3));
    //    //cv::Mat Camera_mat = T_world_to_baselink * mT_basefootprint_cam;
    //
    //    // 计算向量v，从NBV指向物体中心。计算角度θ，即向量v与x轴正方向之间的夹角
    //    cv::Point2f v =  cv::Point2f(bo_first->mean_x,bo_first->mean_y) - SafeNBVPoint_best;
    //    float angle = std::atan2(v.y, v.x);
    //
    //    //转换成  变换矩阵
    //    Eigen::AngleAxisd rotation_vector (angle + mNBV_Angle_correct, Eigen::Vector3d(0,0,1));
    //    Eigen::Matrix3d rotation_matrix = rotation_vector.toRotationMatrix();
    //    cv::Mat rotate_mat = Converter::toCvMat(rotation_matrix);
    //    cv::Mat t_mat = (cv::Mat_<float>(3, 1) << x, y, 0);
    //    cv::Mat T_world_to_baselink = cv::Mat::eye(4, 4, CV_32F);
    //    rotate_mat.copyTo(T_world_to_baselink.rowRange(0, 3).colRange(0, 3));
    //    t_mat.copyTo(T_world_to_baselink.rowRange(0, 3).col(3));
    //    cv::Mat Camera_mat = T_world_to_baselink * mT_basefootprint_cam;
    //
    //
    //
    //    Candidate candidate;
    //    candidate.pose = Camera_mat.clone();
    //    candidate.bo = bo_first;
    //    mvGlobalCandidate.push_back(candidate);
    //
    //}


    // version2:
    double divide = mGobalCandidateNum; //36
    int length_num_half = ceil(divide * (bo_first->length / (bo_first->length + bo_first->width))  /2.0 /2.0);
    int width_num_half = ceil(divide * (bo_first->width / (bo_first->length + bo_first->width))  /2.0 /2.0);

    // 矩形边框的四个中心
    cv::Point2f BOCentor(bo_first->mean_x, bo_first->mean_y);
    cv::Point2f LeftCentor(bo_first->mean_x - bo_first->length/2, bo_first->mean_y);
    cv::Point2f RightCentor(bo_first->mean_x + bo_first->length/2, bo_first->mean_y);
    cv::Point2f TopCentor(bo_first->mean_x , bo_first->mean_y + bo_first->width/2);
    cv::Point2f BottomCentor(bo_first->mean_x , bo_first->mean_y - bo_first->width/2);

    //存储所有交点
    std::vector<cv::Point2f> AllIntersections;
    //上下两边
    for(int i=0; i<length_num_half; i++){
        double length_segment = bo_first->length/2.0   / (length_num_half+0.5);
        cv::Point2f intersection_1( TopCentor.x + length_segment*i , TopCentor.y);
        cv::Point2f intersection_2( TopCentor.x - length_segment*i , TopCentor.y);
        cv::Point2f intersection_3( BottomCentor.x + length_segment*i , BottomCentor.y);
        cv::Point2f intersection_4( BottomCentor.x - length_segment*i , BottomCentor.y);
        AllIntersections.push_back(intersection_1);
        AllIntersections.push_back(intersection_2);
        AllIntersections.push_back(intersection_3);
        AllIntersections.push_back(intersection_4);
    }
    //左右两边
    for(int i=0; i<width_num_half; i++){
        double width_segment = bo_first->width/2.0   / (width_num_half+0.5);
        cv::Point2f intersection_1( LeftCentor.x, LeftCentor.y + width_segment*i );
        cv::Point2f intersection_2( LeftCentor.x , LeftCentor.y - width_segment*i);
        cv::Point2f intersection_3( RightCentor.x , RightCentor.y + width_segment*i);
        cv::Point2f intersection_4( RightCentor.x, RightCentor.y - width_segment*i );
        AllIntersections.push_back(intersection_1);
        AllIntersections.push_back(intersection_2);
        AllIntersections.push_back(intersection_3);
        AllIntersections.push_back(intersection_4);
    }


    for(int i=0; i< AllIntersections.size(); i++){

        // 计算安全交点
        cv::Point2f ray = AllIntersections[i] - BOCentor;
        cv::Point2f ray_norm = normalize(ray);
        float ray_length = std::sqrt(ray.x * ray.x + ray.y * ray.y);
        cv::Point2f SafeNBVPoint = (ray_length+mMinPlaneSafeRadius)*ray_norm + BOCentor;

        ////计算候选点的xy的坐标
        double x = SafeNBVPoint.x;
        double y = SafeNBVPoint.y;


        // 计算向量v，从NBV指向物体中心。计算角度θ，即向量v与x轴正方向之间的夹角
        cv::Point2f v =  cv::Point2f(bo_first->mean_x,bo_first->mean_y) - SafeNBVPoint;
        float angle = std::atan2(v.y, v.x);

        //转换成  变换矩阵
        Eigen::AngleAxisd rotation_vector (angle + mNBV_Angle_correct, Eigen::Vector3d(0,0,1));
        Eigen::Matrix3d rotation_matrix = rotation_vector.toRotationMatrix();
        cv::Mat rotate_mat = Converter::toCvMat(rotation_matrix);
        cv::Mat t_mat = (cv::Mat_<float>(3, 1) << x, y, 0);
        cv::Mat T_world_to_baselink = cv::Mat::eye(4, 4, CV_32F);
        rotate_mat.copyTo(T_world_to_baselink.rowRange(0, 3).colRange(0, 3));
        t_mat.copyTo(T_world_to_baselink.rowRange(0, 3).col(3));
        cv::Mat Camera_mat = T_world_to_baselink * mT_basefootprint_cam;

        Candidate candidate;
        candidate.pose = Camera_mat.clone();
        candidate.bo = bo_first;
        mvGlobalCandidate.push_back(candidate);
    }
}


cv::Point2f NbvGenerator::normalize(cv::Point2f ray_n  ) {
    float norm = std::sqrt(ray_n.x * ray_n.x + ray_n.y * ray_n.y);

    cv::Point2f ray_norm(ray_n.x / norm, ray_n.y / norm);

    return ray_norm;
}

double  NbvGenerator::IntersectionScale(const cv::Point2f& p1, const cv::Point2f& p1End, const cv::Point2f& p2, const cv::Point2f& p2End){
    //return p1方向上的平移量 t
    //double t = cross((p2 - p1), v2) / cross(v1, v2);

    cv::Point2f v1 = p1End - p1;
    cv::Point2f v2 = p2End - p2;

    double t = normalize(p2 - p1).cross(normalize(v2)) / normalize(v1).cross(normalize(v2));

    return t;
}

bool NbvGenerator::computeIntersection(const cv::Point2f& rayStart, const cv::Point2f& rayEnd, const cv::Point2f& segmentStart, const cv::Point2f& segmentEnd, cv::Point2f& SafeNBVPoint )
{
    //计算方法：https://www.cnblogs.com/zhb2000/p/vector-cross-product-solve-intersection.html#:~:text=%E6%88%91%E4%BB%AC%E7%94%A8%E7%82%B9%20p%201%20%E5%92%8C%E5%90%91%E9%87%8F%20v%20%E2%86%92%201%20%E6%9D%A5%E8%A1%A8%E7%A4%BA%E7%9B%B4%E7%BA%BF,%E2%86%92%202%20%3D%200%20%E2%86%92%20%EF%BC%88%E4%B8%A4%E5%90%91%E9%87%8F%E5%B9%B3%E8%A1%8C%EF%BC%89%E8%BF%99%E4%B8%AA%E5%BC%8F%E5%AD%90%E6%B1%82%E5%87%BA%20t%20%E3%80%82

    //计算射线的方向向量 rayDir 和线段的方向向量
    cv::Point2f rayDir = rayEnd - rayStart;
    cv::Point2f segmentDir = segmentEnd - segmentStart;
    float crossProduct = rayDir.x * segmentDir.y - rayDir.y * segmentDir.x; //crossProduct 表示计算两个向量的叉乘（叉积）。在计算几何中，叉乘是用来确定两个向量之间的垂直关系和方向的一种运算。
                                                                            //叉乘的结果是一个新的向量，它垂直于原来两个向量所在的平面，并且方向根据右手法则确定。在几何中，叉乘常用于计算法向量、确定平面、判断点的位置关系等应用。
    //如果 crossProduct 接近于零，则表示射线和线段平行或共线，没有交点
    if (std::abs(crossProduct) < std::numeric_limits<float>::epsilon())
    {
        // 射线与线段平行或共线
        return false;
    }

    //ray方向上的延长尺度  P = rayStart + t * rayDir
    double t_ray = IntersectionScale(rayStart, rayEnd, segmentStart, segmentEnd);

    //segment上的延长尺度  P = segmentStart + u * segmentDir
    double t_segment = IntersectionScale(segmentStart, segmentEnd, rayStart, rayEnd);

    // segment的尺寸
    double length_segment = sqrt(   (segmentEnd.x-segmentStart.x)*(segmentEnd.x-segmentStart.x)   +  (segmentEnd.y-segmentStart.y)*(segmentEnd.y-segmentStart.y)   );
    //double length_segment = sqrt(   (rayEnd.x-rayStart.x)*(rayEnd.x-rayStart.x)   +  (rayEnd.y-rayStart.y)*(rayEnd.y-rayStart.y)   );

    //判断是否正确
     if (t_ray >= 0 && t_segment >= 0 && t_segment <= length_segment)
    {
        SafeNBVPoint = rayStart + (t_ray + mMinPlaneSafeRadius*t_ray) * normalize(rayEnd - rayStart);
        return true;
    }

    return false;
}


// 提取候选平面（mvPlanes_filter）和候选观测点（mvGlobalCandidate）
void  NbvGenerator::Extract_Candidates(){
    mvGlobalCandidate.clear();
    mvCloudBoundary.clear();

    if (mvBackgroud_objects.empty())
        return;
    std::cerr << "当前背景物体的数量为"<< mvBackgroud_objects.size()  << std::endl;

    //1.先按照背景物体的mnid，排序，从小开始处理
    //在前面完成了

    //2.依次查看vpMPs中每个背景物体，从第一个没完成建图的背景物体周围，提取 候选点
    for (auto bo : mvBackgroud_objects)
    {

        //计算tmp的边缘点
        //(1)将tmp转为no_color
        pcl::PointCloud<pcl::PointXYZ>::Ptr  nocolored_pcl_ptr (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud( *(bo->mPlane), *nocolored_pcl_ptr);
        //(2)经纬线扫描法
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boundary(new pcl::PointCloud<pcl::PointXYZ>);
        BoundaryExtraction(nocolored_pcl_ptr, cloud_boundary, 200);
        //将边缘点存入
        mvCloudBoundary.push_back(cloud_boundary);

        //计算candidates
        double safe_radius = mMinPlaneSafeRadius;
        int divide = mGobalCandidateNum;
        int step = floor( cloud_boundary->points.size() / divide);

        for(int i=0; i< divide; i++){
            int index = i*step+ divide/2 ;
            //计算候选点的xy的坐标
            double x,y;
            double d = sqrt(    (cloud_boundary->points[index].x-bo->mean_x)*(cloud_boundary->points[index].x-bo->mean_x)
                            +   (cloud_boundary->points[index].y-bo->mean_y)*(cloud_boundary->points[index].y-bo->mean_y)
                            )
                       + safe_radius;
            double k = (cloud_boundary->points[index].y-bo->mean_y) /
                     (cloud_boundary->points[index].x-bo->mean_x);

            double x_positive = sqrt(d * d / (1 + k * k) ) + bo->mean_x;
            double x_negative = -sqrt(d * d / (1 + k * k) ) + bo->mean_x;
            if( (x_positive-bo->mean_x)*(cloud_boundary->points[index].x-bo->mean_x) > 0 )
                x = x_positive;
            else
                x = x_negative;
            y = k*(x - bo->mean_x) + bo->mean_y;

            //计算xy指向中心的坐标
            cv::Mat view = (cv::Mat_<float>(3, 1) << bo->mean_x-x, bo->mean_y-y, 0);
            double angle = atan( (bo->mean_y-y)/(bo->mean_x-x) );
            if( (bo->mean_x-x)<0 && (bo->mean_y-y)>0 )
                angle = angle +  M_PI;
            if( (bo->mean_x-x)<0 && (bo->mean_y-y)<0 )
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
            candidate.pose = Camera_mat.clone();
            mvGlobalCandidate.push_back(candidate);
        }

        //只需要处理第一个没完成建图的背景物体周围。因此退出
        break;
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
    std::cerr << "PublishBackgroudObjects_and_SupportingPlane: 平面的数量为"<< vpMPs.size()  << std::endl;
    for (auto pMP : vpMPs)  //对vpMPs中每个平面pMP分别进行处理,
    {
        if(pMP->end_activemapping)
            continue;

        // 计算平面与地面的夹角(cos值), 如果夹角很小,则认为水平面. 可以显示
        cv::Mat groud = (cv::Mat_<float>(3, 1) << 0, 0, 1);  ;
        cv::Mat pMP_normal = pMP->GetWorldPos();
        float angle = groud.at<float>(0, 0) * pMP_normal.at<float>(0, 0) +
                      groud.at<float>(1, 0) * pMP_normal.at<float>(1, 0) +
                      groud.at<float>(2, 0) * pMP_normal.at<float>(2, 0);
        //cout<< "垂直夹角angle:"<<angle<<std::endl;
        if ((angle < 0.2) && (angle > -0.2)){
            continue;
            //cout<< "不是水平面:"<<angle<<std::endl;
        }


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

        //计算点云的最小值和最大值    void getMinMax3D (const pcl::PointCloud<PointT> &cloud, PointT &min_pt, PointT &max_pt)

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
        double safe_radius = mMinPlaneSafeRadius;
        int divide = mGobalCandidateNum;
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
            candidate.pose = Camera_mat.clone();
            mvGlobalCandidate.push_back(candidate);
        }
        // 桌面高度的 水平面的数量
        num++;
    }

    //cout << "-------" << endl;
    cout << "桌面高度的水平面的数量: " <<num << endl << endl;
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

void  NbvGenerator::PublishBackgroudObjects_and_SupportingPlane()
{
    std::cout << "PublishBackgroudObjects_and_SupportingPlane: 平面点云的数量为"<< mvPlanes_filter.size()  << std::endl;
    // color.
    std::vector<vector<float> > colors_bgr{ {135,0,248},  {255,0,253},  {4,254,119},  {255,126,1},  {0,112,255},  {0,250,250}   };

    //发布背景物体的cube
    //publishBackgroudObject(colored_pcl_ptr);
    for(auto bo: mvBackgroud_objects){
        publishBackgroudObject(bo);
    }

}
void NbvGenerator::publishSupportingPlane( pcl::PointCloud<pcl::PointXYZRGB>::Ptr allCloudPoints ){
    // color.
    std::vector<vector<float> > colors_bgr{ {135,0,248},  {255,0,253},  {4,254,119},  {255,126,1},  {0,112,255},  {0,250,250}   };
    //plane
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr  colored_pcl_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    colored_pcl_ptr->points.clear();
    int index=0;
    //for(auto tmp : mvPlanes_filter)

    //auto tmp = allCloudPoints;
    index++;
    vector<float> color = colors_bgr[index%6];
    //tmp转为pcl::PointXYZRGB, 才能显示颜色
    for (int i = 0; i <  allCloudPoints->points.size(); i++)
    {
      pcl::PointXYZRGB  p;
      p.x=allCloudPoints->points[i].x;
      p.y=allCloudPoints->points[i].y;
      p.z=allCloudPoints->points[i].z;
      p.r = static_cast<uint8_t>(color[2]);
      p.g = static_cast<uint8_t>(color[1]);
      p.b = static_cast<uint8_t>(color[0]);

      colored_pcl_ptr->points.push_back(p);
    }


    //plane的边界
    //index=2;
    //for(auto cloud_boundary : mvCloudBoundary){
    //    index++;
    //    vector<float> color = colors_bgr[index % 6];
    //    for (int i = 0; i <  cloud_boundary->points.size(); i++)
    //    {
    //      pcl::PointXYZ pno = cloud_boundary->points[i];
    //      pcl::PointXYZRGB  p;
    //      p.x= pno.x;
    //      p.y= pno.y;
    //      p.z= pno.z;
    //      p.r = color[2];
    //      p.g = color[1];
    //      p.b = color[0];
    //      colored_pcl_ptr->points.push_back(p);
    //    }
    //}

    // 发布平面点和边缘点
    sensor_msgs::PointCloud2 colored_msg;
    colored_pcl_ptr->width = 1;
    colored_pcl_ptr->height = colored_pcl_ptr->points.size();
    pcl::toROSMsg( *colored_pcl_ptr,  colored_msg);  //将点云转化为消息才能发布
    colored_msg.header.frame_id = MAP_FRAME_ID;//帧id改成和velodyne一样的
    pubCloud.publish( colored_msg); //发布调整之后的点云数据，主题为/adjustd_cloud
}

void NbvGenerator::publishBackgroudObject( BackgroudObject* bo ){
    // 定义最小值和最大值
    //float x_min = bo->min_x;
    //float y_min = bo->min_y;
    //float z_min = bo->min_z;
    //float x_max = bo->max_x;
    //float y_max = bo->max_y;
    //float z_max = bo->max_z;
    //float length = bo->length;
    //float width = bo->width;
    //float height = bo->height;
    float x_min = (-0.5)*bo->length;
    float x_max = (0.5)*bo->length;
    float y_min = (-0.5)*bo->width;
    float y_max = (0.5)*bo->width;
    float z_min = (-0.5)*bo->height;
    float z_max = (0.5)*bo->height;

    // Create an object in the map.
    //std::cout<<"可视化背景物体"<<std::endl;
    Object_Map *Object3D = new Object_Map;
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
    Object3D->backgroud_object = true;
    //Eigen::Matrix4d pose = Converter::cvMattoMatrix4d(bo->pose_mat);
    g2o::SE3Quat pose =  Converter::toSE3Quat( bo->pose_mat );
    Object3D->mCuboid3D.corner_1 = pose * Eigen::Vector3d(x_min, y_min, 0);
    Object3D->mCuboid3D.corner_2 = pose * Eigen::Vector3d(x_max, y_min, 0);
    Object3D->mCuboid3D.corner_3 = pose * Eigen::Vector3d(x_max, y_max, 0);
    Object3D->mCuboid3D.corner_4 = pose * Eigen::Vector3d(x_min, y_max, 0);
    Object3D->mCuboid3D.corner_5 = pose * Eigen::Vector3d(x_min, y_min, z_max);
    Object3D->mCuboid3D.corner_6 = pose * Eigen::Vector3d(x_max, y_min, z_max);
    Object3D->mCuboid3D.corner_7 = pose * Eigen::Vector3d(x_max, y_max, z_max);
    Object3D->mCuboid3D.corner_8 = pose * Eigen::Vector3d(x_min, y_max, z_max);

    //(1)物体
    visualization_msgs::Marker marker;
    marker.id = bo->mnId;
    //marker.lifetime = ros::Duration(1);
    marker.header.frame_id= MAP_FRAME_ID;
    marker.header.stamp=ros::Time::now();
    marker.type = visualization_msgs::Marker::LINE_LIST; //LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    //marker.color.r = color[2]/255.0; marker.color.g = color[1]/255.0; marker.color.b = color[0]/255.0; marker.color.a = 1.0;
    //if(bo->return_end_ASLAM() == bo->UnExplored){
    //  marker.color.r = 0.0; marker.color.g = 0.0; marker.color.b = 0.0; marker.color.a = 1.0;
    //}
    //else if(bo->return_end_ASLAM() == bo->UnEnd){
    //  marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0; marker.color.a = 1.0;
    //}
    //else{
    //  marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0; marker.color.a = 1.0;
    //}
    std::vector<vector<float> > colors_bgr{ {135,0,248},  {255,0,253},  {4,254,119},  {255,126,1},  {0,112,255},  {0,250,250}   };
    vector<float> color = colors_bgr[ (bo->mnClass +4) % 6];  //+1还是粉色/红色    +2绿色  +3蓝色  +4橙色   +5黄色
    marker.color.r = color[2]/255.0; marker.color.g = color[1]/255.0; marker.color.b = color[0]/255.0; marker.color.a = 1.0;

    marker.scale.x = 0.01;
    //     8------7
    //    /|     /|
    //   / |    / |
    //  5------6  |
    //  |  4---|--3
    //  | /    | /
    //  1------2
    marker.points.push_back(corner_to_marker(Object3D->mCuboid3D.corner_1));
    marker.points.push_back(corner_to_marker(Object3D->mCuboid3D.corner_2));
    marker.points.push_back(corner_to_marker(Object3D->mCuboid3D.corner_2));
    marker.points.push_back(corner_to_marker(Object3D->mCuboid3D.corner_3));
    marker.points.push_back(corner_to_marker(Object3D->mCuboid3D.corner_3));
    marker.points.push_back(corner_to_marker(Object3D->mCuboid3D.corner_4));
    marker.points.push_back(corner_to_marker(Object3D->mCuboid3D.corner_4));
    marker.points.push_back(corner_to_marker(Object3D->mCuboid3D.corner_1));

    marker.points.push_back(corner_to_marker(Object3D->mCuboid3D.corner_5));
    marker.points.push_back(corner_to_marker(Object3D->mCuboid3D.corner_1));
    marker.points.push_back(corner_to_marker(Object3D->mCuboid3D.corner_6));
    marker.points.push_back(corner_to_marker(Object3D->mCuboid3D.corner_2));
    marker.points.push_back(corner_to_marker(Object3D->mCuboid3D.corner_7));
    marker.points.push_back(corner_to_marker(Object3D->mCuboid3D.corner_3));
    marker.points.push_back(corner_to_marker(Object3D->mCuboid3D.corner_8));
    marker.points.push_back(corner_to_marker(Object3D->mCuboid3D.corner_4));

    marker.points.push_back(corner_to_marker(Object3D->mCuboid3D.corner_5));
    marker.points.push_back(corner_to_marker(Object3D->mCuboid3D.corner_6));
    marker.points.push_back(corner_to_marker(Object3D->mCuboid3D.corner_6));
    marker.points.push_back(corner_to_marker(Object3D->mCuboid3D.corner_7));
    marker.points.push_back(corner_to_marker(Object3D->mCuboid3D.corner_7));
    marker.points.push_back(corner_to_marker(Object3D->mCuboid3D.corner_8));
    marker.points.push_back(corner_to_marker(Object3D->mCuboid3D.corner_8));
    marker.points.push_back(corner_to_marker(Object3D->mCuboid3D.corner_5));

    publisher_object_backgroud.publish(marker);
}

void NbvGenerator::publishBackgroudObject(pcl::PointCloud<pcl::PointXYZRGB>::Ptr allCloudPoints ){
    // 定义最小值和最大值
    Eigen::Vector4f minPoint;
    Eigen::Vector4f maxPoint;
    pcl::PointXYZ minPt, maxPt;

    // 计算点云的最小值和最大值
    //pcl::getMinMax3D(*allCloudPoints, minPoint, maxPoint);
    if (allCloudPoints->points.empty()) {
        minPoint << 0.0f, 0.0f, 0.0f, 0.0f;
        maxPoint << 0.0f, 0.0f, 0.0f, 0.0f;
        std::cerr << "Error occurred! 背景物体点云数量为0" << std::endl;
        //std::exit(EXIT_FAILURE); // 退出程序
        return;
    }

    float x_min = std::numeric_limits<float>::max();
    float y_min = std::numeric_limits<float>::max();
    float z_min = std::numeric_limits<float>::max();
    float x_max = -std::numeric_limits<float>::max();
    float y_max = -std::numeric_limits<float>::max();
    float z_max = -std::numeric_limits<float>::max();

    for (const auto& pt : allCloudPoints->points) {
        if (pt.x < x_min) x_min = pt.x;
        if (pt.y < y_min) y_min = pt.y;
        if (pt.z < z_min) z_min = pt.z;
        if (pt.x > x_max) x_max = pt.x;
        if (pt.y > y_max) y_max = pt.y;
        if (pt.z > z_max) z_max = pt.z;
    }

    minPoint << x_min, y_min, z_min, 0.0f;
    maxPoint << x_max, y_max, z_max, 0.0f;


    // 从最小值和最大值创建一个轴对齐的包围盒
    //pcl::PointXYZ min(minPoint[0], minPoint[1], minPoint[2]);
    //pcl::PointXYZ max(maxPoint[0], maxPoint[1], maxPoint[2]);
    //pcl::PointCloud<pcl::PointXYZ>::Ptr boundingBox(new pcl::PointCloud<pcl::PointXYZ>);

    // Create an object in the map.
    std::cout<<"可视化背景物体"<<std::endl;
    Object_Map *Object3D = new Object_Map;
    //Object3D->mvObject_2ds.push_back(obj2d);   // 2D objects in each frame associated with this 3D map object.
    Object3D->mnId = 100;             // 3d objects in the map.
    Object3D->mnClass = 60 /*餐桌*/;      // object class.
    Object3D->mnConfidence_foractive = 1;              // object confidence = mObjectFrame.size().
    //Object3D->mnAddedID_nouse = mCurrentFrame.mnId;        // added id.
    //Object3D->mnLastAddID = mCurrentFrame.mnId;      // last added id.
    //Object3D->mnLastLastAddID = mCurrentFrame.mnId;  // last last added id.
    //Object3D->mLastRect = obj2d->mBox_cvRect;             // last rect.
    //Object3D->mPredictRect = obj->mBoxRect;       // for iou.
    //Object3D->mSumPointsPos = 0; //cv::Mat::zeros(3,1,CV_32F);
    //Object3D->mAveCenter3D = obj2d->mPos_world;  ; //cv::Mat::zeros(3,1,CV_32F);
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
    Object3D->backgroud_object = true;
    Object3D->mCuboid3D.corner_1 = Eigen::Vector3d(x_min, y_min, 0);
    Object3D->mCuboid3D.corner_2 = Eigen::Vector3d(x_max, y_min, 0);
    Object3D->mCuboid3D.corner_3 = Eigen::Vector3d(x_max, y_max, 0);
    Object3D->mCuboid3D.corner_4 = Eigen::Vector3d(x_min, y_max, 0);
    Object3D->mCuboid3D.corner_5 = Eigen::Vector3d(x_min, y_min, z_max);
    Object3D->mCuboid3D.corner_6 = Eigen::Vector3d(x_max, y_min, z_max);
    Object3D->mCuboid3D.corner_7 = Eigen::Vector3d(x_max, y_max, z_max);
    Object3D->mCuboid3D.corner_8 = Eigen::Vector3d(x_min, y_max, z_max);
    Object3D->mCuboid3D.cuboidCenter = Eigen::Vector3d((x_max + x_min) / 2, (y_max + y_min) / 2, (z_max + 0.0) / 2);
    //std::cout<<"debug:publishBackgroudObject1:"<<Object3D->mCuboid3D.corner_2[0]-Object3D->mCuboid3D.corner_1[0]<<", "<<Object3D->mCuboid3D.width<<", "<<Object3D->mCuboid3D.height<<", "<<std::endl;
    //std::cout<<"debug:publishBackgroudObject1:"<<Object3D->mCuboid3D.corner_2[0]-Object3D->mCuboid3D.corner_1[0]<<", "<<Object3D->mCuboid3D.corner_3[1]-Object3D->mCuboid3D.corner_2[1]<<", "<<Object3D->mCuboid3D.corner_3[2]<<", "<<std::endl;
    Object3D->mCuboid3D.lenth = x_max - x_min;
    Object3D->mCuboid3D.width = y_max - y_min;
    Object3D->mCuboid3D.height = z_max - 0.0;
    //Object3D->Update_Twobj();

    //(1)物体
    visualization_msgs::Marker marker;
    marker.id = 0;
    //marker.lifetime = ros::Duration(1);
    marker.header.frame_id= MAP_FRAME_ID;
    marker.header.stamp=ros::Time::now();

    marker.type = visualization_msgs::Marker::LINE_LIST; //LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    //marker.color.r = color[2]/255.0; marker.color.g = color[1]/255.0; marker.color.b = color[0]/255.0; marker.color.a = 1.0;
    marker.color.r = 255.0; marker.color.g = 0.0; marker.color.b = 0.0; marker.color.a = 1.0;
    marker.scale.x = 0.01;
    //     8------7
    //    /|     /|
    //   / |    / |
    //  5------6  |
    //  |  4---|--3
    //  | /    | /
    //  1------2
    marker.points.push_back(corner_to_marker(Object3D->mCuboid3D.corner_1));
    marker.points.push_back(corner_to_marker(Object3D->mCuboid3D.corner_2));
    marker.points.push_back(corner_to_marker(Object3D->mCuboid3D.corner_2));
    marker.points.push_back(corner_to_marker(Object3D->mCuboid3D.corner_3));
    marker.points.push_back(corner_to_marker(Object3D->mCuboid3D.corner_3));
    marker.points.push_back(corner_to_marker(Object3D->mCuboid3D.corner_4));
    marker.points.push_back(corner_to_marker(Object3D->mCuboid3D.corner_4));
    marker.points.push_back(corner_to_marker(Object3D->mCuboid3D.corner_1));

    marker.points.push_back(corner_to_marker(Object3D->mCuboid3D.corner_5));
    marker.points.push_back(corner_to_marker(Object3D->mCuboid3D.corner_1));
    marker.points.push_back(corner_to_marker(Object3D->mCuboid3D.corner_6));
    marker.points.push_back(corner_to_marker(Object3D->mCuboid3D.corner_2));
    marker.points.push_back(corner_to_marker(Object3D->mCuboid3D.corner_7));
    marker.points.push_back(corner_to_marker(Object3D->mCuboid3D.corner_3));
    marker.points.push_back(corner_to_marker(Object3D->mCuboid3D.corner_8));
    marker.points.push_back(corner_to_marker(Object3D->mCuboid3D.corner_4));

    marker.points.push_back(corner_to_marker(Object3D->mCuboid3D.corner_5));
    marker.points.push_back(corner_to_marker(Object3D->mCuboid3D.corner_6));
    marker.points.push_back(corner_to_marker(Object3D->mCuboid3D.corner_6));
    marker.points.push_back(corner_to_marker(Object3D->mCuboid3D.corner_7));
    marker.points.push_back(corner_to_marker(Object3D->mCuboid3D.corner_7));
    marker.points.push_back(corner_to_marker(Object3D->mCuboid3D.corner_8));
    marker.points.push_back(corner_to_marker(Object3D->mCuboid3D.corner_8));
    marker.points.push_back(corner_to_marker(Object3D->mCuboid3D.corner_5));

    publisher_object_backgroud.publish(marker);
}

geometry_msgs::Point NbvGenerator::corner_to_marker(Eigen::Vector3d& v){
    geometry_msgs::Point point;
    point.x = v[0];
    point.y = v[1];
    point.z = v[2];
    return point;
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



    for(int i=0; i < candidates.size()  /*mCandidate_num_topub*/; i++)
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

        cv::Mat Twc = candidates[i].pose.clone();//Tcw.inv();
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

        mCandidate.points.clear();
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
        mCandidate.lifetime = ros::Duration(2.0);
        mCandidate.header.stamp = ros::Time::now();
        mCandidate.scale.x=fPointSize;
        mCandidate.scale.y=fPointSize;
        mCandidate.color.a=1.0f;
        if(i<mCandidate_num_topub){
            int color_num = i%7;
            mCandidate.color.r=float(colors_bgr[color_num][0]/255.0);
            mCandidate.color.b=float(colors_bgr[color_num][1]/255.0);
            mCandidate.color.g=float(colors_bgr[color_num][2]/255.0);
            publisher_candidate.publish(mCandidate);
        }
        else{
            mCandidate.color.r=0.0f;
            mCandidate.color.b=0.0f;
            mCandidate.color.g=0.0f;
            publisher_candidate_unsort.publish(mCandidate);
        }

        //ROS_INFO("Candidate %d, x:%f, y:%f, reward:%f", i, Twc.at<float>(0,3), Twc.at<float>(1,3), candidates[i].reward);
    }
}

void NbvGenerator::publishLocalNBV(const Candidate& nbv){

    ////1.构建地图，并向mam中添加地图点。 TODO：也应该添加 物体。
    //map_data MapData;
    //double theta_interval;
    //vector<MapPoint*> vpPts = mpMap->GetAllMapPoints();
    //for(size_t i=0; i<vpPts.size(); i++){
    //    if(vpPts[i]->isBad())
    //        continue;
    //
    //    float minDist = vpPts[i]->GetMinDistanceInvariance();  //根据金字塔计算出的最近可见距离
    //    float maxDist = vpPts[i]->GetMaxDistanceInvariance();  //根据金字塔计算出的最远可见距离
    //    float foundRatio = vpPts[i]->GetFoundRatio(); //地图点的查找率,如果过低,会被标记为bad point
    //
    //    //float Dist = sqrt((Tsc_curr.at<float>(0,3) - vpPts[i]->GetWorldPos().at<float>(0))*(Tsc_curr.at<float>(0,3) - vpPts[i]->GetWorldPos().at<float>(0))+
    //    //(Tsc_curr.at<float>(1,3) - vpPts[i]->GetWorldPos().at<float>(1))*(Tsc_curr.at<float>(1,3) - vpPts[i]->GetWorldPos().at<float>(2))+
    //    //(Tsc_curr.at<float>(2,3) - vpPts[i]->GetWorldPos().at<float>(2))*(Tsc_curr.at<float>(2,3) - vpPts[i]->GetWorldPos().at<float>(2)));
    //
    //    //if((Dist > maxDist)||(Dist < minDist))
    //    //    continue;
    //
    //    MapData.MapPointsCoordinate.push_back(std::vector<double>{vpPts[i]->GetWorldPos().at<float>(0), vpPts[i]->GetWorldPos().at<float>(1), vpPts[i]->GetWorldPos().at<float>(2)});
    //
    //    //观测角度(共视方向)的冗余量,论文中的alpha_max
    //    if(vpPts[i]->theta_std * 2.5 < 10.0/57.3){  //如果标准差小于..., 则设置冗余量为10弧度
    //        theta_interval = 10.0/57.3;
    //    }else{
    //        theta_interval = vpPts[i]->theta_std * 2.5;
    //    }
    //
    //    MapData.UB.push_back(double(vpPts[i]->theta_mean + theta_interval));    //观测角度(共视方向)的最大值
    //    MapData.LB.push_back(double(vpPts[i]->theta_mean - theta_interval));    //观测角度(共视方向)的最小值
    //    MapData.maxDist.push_back(double(maxDist));         //最远可见距离
    //    MapData.minDist.push_back(double(minDist));         //最近可见距离
    //    MapData.foundRatio.push_back(double(foundRatio));   //地图点的查找率
    //}
    //
    ////2.构建mam camera对象
    //camera camera_model(MapData, 20);    // zhang 这里的阈值20对于我的实验环境是不是 有点高??
    //camera_model.setCamera(mfx, mfy, mcx, mcy,  mImageWidth,  mImageHeight, mMax_dis, mMin_dis );

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
            ////visible_info VI;
            //int visible_pts = 0;
            //double great_angle = -5.0;
            //
            ////利用速度模型, 生成下一时刻的机器人位姿
            //
            ////在机器人位姿的基础上，计算最佳扭头的角度
            //cv::Mat body_new;
            //
            //for(int i=0; i<=mDivide; i++){
            //    double angle = mLocalNeckRange/mDivide * i - mLocalNeckRange/2.0 ;
            //    //旋转
            //    Eigen::AngleAxisd rotation_vector (angle, Eigen::Vector3d(0,0,1));
            //    Eigen::Matrix3d rotation_matrix = rotation_vector.toRotationMatrix();  //分别加45度
            //    //Eigen::Isometry3d trans_matrix;
            //    //trans_matrix.rotate(rotation_vector);
            //    //trans_matrix.pretranslate(Vector3d(0,0,0));
            //    cv::Mat rotate_mat = Converter::toCvMat(rotation_matrix);
            //
            //    //平移
            //    cv::Mat t_mat = (cv::Mat_<float>(3, 1) << 0, 0, 0);
            //
            //    //总变换矩阵
            //    cv::Mat trans_mat = cv::Mat::eye(4, 4, CV_32F);
            //    rotate_mat.copyTo(trans_mat.rowRange(0, 3).colRange(0, 3));
            //    t_mat.copyTo(trans_mat.rowRange(0, 3).col(3));
            //
            //    body_new = T_w_basefootprint * trans_mat;   //旋转之后的机器人位姿
            //    cv::Mat T_w_cam = body_new * mT_basefootprint_cam;   //旋转之后的相机位姿
            //    int num = camera_model.countVisible(T_w_cam);   //利用相机模型， 计算最佳的扭头角度
            //    cout<<"--agl:" << angle/M_PI*180<<",num:"<<num;
            //    if(num > visible_pts){
            //        great_angle = angle;
            //        visible_pts = num;
            //    }
            //    localCandidate lc;
            //    lc.angle = angle;
            //    lc.num = num;
            //
            //    //double localreward = num + ;
            //}
            ////unique_lock<mutex> lock(mMutexMamAngle);
            //mGreat_angle = great_angle;//   /M_PI*180 ;
            ////cout << "----number of points predicted=" << visible_pts <<", angle:"<<mGreat_angle/M_PI*180 << endl;
        }
        else{
            // 直接看背景物体的中心
            //double desk_center_x = -2.0;
            //double desk_center_y = 0.0;
            //double direct_x = transform.getOrigin().x() - desk_center_x;
            //double direct_y = transform.getOrigin().y() - desk_center_y;

            //世界坐标系下，物体的中心
            cv::Mat desk_center_w = cv::Mat::zeros(4,1,CV_32F);
            desk_center_w.at<float>(0,0) = nbv.bo->mean_x;
            desk_center_w.at<float>(1,0) = nbv.bo->mean_y;
            //std::cout<<"Local NBV的x"<<nbv.bo->mean_x<<"， y"<<nbv.bo->mean_y<<std::endl;
            desk_center_w.at<float>(2,0) = 0.0;
            desk_center_w.at<float>(3,0) = 1.0;
            //机器人坐标系下，物体的中心
            cv::Mat desk_center_robot = T_w_basefootprint.inv() * desk_center_w;
            double angle = atan2( desk_center_robot.at<float>(1,0),  desk_center_robot.at<float>(0,0) );  //与x轴的夹角
            mGreat_angle = angle;
            //cout << "---- angle:" <<mGreat_angle/M_PI*180 << endl;

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

double NbvGenerator::computeCosAngle_Signed( Eigen::Vector3d &v1 /*基坐标轴*/,  Eigen::Vector3d &v2, bool isSigned/*为1时，角度范围为360度*/){
    //Eigen::Vector3d view(   objectPose.at<float>(0,3)-candidate_pose.at<float>(0,3),
    //                        objectPose.at<float>(1,3)-candidate_pose.at<float>(1,3),
    //                        0.0     );
    //double cosTheta  = view.dot(ie) / (view.norm() * ie.norm()); //角度cos值
    //return (cosTheta); //std::acos(cosTheta)
    double cosValNew = v1.dot(v2) / (v1.norm()*v2.norm()); //通过向量的点乘, 计算角度cos值
    double angleNew = acos(cosValNew) * 180 / M_PI;     //弧度角

    // 如果为360度内的角度，则以v1是基坐标轴，如果v2逆时针旋转到zerovec，则修正夹角
    if(isSigned){
        if (v2.cross(v1)(2) > 0) {
            angleNew = 360 - angleNew;
        }
    }

    return angleNew;
}

void NbvGenerator::addOldNBV(Candidate &candidate){
    mNBVs_old.push_back(candidate);
    for (int i = 0; i < mNBVs_old.size(); i++) {
        //for (int j = i + 1; j < mNBVs_old.size(); j++) {
            double dis = sqrt( (mNBVs_old[i].pose.at<float>(0,3)-candidate.pose.at<float>(0,3)) * (mNBVs_old[i].pose.at<float>(0,3)-candidate.pose.at<float>(0,3)) +
                               (mNBVs_old[i].pose.at<float>(1,3)-candidate.pose.at<float>(1,3)) * (mNBVs_old[i].pose.at<float>(1,3)-candidate.pose.at<float>(1,3))
                            );
            if(dis>mNBVs_scale)
                mNBVs_scale = dis;
        //}
    }
}

void NbvGenerator::clearOldNBV(){
    mNBVs_old.clear();
    mNBVs_scale = 0.0;
}

bool NbvGenerator::near(Object_Map* fo, const cv::Mat& nbv, double &distance){
    distance =   (fo->mCuboid3D.cuboidCenter.x() - nbv.at<float>(0,3))  *   (fo->mCuboid3D.cuboidCenter.x() - nbv.at<float>(0,3))
                    +   (fo->mCuboid3D.cuboidCenter.y() - nbv.at<float>(1,3))  *   (fo->mCuboid3D.cuboidCenter.y() - nbv.at<float>(1,3)) ;
    distance = sqrt(distance);
    if(distance<=0){
        // 输出错误信息到 std::cerr
        std::cerr << "Error: distance计算值<0. 错误 !" << std::endl;
        // 终止程序
        std::exit(EXIT_FAILURE);
    }
    if(distance<1.5) {
        distance = 1.5;
        return true;
    }
    else if(distance<1.0){
        return true;
    }
    else
        return false;
}

//计算每个候选视点的 评价函数
void NbvGenerator::computeReward(Candidate &candidate){


    //机器人底盘的世界坐标
    Eigen::Matrix4d T_w_basefoot = Converter::cvMattoMatrix4d( candidate.pose* mT_basefootprint_cam.inv()  );

    //reward = 所有物体的（关联置信度*非完整性评价*观测角度）  +   与已有的NBV的距离

    //1.所有物体的（关联置信度*非完整性评价*观测角度
    double np_reward = 0;
    double IE_reward = 0;
    double object_reward = 0;
    int object_viewed_num = 0;
    Eigen::Vector3d direction_all = Eigen::Vector3d::Zero();
    auto obj3ds = candidate.bo->mvFOs;
    //std::cout<<"Reward compute: ";
    int num_valid=0;
    for (int i = 0; i < (int)obj3ds.size(); i++) {
        Object_Map *obj3d = obj3ds[i];
        if(obj3d->bad_3d)
            continue;
        if(obj3d->end_build)
            continue;

        //bool viewed = obj3d->WheatherInRectFrameOf(candidate.pose, mfx, mfy, mcx, mcy, mImageWidth, mImageHeight);
        double distance = 0.0;
        bool viewed = near(obj3d, candidate.pose, distance);
        if(viewed )
        {
            //（1）关联置信度
            np_reward = 0;

            // (2)物体的完整性评价 mIE
            IE_reward = obj3d->mIE;
            if(IE_reward<0 || IE_reward>1){
                // 输出错误信息到 std::cerr
                std::cerr << "Error: IE_reward<0 || IE_reward>1 !" << std::endl;
                // 终止程序
                //std::exit(EXIT_FAILURE);
                continue;
            }

            num_valid++;

            // (3) Candidate和物体最佳视角 之间的视线夹角.
            Eigen::Vector3d direction = obj3d->mMainDirection;
            object_viewed_num ++;
            Eigen::Vector3d robot_view_object (
                obj3d->mCuboid3D.cuboidCenter(0) - T_w_basefoot(0,3) ,
                obj3d->mCuboid3D.cuboidCenter(1) - T_w_basefoot(1,3) ,
                0.0
            );
            double cosTheta  = robot_view_object.dot(direction) / (robot_view_object.norm() * direction.norm()); //角度cos值
            //std::cout<<",  [IE_reward,cosTheta]:"<<IE_reward <<","<< cosTheta;
            object_reward += /*np_reward **/ IE_reward * cosTheta /*/ distance*/;
        }
    }
    //std::cout<<std::endl;

    Eigen::Vector3d v_deskcentor(candidate.bo->mean_x, candidate.bo->mean_y ,0.0);
    // 2. 与nbvs old之间的角度  间隔开
    //Eigen::Vector3d v1(1,0,0);
    //double angle_oldNBV = 0;
    //for( auto nbv: mNBVs_old){
    //    Eigen::Vector3d v2( v_deskcentor(0)-nbv.pose.at<float>(0,3),
    //                        v_deskcentor(1)-nbv.pose.at<float>(1,3),
    //                        0.0);
    //    angle_oldNBV += computeCosAngle_Signed(v1, v2, 1);
    //}
    //
    //angle_oldNBV /= mNBVs_old.size();//NBVold与v1的平均夹角
    //Eigen::Vector3d v2 (    v_deskcentor(0)-candidate.pose.at<float>(0,3),
    //                        v_deskcentor(1)-candidate.pose.at<float>(1,3),
    //                                0.0  );
    //
    //double angle_Candidate = computeCosAngle_Signed(v1, v2, 1);//candidate_centor 与 v1的夹角
    //double angle_reward =  fabs(angle_Candidate-angle_oldNBV)  / 360;

    double angle_oldNBV = 0;
    cv::Point2f v_nbvs(0, 0);
    for( auto nbv: mNBVs_old){
        if(nbv.bo->mnId == candidate.bo->mnId) {
            cv::Point2f v_nbv(nbv.pose.at<float>(0, 3) - v_deskcentor(0),
                              nbv.pose.at<float>(1, 3) - v_deskcentor(1));

            v_nbvs += normalize(v_nbv);
        }
    }
    v_nbvs /= (float)mNBVs_old.size();//NBVold与v1的平均夹角
    Eigen::Vector3d v_nbvs_eigen (    v_nbvs.x, v_nbvs.y, 0.0  );
    Eigen::Vector3d v_candidate (   candidate.pose.at<float>(0,3)-v_deskcentor(0),
                                    candidate.pose.at<float>(1,3)-v_deskcentor(1),
                                    0.0  );
    double angle_Candidate = computeCosAngle_Signed(v_nbvs_eigen, v_candidate, 0);//candidate_centor 与 v1的夹角
    double angle_reward =  angle_Candidate  / 180;

    //3. 与nbvs old保持最大距离  【不好用】
    //Eigen::Vector3d v_deskcentor(candidate.bo->mean_x, candidate.bo->mean_y ,0.0);
    //double distance_oldNBV = 0;
    //for( auto nbv: mNBVs_old){
    //    double distance = sqrt(  (candidate.pose.at<float>(0,3)-nbv.pose.at<float>(0,3))*(candidate.pose.at<float>(0,3)-nbv.pose.at<float>(0,3))
    //                            +(candidate.pose.at<float>(1,3)-nbv.pose.at<float>(1,3))*(candidate.pose.at<float>(1,3)-nbv.pose.at<float>(1,3))
    //                          );
    //    distance_oldNBV += distance;
    //}
    //
    //distance_oldNBV /= mNBVs_old.size();//NBVold与v1的平均distance
    //double distance_reward =  distance_oldNBV/sqrt(candidate.bo->length*candidate.bo->length + candidate.bo->width*candidate.bo->width);

    //4. 与nbvs old保持一定的视角差，防止陷入局部最优。
    double angle_difference_cost = 0;
    Eigen::Vector3d v_candidate_desk (      v_deskcentor(0)-candidate.pose.at<float>(0,3),
                                            v_deskcentor(1)-candidate.pose.at<float>(1,3),
                                            0.0  );
    for( auto nbv: mNBVs_old){
        if(nbv.bo->mnId == candidate.bo->mnId){
            Eigen::Vector3d v_oldNbv_desk( v_deskcentor(0)-nbv.pose.at<float>(0,3),
                            v_deskcentor(1)-nbv.pose.at<float>(1,3),
                            0.0);
            double angle_difference = computeCosAngle_Signed(v_oldNbv_desk, v_candidate_desk, 0);
            if(angle_difference<20)
                angle_difference_cost += 1;
        }

    }

    //3. 两项相加,
    double reward = object_reward - angle_difference_cost*mReward_angle_cost + angle_reward*mReward_dis;
    printf("[Result reward compute] x:%f, y:%f, reward:%f, object_reward:%f, angle_difference_cost:%f, angle_reward:%f \n",
           candidate.pose.at<float>(0,3), candidate.pose.at<float>(1,3), reward, object_reward, angle_difference_cost, angle_reward*mReward_dis);
    //ROS_INFO_STREAM("  robot_view_world:" << robot_view_world.format(Eigen::IOFormat(4, 0, ", ", " << ", "", "", " << ")) << std::endl);
    //ROS_INFO_STREAM("  direction_all:" << direction_all.format(Eigen::IOFormat(4, 0, ", ", " << ", "", "", " << ")) << std::endl);
    candidate.reward = reward;
}


};

