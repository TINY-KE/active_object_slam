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

    fPointSize=0.01;
    mCandidate.header.frame_id = MAP_FRAME_ID;
    mCandidate.ns = CANDIDATE_NAMESPACE;
    mCandidate.id=1;
    mCandidate.type = visualization_msgs::Marker::LINE_LIST;
    mCandidate.pose.orientation.w=1.0;
    mCandidate.action=visualization_msgs::Marker::ADD;

    // Configure MapPlanes 平面的中心法线
    mPlanesCentroid.header.frame_id = MAP_FRAME_ID;
    mPlanesCentroid.ns = "PlaneNorm";
    mPlanesCentroid.id=2;
    mPlanesCentroid.type = visualization_msgs::Marker::POINTS;
    mPlanesCentroid.scale.x= 10 * fPointSize;
    mPlanesCentroid.scale.y= 10 * fPointSize;
    mPlanesCentroid.pose.orientation.w=1.0;
    mPlanesCentroid.action=visualization_msgs::Marker::ADD;

    //
}

void NbvGenerator::Run() {
    while(1)
    {
        vector<MapPlane *> vpMPlanes = mpMap->GetAllMapPlanes();
        PublishMapPlanes(vpMPlanes);
        vector<Object_Map*> vpMapObjects = mpMap->GetObjects();
        //计算位姿

        //计算视点的评价函数

        //



    }
}

void  NbvGenerator::ExtractCandidates(){
    mPlanesCentroid.points.clear();
    mPlanesCentroid.lifetime = ros::Duration(0.2);
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
    vector<cv::Mat> allCandidate;
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
        mPlanesCentroid.points.push_back(p);


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

        //(2)经纬线扫描法
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
            Eigen::AngleAxisd rotation_vector1 (angle, Eigen::Vector3d(0,0,1));
            Eigen::Matrix3d rotation_matrix1 = rotation_vector1.toRotationMatrix();
            Eigen::AngleAxisd rotation_vector2 (M_PI/2, Eigen::Vector3d(0,1,0));
            Eigen::Matrix3d rotation_matrix2 = rotation_vector2.toRotationMatrix();
            Eigen::AngleAxisd rotation_vector3 (-M_PI/2, Eigen::Vector3d(0,0,1));
            Eigen::Matrix3d rotation_matrix3 = rotation_vector3.toRotationMatrix();
            Eigen::Matrix3d rotation_matrix = rotation_matrix1*rotation_matrix2*rotation_matrix3;
            cv::Mat rotate_mat = Converter::toCvMat(rotation_matrix);
            cv::Mat t_mat = (cv::Mat_<float>(3, 1) << x, y, mean_z);
            cv::Mat trans_mat = cv::Mat::eye(4, 4, CV_32F);
            rotate_mat.copyTo(trans_mat.rowRange(0, 3).colRange(0, 3));
            t_mat.copyTo(trans_mat.rowRange(0, 3).col(3));
            allCandidate.push_back(trans_mat);
        }

        // 桌面高度的 水平面的数量
        num++;

    }
    cout << "-------" << endl;
    cout << "桌面高度的水平面的数量: " <<num << endl << endl;

    //发布中心点的rviz marker
    mPlanesCentroid.header.stamp = ros::Time::now();
    mPlanesCentroid.color.r = 0.0; mPlanesCentroid.color.g = 0.0; mPlanesCentroid.color.b = 1.0; mPlanesCentroid.color.a = 1.0;
    publisher_centroid.publish(mPlanesCentroid);

    //发布桌面边缘的候选点
    PublishCurrentCamera(allCandidate);

    // 发布平面点和边缘点cVBZXN
    sensor_msgs::PointCloud2 colored_msg;
    colored_pcl_ptr->width = 1;
    colored_pcl_ptr->height = colored_pcl_ptr->points.size();
    pcl::toROSMsg( *colored_pcl_ptr,  colored_msg);  //将点云转化为消息才能发布
    colored_msg.header.frame_id = MAP_FRAME_ID;//帧id改成和velodyne一样的
    pubCloud.publish( colored_msg); //发布调整之后的点云数据，主题为/adjustd_cloud
}

void  NbvGenerator::PublishMapPlanes(const vector<MapPlane *> &vpMPs)
{
    mPlanesCentroid.points.clear();
    mPlanesCentroid.lifetime = ros::Duration(0.2);
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
    vector<cv::Mat> allCandidate;
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
        mPlanesCentroid.points.push_back(p);


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

        //(2)经纬线扫描法
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
            Eigen::AngleAxisd rotation_vector1 (angle, Eigen::Vector3d(0,0,1));
            Eigen::Matrix3d rotation_matrix1 = rotation_vector1.toRotationMatrix();
            Eigen::AngleAxisd rotation_vector2 (M_PI/2, Eigen::Vector3d(0,1,0));
            Eigen::Matrix3d rotation_matrix2 = rotation_vector2.toRotationMatrix();
            Eigen::AngleAxisd rotation_vector3 (-M_PI/2, Eigen::Vector3d(0,0,1));
            Eigen::Matrix3d rotation_matrix3 = rotation_vector3.toRotationMatrix();
            Eigen::Matrix3d rotation_matrix = rotation_matrix1*rotation_matrix2*rotation_matrix3;
            cv::Mat rotate_mat = Converter::toCvMat(rotation_matrix);
            cv::Mat t_mat = (cv::Mat_<float>(3, 1) << x, y, mean_z);
            cv::Mat trans_mat = cv::Mat::eye(4, 4, CV_32F);
            rotate_mat.copyTo(trans_mat.rowRange(0, 3).colRange(0, 3));
            t_mat.copyTo(trans_mat.rowRange(0, 3).col(3));
            allCandidate.push_back(trans_mat);
        }

        // 桌面高度的 水平面的数量
        num++;

    }
    cout << "-------" << endl;
    cout << "桌面高度的水平面的数量: " <<num << endl << endl;

    //发布中心点的rviz marker
    mPlanesCentroid.header.stamp = ros::Time::now();
    mPlanesCentroid.color.r = 0.0; mPlanesCentroid.color.g = 0.0; mPlanesCentroid.color.b = 1.0; mPlanesCentroid.color.a = 1.0;
    publisher_centroid.publish(mPlanesCentroid);

    //发布桌面边缘的候选点
    PublishCurrentCamera(allCandidate);

    // 发布平面点和边缘点cVBZXN
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


void NbvGenerator::PublishCurrentCamera(const vector<cv::Mat> &Twcs)
{
    if(Twcs.empty())
        return ;

    mCandidate.points.clear();

    for(int i=0; i<Twcs.size(); i++)
    {
        //cv::Mat Tcw = Tcws[i];
        float d = 0.04;
        //Camera is a pyramid. Define in camera coordinate system
        cv::Mat o = (cv::Mat_<float>(4, 1) << 0, 0, 0, 1);
        cv::Mat p1 = (cv::Mat_<float>(4, 1) << d, d * 0.8, d * 0.5, 1);
        cv::Mat p2 = (cv::Mat_<float>(4, 1) << d, -d * 0.8, d * 0.5, 1);
        cv::Mat p3 = (cv::Mat_<float>(4, 1) << -d, -d * 0.8, d * 0.5, 1);
        cv::Mat p4 = (cv::Mat_<float>(4, 1) << -d, d * 0.8, d * 0.5, 1);

        cv::Mat Twc = Twcs[i];//Tcw.inv();
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
    }

    mCandidate.header.stamp = ros::Time::now();
    mCandidate.color.r=0.0f;
    mCandidate.color.b=0.0f;
    mCandidate.color.g=1.0f;
    mCandidate.color.a=1.0f;
    mCandidate.scale.x=fPointSize;
    mCandidate.scale.y=fPointSize;
    publisher_candidate.publish(mCandidate);
}


void NbvGenerator::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

};

