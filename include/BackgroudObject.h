//
// Created by zhjd on 5/19/23.
//

#ifndef ACTIVE_EAO_NEW_BACKGROUDOBJECT_H
#define ACTIVE_EAO_NEW_BACKGROUDOBJECT_H

// pcl边界
#include <pcl/features/boundary.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>

//eigen cv的convert
#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>
#include<opencv2/core/eigen.hpp>

#include "Converter.h"
#include "MapPlane.h"
#include "Object.h"

//typedef pcl::PointXYZRGB PointT;
//typedef pcl::PointCloud<PointT> PointCloud;

namespace ORB_SLAM2
{

class Object_Map;

class BackgroudObject {

public:
    BackgroudObject();
    ~BackgroudObject();

public:
    int mnId;
    int mnClass = 60;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr mPlane;         //支撑面
    double mean_x,mean_y,mean_z;	//物体中心
    double max_x,max_y,max_z;
    double min_x,min_y,min_z;
    double length,width,height;
    bool end_activemapping = false;
    double IEvalue;
    double FO_num, FO_num_notend ;
    cv::Mat pose_mat = cv::Mat::eye(4, 4, CV_32F);

public:
    bool include(Object_Map* fo);

    void computeValue(std::vector<Object_Map*> FOs);

    void computePose();

    bool return_end_active_mapping();
};



}

#endif //ACTIVE_EAO_NEW_BACKGROUDOBJECT_H
