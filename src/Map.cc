/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "Map.h"

#include<mutex>

namespace ORB_SLAM2
{

Map::Map():mnMaxKFid(0),mnBigChangeIdx(0)
{
    mfDisTh = 0.2;
    mfAngleTh = 0.8;
}

void Map::AddKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.insert(pKF);
    if(pKF->mnId>mnMaxKFid)
        mnMaxKFid=pKF->mnId;
}

void Map::AddMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.insert(pMP);
}

void Map::EraseMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.erase(pMP);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::EraseKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.erase(pKF);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs)
{
    unique_lock<mutex> lock(mMutexMap);
    mvpReferenceMapPoints = vpMPs;
}

void Map::InformNewBigChange()
{
    unique_lock<mutex> lock(mMutexMap);
    mnBigChangeIdx++;
}

int Map::GetLastBigChangeIdx()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnBigChangeIdx;
}

vector<KeyFrame*> Map::GetAllKeyFrames()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
}

vector<MapPoint*> Map::GetAllMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<MapPoint*>(mspMapPoints.begin(),mspMapPoints.end());
}

long unsigned int Map::MapPointsInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspMapPoints.size();
}

long unsigned int Map::KeyFramesInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspKeyFrames.size();
}

vector<MapPoint*> Map::GetReferenceMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return mvpReferenceMapPoints;
}

long unsigned int Map::GetMaxKFid()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMaxKFid;
}

void Map::clear()
{
    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
        delete *sit;

    for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++)
        delete *sit;

    mspMapPoints.clear();
    mspKeyFrames.clear();
    mnMaxKFid = 0;
    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();
}


//[active slam]
void Map::AddObject(Object_Map *pObj)
{
    unique_lock<mutex> lock(mMutexMap);
    mvObjectMap.insert(pObj);
}

vector<Object_Map*> Map::GetObjects()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<Object_Map*>(mvObjectMap.begin(),mvObjectMap.end());
}

void Map::ClearBackgroudObjects() {
    mvBackgroudObjectMap.clear();
}

void Map::AddBackgroudObject(BackgroudObject *pObj)
{
    unique_lock<mutex> lock(mMutexMap);
    for(std::set<BackgroudObject*>::iterator iter=mvBackgroudObjectMap.begin(); iter!=mvBackgroudObjectMap.end();  ){
       if(pObj->mnId == (*iter)->mnId ){
           iter = mvBackgroudObjectMap.erase(iter);
       }
       else{
           ++ iter;
       }
    }
    mvBackgroudObjectMap.insert(pObj);
}

vector<BackgroudObject*> Map::GetBackgroudObjects()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<BackgroudObject*>(mvBackgroudObjectMap.begin(),mvBackgroudObjectMap.end());
}


// plane

// add plane --------------------------------
void Map::AddMapPlane(MapPlane *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    // 最开始在RGBD初始化时候使用和创建关键帧，插入平面的系数
    mspMapPlanes.insert(pMP);
}

void Map::EraseMapPlane(MapPlane *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPlanes.erase(pMP);

    mvnRemovedPlanes.push_back(pMP->mnId);
}

void Map::AssociatePlanesByBoundary(Frame &pF, bool out)
{

    unique_lock<mutex> lock(mMutexMap);
    // 当前帧为pF
    pF.mbNewPlane = false;

    if (out)
        cout << "Plane associate in map  ID :  " << pF.mnId << "   num of Plane: " << pF.mnPlaneNum << " TH: " << mfDisTh << endl;
    // 取出当前帧下的可见平面
    // 1. 判断是否重合（在mspMapPlanes和mspNotSeenMapPlanes），放在mvpMapPlanes下
    // 2. 判断是否为垂直或者平行的平面（仅在mspMapPlanes中）
    for (int i = 0; i < pF.mnPlaneNum; ++i)
    {
        // 遍历当前观测帧下的每个平面，并把平面系数投影到了世界坐标系下
        cv::Mat pM = pF.ComputePlaneWorldCoeff(i);
        if (out)
            cout << " plane  " << i << " : " << endl;
        float ldTh = mfDisTh;
        // 遍历地图实例上的每个平面
        for (set<MapPlane *>::iterator sit = mspMapPlanes.begin(), send = mspMapPlanes.end(); sit != send; sit++)
        {
            cv::Mat pW = (*sit)->GetWorldPos();
            // 获得两个平面夹角的cos值
            float angle = pM.at<float>(0, 0) * pW.at<float>(0, 0) +
                          pM.at<float>(1, 0) * pW.at<float>(1, 0) +
                          pM.at<float>(2, 0) * pW.at<float>(2, 0);

            if (out)
                cout << ":  angle : " << angle << endl;

            // 如果夹角很小的情况下
            if ((angle > mfAngleTh || angle < -mfAngleTh)) // associate plane
            {
                // 求解地图实例上的边缘点集中每个点到平面的平均距离(实际程序中是:最小距离)
                // 其中,  pM是当前帧的第i个平面在世界坐标系的投影
                //      (*sit)->mvBoundaryPoints是某个地图中平面的边界点
                // 如果关联成功了, 则直接将当前帧i清空, 转而放入(*sit).
                double dis = PointDistanceFromPlane(pM, (*sit)->mvBoundaryPoints, out);
                // 小于阈值则进行数据关联
                if (dis < ldTh)
                {
                    ldTh = dis;
                    if (out)
                        cout << "  associate!" << endl;
                    pF.mvpMapPlanes[i] = static_cast<MapPlane *>(nullptr);
                    // 注意，最后放入的是地图实例而不是观测（即在各帧中的平面索引）。后续可以将地图实例对应的所有帧中的平面融合在一起， 成为一个平面点云集合
                    pF.mvpMapPlanes[i] = (*sit);   //*sit代表全局地图中的一个plane

                    //更新*sit的包络框


                    continue;
                }
            }
        }
        if (out)
            cout << endl;
    }

    // 不符合上述要求的视为新的平面
    for (auto p : pF.mvpMapPlanes)
    {
        if (p == nullptr)
            pF.mbNewPlane = true;
    }
}

double Map::PointDistanceFromPlane(const cv::Mat &plane, PointCloud::Ptr boundry, bool out)
{
    double res = 100;
    if (out)
        cout << " compute dis: " << endl;
    // 点到直线的距离：d = 1/M * ( Mx + d )
    for (auto p : boundry->points)
    {
        double dis = abs(plane.at<float>(0, 0) * p.x +
                         plane.at<float>(1, 0) * p.y +
                         plane.at<float>(2, 0) * p.z +
                         plane.at<float>(3, 0));
        if (dis < res)
            res = dis;
    }
    if (out)
        cout << endl
             << "ave : " << res << endl;   //zhang: 这不是最小距离吗? 怎么标注的是平均距离
    return res;
}

vector<MapPlane *> Map::GetAllMapPlanes()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<MapPlane *>(mspMapPlanes.begin(), mspMapPlanes.end());
    ;
}

void Map::SearchMatchedPlanes(KeyFrame *pKF, cv::Mat Scw, const vector<MapPlane *> &vpPlanes, vector<MapPlane *> &vpMatched, bool out)
{

    unique_lock<mutex> lock(mMutexMap);

    cv::Mat ScwT;
    cv::transpose(Scw, ScwT);

    vpMatched = vector<MapPlane *>(pKF->mnPlaneNum, static_cast<MapPlane *>(nullptr));

    if (out)
        cout << "Plane associate in map  ID :  " << pKF->mnId << "   num of Plane: " << pKF->mnPlaneNum << " TH: " << mfDisTh << endl;

    for (int i = 0; i < pKF->mnPlaneNum; ++i)
    {

        cv::Mat pM = ScwT * pKF->mvPlaneCoefficients[i];
        if (out)
            cout << " plane  " << i << " : " << endl;
        float ldTh = mfDisTh;
        for (int j = 0; j < vpPlanes.size(); ++j)
        {
            cv::Mat pW = vpPlanes[j]->GetWorldPos();

            float angle = pM.at<float>(0, 0) * pW.at<float>(0, 0) +
                          pM.at<float>(1, 0) * pW.at<float>(1, 0) +
                          pM.at<float>(2, 0) * pW.at<float>(2, 0);

            if (out)
                cout << j << ":  angle : " << angle << endl;
            // associate plane
            if ((angle > mfAngleTh || angle < -mfAngleTh))   // 因为计算出的是cos值, 所以大于阈值(0.8),代表角度接近0
            {

                double dis = PointDistanceFromPlane(pM, vpPlanes[j]->mvBoundaryPoints, out);
                if (dis < ldTh)
                {
                    ldTh = dis;
                    if (out)
                        cout << "  associate!" << endl;
                    vpMatched[i] = static_cast<MapPlane *>(nullptr);
                    vpMatched[i] = vpPlanes[j];
                    continue;
                }
            }
        }
    }
}

std::vector<long unsigned int> Map::GetRemovedPlanes()
{
    unique_lock<mutex> lock(mMutexMap);
    return mvnRemovedPlanes;
}



Object_Map* Map::computeMinandMax(MapPlane * plane){
    //计算当前平面,在各关键帧中对应的平面
    map<KeyFrame *, int> observations = plane->GetObservations();  //std::map<KeyFrame*, int> mObservations;

    //将各关键帧中的平面,融合为一个allCloudPoints
    //typedef pcl::PointXYZRGB PointT;
    //pcl::PointCloud<PointT> PointCloud;
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

    ////降维过滤器
    //pcl::VoxelGrid<PointT> voxel;
    //voxel.setLeafSize(0.002, 0.002, 0.002);
    //
    ////对allCloudPoints降维成tmp
    //PointCloud::Ptr tmp(new PointCloud());
    //voxel.setInputCloud(allCloudPoints);
    //voxel.filter(*tmp);

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
        std::exit(EXIT_FAILURE); // 退出程序
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
    std::cout<<"INIT背景物体"<<std::endl;
    Object_Map *Object3D = new Object_Map;
    //Object3D->mvObject_2ds.push_back(obj2d);   // 2D objects in each frame associated with this 3D map object.
    Object3D->mnId = 0;             // 3d objects in the map.
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

    return Object3D;
}

} //namespace ORB_SLAM
