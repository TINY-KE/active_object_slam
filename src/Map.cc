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
                    pF.mvpMapPlanes[i] = (*sit);
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


} //namespace ORB_SLAM
