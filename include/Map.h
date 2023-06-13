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

#ifndef MAP_H
#define MAP_H

#include "MapPoint.h"
#include "KeyFrame.h"
#include <set>
#include "Object.h"
#include <mutex>

#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/ModelCoefficients.h>

namespace ORB_SLAM2
{

class MapPoint;
class KeyFrame;
class Object_Map;
class BackgroudObject;
class MapPlane;
class Frame;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;

class Map
{
public:
    Map();

    void AddKeyFrame(KeyFrame* pKF);
    void AddMapPoint(MapPoint* pMP);
    void EraseMapPoint(MapPoint* pMP);
    void EraseKeyFrame(KeyFrame* pKF);
    void SetReferenceMapPoints(const std::vector<MapPoint*> &vpMPs);
    void InformNewBigChange();
    int GetLastBigChangeIdx();

    std::vector<KeyFrame*> GetAllKeyFrames();
    std::vector<MapPoint*> GetAllMapPoints();
    std::vector<MapPoint*> GetReferenceMapPoints();

    long unsigned int MapPointsInMap();
    long unsigned  KeyFramesInMap();

    long unsigned int GetMaxKFid();

    void clear();

    std::vector<KeyFrame*> mvpKeyFrameOrigins;

    std::mutex mMutexMapUpdate;

    // This avoid that two points are created simultaneously in separate threads (id conflict)
    std::mutex mMutexPointCreation;

protected:
    std::set<MapPoint*> mspMapPoints;
    std::set<KeyFrame*> mspKeyFrames;

    std::vector<MapPoint*> mvpReferenceMapPoints;

    long unsigned int mnMaxKFid;

    // Index related to a big change in the map (loop closure, global BA)
    int mnBigChangeIdx;

    std::mutex mMutexMap;

//[active slam]
public:
    std::vector<cv::Mat> mvNBVs_pose; //存储已到达的NBV，从而使下一个NBV尽量已到达的位置。
protected:
    //std::vector<Object_Map*> mvObjectMap;
    std::set<Object_Map*> mvObjectMap;
    std::set<BackgroudObject*> mvBackgroudObjectMap;

public:
    void AddObject(Object_Map *pObj);
    std::vector<Object_Map*> GetObjects();
    void ClearBackgroudObjects();
    void AddBackgroudObject(BackgroudObject *pObj);
    std::vector<BackgroudObject*> GetBackgroudObjects();

//plane
public:
    void AddMapPlane(MapPlane *pMP);
    void EraseMapPlane(MapPlane *pMP);
    std::vector<MapPlane*> GetAllMapPlanes();
    void AssociatePlanesByBoundary(Frame &pF, bool out=false);
    double PointDistanceFromPlane(const cv::Mat &plane, PointCloud::Ptr boundry, bool out=false);
    void SearchMatchedPlanes(KeyFrame *pKF, cv::Mat Scw, const std::vector<MapPlane *> &vpPlanes, std::vector<MapPlane *> &vpMatched, bool out=false);
    std::vector<long unsigned int> GetRemovedPlanes();
    Object_Map* computeMinandMax(MapPlane * plane);
protected:
    float mfDisTh;
    float mfAngleTh;
    std::set<MapPlane*> mspMapPlanes;   //用于存储 地图中的平面特征
    std::vector<long unsigned int> mvnRemovedPlanes;
    // add plane end ----------------------------------
};


} //namespace ORB_SLAM

#endif // MAP_H
