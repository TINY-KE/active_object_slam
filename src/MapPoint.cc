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

#include "MapPoint.h"
#include "ORBmatcher.h"

#include<mutex>

namespace ORB_SLAM2
{

long unsigned int MapPoint::nNextId=0;
mutex MapPoint::mGlobalMutex;

MapPoint::MapPoint(const cv::Mat &Pos, KeyFrame *pRefKF, Map* pMap):
    mnFirstKFid(pRefKF->mnId), mnFirstFrame(pRefKF->mnFrameId), nObs(0), mnTrackReferenceForFrame(0),
    mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
    mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(pRefKF), mnVisible(1), mnFound(1), mbBad(false),
    mpReplaced(static_cast<MapPoint*>(NULL)), mfMinDistance(0), mfMaxDistance(0), mpMap(pMap)
{
    Pos.copyTo(mWorldPos);
    mNormalVector = cv::Mat::zeros(3,1,CV_32F);

    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    unique_lock<mutex> lock(mpMap->mMutexPointCreation);
    mnId=nNextId++;
}

MapPoint::MapPoint(const cv::Mat &Pos, Map* pMap, Frame* pFrame, const int &idxF):
    mnFirstKFid(-1), mnFirstFrame(pFrame->mnId), nObs(0), mnTrackReferenceForFrame(0), mnLastFrameSeen(0),
    mnBALocalForKF(0), mnFuseCandidateForKF(0),mnLoopPointForKF(0), mnCorrectedByKF(0),
    mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(static_cast<KeyFrame*>(NULL)), mnVisible(1),
    mnFound(1), mbBad(false), mpReplaced(NULL), mpMap(pMap)
{
    Pos.copyTo(mWorldPos);
    cv::Mat Ow = pFrame->GetCameraCenter();
    mNormalVector = mWorldPos - Ow;
    mNormalVector = mNormalVector/cv::norm(mNormalVector);

    mNormalVectors.push_back(mNormalVector);  //NBV MAM

    cv::Mat PC = Pos - Ow;
    const float dist = cv::norm(PC);
    const int level = pFrame->mvKeysUn[idxF].octave;
    const float levelScaleFactor =  pFrame->mvScaleFactors[level];
    const int nLevels = pFrame->mnScaleLevels;

    mfMaxDistance = dist*levelScaleFactor;
    mfMinDistance = mfMaxDistance/pFrame->mvScaleFactors[nLevels-1];

    pFrame->mDescriptors.row(idxF).copyTo(mDescriptor);

    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    unique_lock<mutex> lock(mpMap->mMutexPointCreation);
    mnId=nNextId++;
}

void MapPoint::SetWorldPos(const cv::Mat &Pos)
{
    unique_lock<mutex> lock2(mGlobalMutex);
    unique_lock<mutex> lock(mMutexPos);
    Pos.copyTo(mWorldPos);
}

cv::Mat MapPoint::GetWorldPos()
{
    unique_lock<mutex> lock(mMutexPos);
    return mWorldPos.clone();
}

cv::Mat MapPoint::GetNormal()
{
    unique_lock<mutex> lock(mMutexPos);
    return mNormalVector.clone();
}

KeyFrame* MapPoint::GetReferenceKeyFrame()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mpRefKF;
}

void MapPoint::AddObservation(KeyFrame* pKF, size_t idx)
{   // type remind --> std::map<KeyFrame*,size_t> mObservations;  观测到该MapPoint的KF和该MapPoint在KF中的索引
    unique_lock<mutex> lock(mMutexFeatures);
    if(mObservations.count(pKF))
        return;
    mObservations[pKF]=idx;

    if(pKF->mvuRight[idx]>=0)
        nObs+=2;
    else
        nObs++;
}

void MapPoint::EraseObservation(KeyFrame* pKF)
{   // 删除观测关系
    // 它的参数KeyFrame* pKF指的是关键帧
    bool bBad=false;
    {
        unique_lock<mutex> lock(mMutexFeatures);
        //  这个函数首先判断该关键帧是否在观测中，如果在，就从存放观测关系的容器mObservations中移除该关键帧，
        if(mObservations.count(pKF))
        {
            int idx = mObservations[pKF];
            if(pKF->mvuRight[idx]>=0)
                nObs-=2;
            else
                nObs--;

            mObservations.erase(pKF);
            // 接着判断该帧是否是参考关键帧，如果是，参考关键帧换成观测的第一帧，因为不能没有参考关键帧呀。
            if(mpRefKF==pKF)
                mpRefKF=mObservations.begin()->first;

            //删除以后，如果该MapPoint被观测的次数小于2，那么这个MapPoint就没有存在的必要了，需要删除。
            if(nObs<=2)
                bBad=true;
        }
    }

    if(bBad)
        SetBadFlag();  //删除地图点，并清除关键帧和地图中所有和该地图点对应的关联关系
}

map<KeyFrame*, size_t> MapPoint::GetObservations()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mObservations;
}

int MapPoint::Observations()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return nObs;
}

void MapPoint::SetBadFlag()  //删除地图点，并清除关键帧和地图中所有和该地图点对应的关联关系
{
    map<KeyFrame*,size_t> obs;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        mbBad=true;
        obs = mObservations;
        //清除该地图点所有的观测关系
        mObservations.clear();
    }
    for(map<KeyFrame*,size_t>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {   //删除关键帧中和该MapPoint对应的匹配关系
        KeyFrame* pKF = mit->first;
        pKF->EraseMapPointMatch(mit->second);
    }
    //从地图中删除MapPoint
    mpMap->EraseMapPoint(this);
}

MapPoint* MapPoint::GetReplaced()
{
    unique_lock<mutex> lock1(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    return mpReplaced;
}

void MapPoint::Replace(MapPoint* pMP)
{   //该函数的作用是将当前地图点(this)，替换成pMp，这主要是因为在使用闭环时，完成闭环优化以后，需要调整地图点和关键帧，建立新的关系。
    //具体流程是循环遍历所有的观测信息，判断此MapPoint是否在该关键帧中
    if(pMP->mnId==this->mnId)
        return;

    int nvisible, nfound;
    map<KeyFrame*,size_t> obs;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        obs=mObservations;
        mObservations.clear();
        mbBad=true;
        nvisible = mnVisible;
        nfound = mnFound;
        mpReplaced = pMP;
    }

    for(map<KeyFrame*,size_t>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        // Replace measurement in keyframe
        KeyFrame* pKF = mit->first;

        if(!pMP->IsInKeyFrame(pKF))
        {
            pKF->ReplaceMapPointMatch(mit->second, pMP);
            pMP->AddObservation(pKF,mit->second);
        }
        else
        {
            pKF->EraseMapPointMatch(mit->second);
        }
    }
    pMP->IncreaseFound(nfound);
    pMP->IncreaseVisible(nvisible);
    pMP->ComputeDistinctiveDescriptors();

    mpMap->EraseMapPoint(this);
}

bool MapPoint::isBad()
{
    unique_lock<mutex> lock(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    return mbBad;
}

void MapPoint::IncreaseVisible(int n)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mnVisible+=n;
}

void MapPoint::IncreaseFound(int n)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mnFound+=n;
}

float MapPoint::GetFoundRatio()  //MapPoint的被观测到的比例，也就是被视野范围内的相机观测到的次数除以可见次数
{
    unique_lock<mutex> lock(mMutexFeatures);
    return static_cast<float>(mnFound)/mnVisible;
    //mnFound：当前MapPoint被观测到的次数，即成功匹配的次数。 -->TrackLocalMap的时候，如果这个地图点被当前帧观测到并且匹配上了地图点，并且经过BA优化后还是内点，那么这个地图点的mnFound就+1。也就是这个变量被普通帧看到就+1.
    //mnVisible：当前MapPoint可见的次数，即被提取为特征点的次数。-->只是说能被当前帧看到，但不一定能匹配上，所以这个时候mnVisible也要+1.
}

void MapPoint::ComputeDistinctiveDescriptors()
{   //由于一个MapPoint会被许多相机观测到，因此在插入关键帧后，需要判断是否更新当前点的最适合的描述子。最好的描述子与其他描述子应该具有最小的平均距离，
    // 因此先获得当前点的所有描述子，然后计算描述子之间的两两距离，对所有距离取平均，最后找离这个中值距离最近的描述子。
    // Retrieve all observed descriptors
    vector<cv::Mat> vDescriptors;

    map<KeyFrame*,size_t> observations;

    {
        unique_lock<mutex> lock1(mMutexFeatures);
        if(mbBad)
            return;
        observations=mObservations;
    }

    if(observations.empty())
        return;

    vDescriptors.reserve(observations.size());

    for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;

        if(!pKF->isBad())
            vDescriptors.push_back(pKF->mDescriptors.row(mit->second));
    }

    if(vDescriptors.empty())
        return;

    // Compute distances between them
    const size_t N = vDescriptors.size();

    float Distances[N][N];
    for(size_t i=0;i<N;i++)
    {
        Distances[i][i]=0;
        for(size_t j=i+1;j<N;j++)
        {
            int distij = ORBmatcher::DescriptorDistance(vDescriptors[i],vDescriptors[j]);
            Distances[i][j]=distij;
            Distances[j][i]=distij;
        }
    }

    // Take the descriptor with least median distance to the rest
    int BestMedian = INT_MAX;
    int BestIdx = 0;
    for(size_t i=0;i<N;i++)
    {
        vector<int> vDists(Distances[i],Distances[i]+N);
        sort(vDists.begin(),vDists.end());
        int median = vDists[0.5*(N-1)];

        if(median<BestMedian)
        {
            BestMedian = median;
            BestIdx = i;
        }
    }

    {
        unique_lock<mutex> lock(mMutexFeatures);
        mDescriptor = vDescriptors[BestIdx].clone();
    }
}

cv::Mat MapPoint::GetDescriptor()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mDescriptor.clone();
}

int MapPoint::GetIndexInKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexFeatures);
    if(mObservations.count(pKF))
        return mObservations[pKF];
    else
        return -1;
}

bool MapPoint::IsInKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexFeatures);
    return (mObservations.count(pKF));
}

//void MapPoint::UpdateNormalAndDepth()
//{   //所谓的法向量，就是也就是说相机光心指向地图点的方向，计算这个方向方法很简单，只需要用地图点的三维坐标减去相机光心的三维坐标就可以。
//    map<KeyFrame*,size_t> observations;  //observations是观测到此point的全部关键帧 及此point的索引
//    KeyFrame* pRefKF;
//    cv::Mat Pos;
//    {
//        unique_lock<mutex> lock1(mMutexFeatures);
//        unique_lock<mutex> lock2(mMutexPos);
//        if(mbBad)
//            return;
//        observations=mObservations;
//        pRefKF=mpRefKF;
//        Pos = mWorldPos.clone();
//    }
//
//    if(observations.empty())
//        return;
//
//    cv::Mat normal = cv::Mat::zeros(3,1,CV_32F);
//    int n=0;
//    for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
//    {
//        KeyFrame* pKF = mit->first;
//        cv::Mat Owi = pKF->GetCameraCenter();
//        ///观测点坐标减去关键帧中相机光心的坐标就是观测方向   //也就是说相机光心指向地图点
//        cv::Mat normali = mWorldPos - Owi;
//        //对其进行归一化后相加. 以下的normal是个求和, 因此还得除以size()
//        normal = normal + normali/cv::norm(normali);
//        n++;
//    }
//
//    cv::Mat PC = Pos - pRefKF->GetCameraCenter();
//    const float dist = cv::norm(PC);
//    const int level = pRefKF->mvKeysUn[observations[pRefKF]].octave;
//    const float levelScaleFactor =  pRefKF->mvScaleFactors[level];
//    const int nLevels = pRefKF->mnScaleLevels;
//
//    //深度范围：地图点到参考帧（只有一帧）相机中心距离，乘上参考帧中描述子获取金字塔放大尺度
//    //得到最大距离mfMaxDistance;最大距离除以整个金字塔最高层的放大尺度得到最小距离mfMinDistance.
//    //通常说来，距离较近的地图点，将在金字塔较高的地方提出，
//    //距离较远的地图点，在金字塔层数较低的地方提取出（金字塔层数越低，分辨率越高，才能识别出远点）
//    //因此，通过地图点的信息（主要对应描述子），我们可以获得该地图点对应的金字塔层级
//    //从而预测该地图点在什么范围内能够被观测到
//    {
//        unique_lock<mutex> lock3(mMutexPos);
//        mfMaxDistance = dist*levelScaleFactor;
//        mfMinDistance = mfMaxDistance/pRefKF->mvScaleFactors[nLevels-1];
//        mNormalVector = normal/n;
//    }
//}

float MapPoint::GetMinDistanceInvariance()
{
    unique_lock<mutex> lock(mMutexPos);
    return 0.8f*mfMinDistance;
}

float MapPoint::GetMaxDistanceInvariance()
{
    unique_lock<mutex> lock(mMutexPos);
    return 1.2f*mfMaxDistance;
}
/*                      ___
    nearer            /_____\       level: n-1   --> dmin
                     /_______\                             d/dmin=1.2^(n-1-m)
                    /_________\     level: m     --> d
                   /___________\                           dmax/d=1.2^(m)
    farther       /_____________\   level: 0     --> dmax
    注意金字塔scalefator和距离的关系: 当特征点对应scalefactor为1.2的意思是: 图片分辨率下降1.2倍后, 可以提取出特征点(分辨率更高,肯定也能提出,这里取金字塔中能够提取出该特征点最高层级作为该特征点的层级).
    同时由当前特征点的距离,推测出所在的层级.
* */
int MapPoint::PredictScale(const float &currentDist, KeyFrame* pKF)
{
    float ratio;
    {
        unique_lock<mutex> lock(mMutexPos);
        ratio = mfMaxDistance/currentDist;
    }

    int nScale = ceil(log(ratio)/pKF->mfLogScaleFactor);
    if(nScale<0)
        nScale = 0;
    else if(nScale>=pKF->mnScaleLevels)
        nScale = pKF->mnScaleLevels-1;

    return nScale;
}

int MapPoint::PredictScale(const float &currentDist, Frame* pF)
{
    float ratio;
    {
        unique_lock<mutex> lock(mMutexPos);
        ratio = mfMaxDistance/currentDist;
    }

    int nScale = ceil(log(ratio)/pF->mfLogScaleFactor);
    if(nScale<0)
        nScale = 0;
    else if(nScale>=pF->mnScaleLevels)
        nScale = pF->mnScaleLevels-1;

    return nScale;
}

//NBV MAM
void MapPoint::UpdateNormalAndDepth()
{
    map<KeyFrame*,size_t> observations;
    KeyFrame* pRefKF;
    cv::Mat Pos;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        if(mbBad)
            return;
        observations=mObservations;
        pRefKF=mpRefKF;
        Pos = mWorldPos.clone();
    }

    if(observations.empty())
        return;

    cv::Mat normal = cv::Mat::zeros(3,1,CV_32F);
    int n=0;   //当前 MapPoint 所被观测到的 KeyFrame 的数量，也就是 MapPoint 在多少个 KeyFrame 中出现过。
    mNormalVectors.clear();
    theta_sVector.clear();
    for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;
        cv::Mat Owi = pKF->GetCameraCenter();
        cv::Mat normali = mWorldPos - Owi;  //计算每个观测的视线方向
        normal = normal + normali/cv::norm(normali);   //将所有视线方向求和得到平均视线方向normal
        mNormalVectors.push_back(normali/cv::norm(normali)); //MAM
        // compute the viewing angle in the world frame
        float theta = atan2(normali.at<float>(0,0),normali.at<float>(1,0));  //MAM 对每个观测视线方向计算在世界坐标系下的观察角度theta.   与y轴的夹角
        theta_sVector.push_back(theta); //MAM 并将这些角度存储在theta_sVector中。
        n++;
    }

    cv::Mat PC = Pos - pRefKF->GetCameraCenter();
    const float dist = cv::norm(PC);
    const int level = pRefKF->mvKeysUn[observations[pRefKF]].octave;
    const float levelScaleFactor =  pRefKF->mvScaleFactors[level];
    const int nLevels = pRefKF->mnScaleLevels;

    //通常说来，距离较近的地图点，将在金字塔较高的地方提出，
    //距离较远的地图点，在金字塔层数较低的地方提取出（金字塔层数越低，分辨率越高，才能识别出远点）
    //因此，通过地图点的信息（主要对应描述子），我们可以获得该地图点对应的金字塔层级
    //从而预测该地图点在什么范围内能够被观测到
    {
        unique_lock<mutex> lock3(mMutexPos);
        //深度范围：地图点到参考帧（只有一帧）相机中心距离，乘上参考帧中描述子获取金字塔放大尺度得到最大距离mfMaxDistance;
        mfMaxDistance = dist*levelScaleFactor;
        //最大距离除以整个金字塔最高层的放大尺度得到最小距离mfMinDistance.
        mfMinDistance = mfMaxDistance/pRefKF->mvScaleFactors[nLevels-1];
        mNormalVector = normal/n;  //这里的 normal 变量是所有观测到的 KeyFrame 对当前 MapPoint 计算得到的法线向量的和。将 normal 除以 n 可以得到这些法线向量的平均值，从而计算出当前 MapPoint 的平均法线向量。
        // compute the mean and std of viewing angle
        compute_std_pts(theta_sVector, theta_mean, theta_std); //MAM: 计算theta_sVector的均值和标准偏差
    }
}

void MapPoint::compute_std_pts(std::vector<float> v, float & mean, float & stdev)
{
    //v中是各参考关键帧对point的观测角度

    float sum = std::accumulate(v.begin(), v.end(), 0.0);  //所有theta的和
    mean = sum / v.size(); //theta的均值

    std::vector<float> diff(v.size());
    std::transform(v.begin(), v.end(), diff.begin(),
                std::bind2nd(std::minus<float>(), mean));  // 计算每个theta与平均值的差 diff，
    float sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);  //计算所有diff的平方和, 得sq_sum
    stdev = std::sqrt(sq_sum / v.size());  //计算所有diff的标准偏差
}
//NBV MAM end

} //namespace ORB_SLAM
