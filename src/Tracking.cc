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


#include "Tracking.h"
#include "Eigen/Dense"
#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>
#include<opencv2/core/eigen.hpp>
#include"ORBmatcher.h"
#include"FrameDrawer.h"
#include"Converter.h"
#include"Map.h"
#include"Initializer.h"
#include"Object.h"
#include"Optimizer.h"
#include"PnPsolver.h"

#include<iostream>

#include<mutex>
#include <unistd.h>

//NBV MAM
#include<camera.h>

using namespace std;

namespace ORB_SLAM2
{

Tracking::Tracking(System *pSys, ORBVocabulary* pVoc, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Map *pMap, KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor, MapPublisher*  pMapPublisher):
    mState(NO_IMAGES_YET), mSensor(sensor), mbOnlyTracking(false), mbVO(false), mpORBVocabulary(pVoc),
    mpKeyFrameDB(pKFDB), mpInitializer(static_cast<Initializer*>(NULL)), mpSystem(pSys), mpViewer(NULL), mpNbvGenerator(NULL),
    mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpMap(pMap), mnLastRelocFrameId(0),  mpMapPublisher(pMapPublisher)
{
    // Load camera parameters from settings file
    mStrSettingPath = strSettingPath;
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    // [kinect dk ]
    const float k4 = fSettings["Camera.k4"];
    if(k4!=0)
    {
        DistCoef.resize(6);
        DistCoef.at<float>(5) = k4;
    }
    const float k5 = fSettings["Camera.k5"];
    if(k5!=0)
    {
        DistCoef.resize(7);
        DistCoef.at<float>(6) = k5;
    }
    const float k6 = fSettings["Camera.k6"];
    if(k6!=0)
    {
        DistCoef.resize(8);
        DistCoef.at<float>(7) = k6;
    }
    
    DistCoef.copyTo(mDistCoef);

    mbf = fSettings["Camera.bf"];

    float fps = fSettings["Camera.fps"];
    if(fps==0)
        fps=30;

    // Max/Min Frames to insert keyframes and to check relocalisation
    mMinFrames = 0;
    mMaxFrames = fps;

    cout << endl << "Camera Parameters: " << endl;
    cout << "- fx: " << fx << endl;
    cout << "- fy: " << fy << endl;
    cout << "- cx: " << cx << endl;
    cout << "- cy: " << cy << endl;
    cout << "- k1: " << DistCoef.at<float>(0) << endl;
    cout << "- k2: " << DistCoef.at<float>(1) << endl;
    if(DistCoef.rows==5)
        cout << "- k3: " << DistCoef.at<float>(4) << endl;
    if(DistCoef.rows==8)
    {
        cout << "- k3: " << DistCoef.at<float>(4) << endl;
        cout << "- k4: " << DistCoef.at<float>(5) << endl;
        cout << "- k5: " << DistCoef.at<float>(6) << endl;
        cout << "- k6: " << DistCoef.at<float>(7) << endl;
    }
    cout << "- p1: " << DistCoef.at<float>(2) << endl;
    cout << "- p2: " << DistCoef.at<float>(3) << endl;
    cout << "- fps: " << fps << endl;


    int nRGB = fSettings["Camera.RGB"];
    mbRGB = nRGB;

    if(mbRGB)
        cout << "- color order: RGB (ignored if grayscale)" << endl;
    else
        cout << "- color order: BGR (ignored if grayscale)" << endl;

    // Load ORB parameters

    int nFeatures = fSettings["ORBextractor.nFeatures"];
    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    int nLevels = fSettings["ORBextractor.nLevels"];
    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    int fMinThFAST = fSettings["ORBextractor.minThFAST"];

    mpORBextractorLeft = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    if(sensor==System::STEREO)
        mpORBextractorRight = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    if(sensor==System::MONOCULAR)
        mpIniORBextractor = new ORBextractor(2*nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    // STEP line detect +++++++++++++++++++++++++++++++++++++++++++
    bool use_LSD_algorithm = false;
    bool save_to_imgs = false;
    bool save_to_txts = false;
    int numOfOctave_ = 1;
    float Octave_ratio = 2.0;
    line_lbd_ptr = new line_lbd_detect(numOfOctave_, Octave_ratio);
    line_lbd_ptr->use_LSD = use_LSD_algorithm;
    line_lbd_ptr->save_imgs = save_to_imgs;
    line_lbd_ptr->save_txts = save_to_txts;
    line_lbd_ptr->line_length_thres = 15; // the threshold of removing short line.
    // line detect ------------------------------------------------

    cout << endl  << "ORB Extractor Parameters: " << endl;
    cout << "- Number of Features: " << nFeatures << endl;
    cout << "- Scale Levels: " << nLevels << endl;
    cout << "- Scale Factor: " << fScaleFactor << endl;
    cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
    cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;

    if(sensor==System::STEREO || sensor==System::RGBD)
    {
        mThDepth = mbf*(float)fSettings["ThDepth"]/fx;
        cout << endl << "Depth Threshold (Close/Far Points): " << mThDepth << endl;
    }

    if(sensor==System::RGBD)
    {
        mDepthMapFactor = fSettings["DepthMapFactor"];
        if(fabs(mDepthMapFactor)<1e-5)
            mDepthMapFactor=1;
        else
            mDepthMapFactor = 1.0f/mDepthMapFactor;
    }

    //NBV MAM
    //float qx = fSettings["Trobot_camera.qx"], qy = fSettings["Trobot_camera.qy"], qz = fSettings["Trobot_camera.qz"], qw = fSettings["Trobot_camera.qw"],
    //      tx = fSettings["Trobot_camera.tx"], ty = fSettings["Trobot_camera.ty"], tz = fSettings["Trobot_camera.tz"];
    //mT_baselink_cam = Converter::Quation2CvMat(qx, qy, qz, qw, tx, ty, tz );
    //mDivide = fSettings["MAM.divide"];
    //NBV MAM END
}

void Tracking::SetLocalMapper(LocalMapping *pLocalMapper)
{
    mpLocalMapper=pLocalMapper;
}

void Tracking::SetLoopClosing(LoopClosing *pLoopClosing)
{
    mpLoopClosing=pLoopClosing;
}

void Tracking::SetViewer(Viewer *pViewer)
{
    mpViewer=pViewer;
}


cv::Mat Tracking::GrabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const double &timestamp)
{
    return mCurrentFrame.mTcw.clone();
}


cv::Mat Tracking::GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp, const std::vector<BoxSE> & bbox)
{
    mImGray = imRGB;
    cv::Mat imDepth = imD;
    // cvtColor(mImRGB,mImRGB,CV_BGRA2BGR);//[kinect dk] 这一行有没有用？？

    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
    }

    if((fabs(mDepthMapFactor-1.0f)>1e-5) || imDepth.type()!=CV_32F)
        imDepth.convertTo(imDepth,CV_32F,mDepthMapFactor);

    //(1)存入物体语义的识别信息和彩色图像
    mCurrentFrame = Frame( imRGB,mImGray,imDepth,timestamp,mpORBextractorLeft,line_lbd_ptr,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth,bbox);
    //mCurrentFrame.boxes = bbox;
    //mCurrentFrame.addColorImg(imRGB);

    //(2)初始帧的位姿
    cv::FileStorage fSettings(mStrSettingPath, cv::FileStorage::READ);
    int ConstraintType = fSettings["ConstraintType"];
    if ( ConstraintType != 1 && ConstraintType != 2 && ConstraintType != 3){
        std::cerr << ">>>>>> [WARRNING] USE NO PARAM CONSTRAINT TYPE!" << std::endl;
        ConstraintType = 1;
    }
    if (ConstraintType == 1){// robot_camera tf
        float qx = fSettings["Tworld_camera.qx"], qy = fSettings["Tworld_camera.qy"], qz = fSettings["Tworld_camera.qz"], qw = fSettings["Tworld_camera.qw"],
                tx = fSettings["Tworld_camera.tx"], ty = fSettings["Tworld_camera.ty"], tz = fSettings["Tworld_camera.tz"];
         //float qx = fSettings["Tgroud_firstcamera.qx"], qy = fSettings["Tgroud_firstcamera.qy"], qz = fSettings["Tgroud_firstcamera.qz"], qw = fSettings["Tgroud_firstcamera.qw"],
         //       tx = fSettings["Tgroud_firstcamera.tx"], ty = fSettings["Tgroud_firstcamera.ty"], tz = fSettings["Tgroud_firstcamera.tz"];
        mCurrentFrame.mGroundtruthPose_mat = cv::Mat::eye(4, 4, CV_32F);
        Eigen::Quaterniond quaternion(Eigen::Vector4d(qx, qy, qz, qw));
        Eigen::AngleAxisd rotation_vector(quaternion);
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        T.rotate(rotation_vector);
        T.pretranslate(Eigen::Vector3d(tx, ty, tz));
        Eigen::Matrix4d GroundtruthPose_eigen = T.matrix();
        cv::Mat cv_mat_32f;
        cv::eigen2cv(GroundtruthPose_eigen, cv_mat_32f);
        cv_mat_32f.convertTo(mCurrentFrame.mGroundtruthPose_mat, CV_32F);

    } else if(ConstraintType == 2){
        // TODO: IMU
    } else if (ConstraintType == 3){// ros tf
        tf::TransformListener listener;
        tf::StampedTransform transform;
        cv::Mat T_w_camera = cv::Mat::eye(4,4,CV_32F);
        try
        {
            listener.waitForTransform("/map", "/camera_depth_optical_frame", ros::Time(0), ros::Duration(1.0));
            listener.lookupTransform("/map", "/camera_depth_optical_frame", ros::Time(0), transform);
            T_w_camera = Converter::Quation2CvMat(
                            transform.getRotation().x(),
                            transform.getRotation().y(),
                            transform.getRotation().z(),
                            transform.getRotation().w(),
                            transform.getOrigin().x(),
                            transform.getOrigin().y(),
                            transform.getOrigin().z()
                    );
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s -->> lost tf from /map to /base_footprint",ex.what());
        }

        mCurrentFrame.mGroundtruthPose_mat = T_w_camera;
    }

    //(3) 开始执行track
    Track();

    return mCurrentFrame.mTcw.clone();
}


cv::Mat Tracking::GrabImageMonocular(const cv::Mat &im, const double &timestamp)
{
    return mCurrentFrame.mTcw.clone();
}

void Tracking::Track()
{
    if(mState==NO_IMAGES_YET)
    {
        mState = NOT_INITIALIZED;
    }

    mLastProcessedState=mState;

    // Get Map Mutex -> Map cannot be changed
    unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

    if(mState==NOT_INITIALIZED)
    {
        if(mSensor==System::STEREO || mSensor==System::RGBD)
            StereoInitialization();
        else
            MonocularInitialization();

        mpFrameDrawer->Update(this);

        if(mState!=OK)
            return;
    }
    else
    {
        // System is initialized. Track Frame.
        bool bOK;

        // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
        {
            // Local Mapping is activated. This is the normal behaviour, unless
            // you explicitly activate the "only tracking" mode.

            if(mState==OK)
            {
                // Local Mapping might have changed some MapPoints tracked in last frame
                CheckReplacedInLastFrame();   //对mLastFrame中的所有point, 更改relocal的结果

                if(mVelocity.empty() || mCurrentFrame.mnId<mnLastRelocFrameId+2)    //如果没有卡尔曼滤波所需的速度 或者 刚开始relocal不久. 则执行trackReference
                {
                    bOK = TrackReferenceKeyFrame();
                }
                else  //否则,进行卡尔曼滤波
                {
                    bOK = TrackWithMotionModel();
                    if(!bOK){   //如果卡尔曼滤波失败, 则再 通过trackReference
                        bOK = TrackReferenceKeyFrame();
                    }
                }
            }
            else
            {
                bOK = Relocalization();
            }
        }

        mCurrentFrame.mpReferenceKF = mpReferenceKF;

        // 如果trackmotion 或者trackReference 成功了(bOK==TRUE). 那么我们就获得了相机的粗略位姿. 接下来通过TrackLocalMap ,获得更精确的位姿.
        // If we have an initial estimation of the camera pose and matching. Track the local map.
        {
            if(bOK)
                bOK = TrackLocalMap();
        }

        //如果 TrackLocalMap 也成功了, 是最好的结果
        if(bOK)
            mState = OK;
        else
            mState=LOST;

        // Update drawer
        mpFrameDrawer->Update(this);

        //如果 TrackLocalMap 也成功了, 那可以考虑生成关键帧
        // If tracking were good, check if we insert a keyframe
        if(bOK)
        {
            // add plane -------------------
            // Update Planes
            // std::cout << "[DEBUG] mnPlaneNum is " << mCurrentFrame.mnPlaneNum << std::endl;
            for (int i = 0; i < mCurrentFrame.mnPlaneNum; ++i)
            {
                MapPlane *pMPl = mCurrentFrame.mvpMapPlanes[i];
                if (pMPl && pMPl->mbSeen)
                {
                    // 更新边界点
                    // TODO: 为什么不更新新生成的平面？
                    pMPl->UpdateBoundary(mCurrentFrame, i);
                }
                else
                {
                    // mCurrentFrame.mbNewPlane = true;
                }
            }
            // add plane end -----------------

            // Update motion model
            if(!mLastFrame.mTcw.empty())
            {
                cv::Mat LastTwc = cv::Mat::eye(4,4,CV_32F);
                mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0,3).colRange(0,3));
                mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0,3).col(3));
                mVelocity = mCurrentFrame.mTcw*LastTwc;
            }
            else
                mVelocity = cv::Mat();

            mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);
            mpMapPublisher->SetCurrentCameraPose(mCurrentFrame.mTcw);

            // Clean VO matches
            for(int i=0; i<mCurrentFrame.N; i++)
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(pMP)
                    if(pMP->Observations()<1)
                    {
                        mCurrentFrame.mvbOutlier[i] = false;
                        mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                    }
            }

            // Delete temporal MapPoints
            for(list<MapPoint*>::iterator lit = mlpTemporalPoints.begin(), lend =  mlpTemporalPoints.end(); lit!=lend; lit++)
            {
                MapPoint* pMP = *lit;
                delete pMP;
            }
            mlpTemporalPoints.clear();

            // Check if we need to insert a new keyframe
            if (NeedNewKeyFrame() == 1)
                CreateNewKeyFrame(false);
            else if (NeedNewKeyFrame() == 2)    // note [EAO] create keyframes by the new object.
            {
                CreateNewKeyFrame(true);
            }

            // We allow points with high innovation (considererd outliers by the Huber Function)
            // pass to the new keyframe, so that bundle adjustment will finally decide
            // if they are outliers or not. We don't want next frame to estimate its position
            // with those points so we discard them in the frame.
            for(int i=0; i<mCurrentFrame.N;i++)
            {
                if(mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
                    mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
            }
        }

        //如果 TrackLocalMap 失败了,并且刚初始化不久. 则重新开始建图
        // Reset if the camera get lost soon after initialization
        if(mState==LOST)
        {
            if(mpMap->KeyFramesInMap()<=5)
            {
                cout << "Track lost soon after initialisation, reseting..." << endl;
                mpSystem->Reset();
                return;
            }
        }

        // 最后,如果当前帧的参考关键帧 不存在, 则设置为当前的参考关键帧. 将当前帧成为 last帧.  开始处理下一帧
        if(!mCurrentFrame.mpReferenceKF)
            mCurrentFrame.mpReferenceKF = mpReferenceKF;
        mLastFrame = Frame(mCurrentFrame);
    }

    // Store frame pose information to retrieve the complete camera trajectory afterwards.
    if(!mCurrentFrame.mTcw.empty())
    {
        cv::Mat Tcr = mCurrentFrame.mTcw*mCurrentFrame.mpReferenceKF->GetPoseInverse();
        mlRelativeFramePoses.push_back(Tcr);
        mlpReferences.push_back(mpReferenceKF);
        mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
        mlbLost.push_back(mState==LOST);
    }
    // 如果当前帧的位姿为空(应该是track失败了),则mlRelativeFramePoses mlpReferences mlFrameTimes继承上一帧的结果.
    else
    {
        // This can happen if tracking is lost
        mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
        mlpReferences.push_back(mlpReferences.back());
        mlFrameTimes.push_back(mlFrameTimes.back());
        mlbLost.push_back(mState==LOST);
    }

}


void Tracking::StereoInitialization()
{
    if(mCurrentFrame.N>500)
    {
        // Set Frame pose to the origin
        mCurrentFrame.SetPose(cv::Mat::eye(4,4,CV_32F));

        // Create KeyFrame
        KeyFrame* pKFini = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);

        // Insert KeyFrame in the map
        mpMap->AddKeyFrame(pKFini);

        //(1)将第一帧,  转为相对地面的真实位姿
        cv::Mat Worldframe_to_Firstframe = mCurrentFrame.mGroundtruthPose_mat;
        cv::Mat R = Worldframe_to_Firstframe.rowRange(0, 3).colRange(0, 3);
        cv::Mat t = Worldframe_to_Firstframe.rowRange(0, 3).col(3);
        cv::Mat Rinv = R.t();
        cv::Mat Ow = -Rinv * t;
        cv::Mat Firstframe_to_Worldframe = cv::Mat::eye(4, 4, CV_32F);
        Rinv.copyTo (Firstframe_to_Worldframe.rowRange(0, 3).colRange(0, 3));
        Ow.copyTo   (Firstframe_to_Worldframe.rowRange(0, 3).col(3));
        pKFini->SetPose(pKFini->GetPose() * Firstframe_to_Worldframe);

        //(2)将第一帧内部的point,  转为相对地面的真实位姿, 并添加到map中
        // Create MapPoints and asscoiate to KeyFrame
        for(int i=0; i<mCurrentFrame.N;i++)
        {
            float z = mCurrentFrame.mvDepth[i];
            if(z>0)
            {
                cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                MapPoint* pNewMP = new MapPoint(x3D,pKFini,mpMap);
                pNewMP->AddObservation(pKFini,i);
                pKFini->AddMapPoint(pNewMP,i);
                pNewMP->ComputeDistinctiveDescriptors();
                pNewMP->UpdateNormalAndDepth();
                pNewMP->SetWorldPos( Worldframe_to_Firstframe.rowRange(0, 3).colRange(0, 3)  * pNewMP->GetWorldPos()
                                                + Worldframe_to_Firstframe.rowRange(0, 3).col(3)  );
                mpMap->AddMapPoint(pNewMP);
                mCurrentFrame.mvpMapPoints[i]=pNewMP;
            }
        }
        cout << "New map created with " << mpMap->MapPointsInMap() << " points" << endl;

        //(3)当前帧 添加到localmap中
        mpLocalMapper->InsertKeyFrame(pKFini);
        mCurrentFrame.SetPose(pKFini->GetPose());
        mLastFrame = Frame(mCurrentFrame);
        mnLastKeyFrameId=mCurrentFrame.mnId;
        mpLastKeyFrame = pKFini;

        mvpLocalKeyFrames.push_back(pKFini);
        mvpLocalMapPoints=mpMap->GetAllMapPoints();
        mpReferenceKF = pKFini;
        mCurrentFrame.mpReferenceKF = pKFini;

        mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

        mpMap->mvpKeyFrameOrigins.push_back(pKFini);

        mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);
        mpMapPublisher->SetCurrentCameraPose(mCurrentFrame.mTcw);

        mState=OK;
    }
}

void Tracking::MonocularInitialization()
{

}

void Tracking::CreateInitialMapMonocular()
{

}

void Tracking::CheckReplacedInLastFrame()
{
    for(int i =0; i<mLastFrame.N; i++)
    {
        MapPoint* pMP = mLastFrame.mvpMapPoints[i];

        if(pMP)
        {
            MapPoint* pRep = pMP->GetReplaced();
            if(pRep)
            {
                mLastFrame.mvpMapPoints[i] = pRep;
            }
        }
    }
}


bool Tracking::TrackReferenceKeyFrame()
{
    // Compute Bag of Words vector
    mCurrentFrame.ComputeBoW();

    // We perform first an ORB matching with the reference keyframe
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.7,true);
    vector<MapPoint*> vpMapPointMatches;

    int nmatches = matcher.SearchByBoW(mpReferenceKF,mCurrentFrame,vpMapPointMatches);

    if(nmatches<15)
        return false;

    mCurrentFrame.mvpMapPoints = vpMapPointMatches;
    mCurrentFrame.SetPose(mLastFrame.mTcw);   //TrackReference中的当前位姿设置为  等于lastframe的位姿. 纯靠g2o的计算??

    // add plane
    mpMap->AssociatePlanesByBoundary(mCurrentFrame);
    // add plane end

    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                nmatchesMap++;
        }
    }

    // add plane
    int nDisgardPlane = 0;
    for (int i = 0; i < mCurrentFrame.mnPlaneNum; i++)
    {
        if (mCurrentFrame.mvpMapPlanes[i])
        {
            if (mCurrentFrame.mvpMapPlanes[i] != nullptr && mCurrentFrame.mvbPlaneOutlier[i])
            {
                mCurrentFrame.mvpMapPlanes[i] = static_cast<MapPlane *>(NULL);
                nmatches--;
                nDisgardPlane++;
            }
            else
                nmatchesMap++;
        }
    }

    return nmatchesMap>=10;
}

void Tracking::UpdateLastFrame_fortrackmotion()
{
    // Update pose according to reference keyframe
    // step1 根据参考关键帧确定当前帧的位姿，使用参考关键帧是因为参考关键帧的位姿更加准确
    // 把上一帧的参考关键帧，作为当前帧的参考帧【但不一定是参考关键帧?】
    KeyFrame* pRef = mLastFrame.mpReferenceKF;
    // 参考关键帧->上一帧  的变换关系
    cv::Mat Tlr = mlRelativeFramePoses.back();
    //  Tlw = Tlr * Trw 三维的
    mLastFrame.SetPose(Tlr*pRef->GetPose());

    if(mnLastKeyFrameId==mLastFrame.mnId || mSensor==System::MONOCULAR || !mbOnlyTracking)
        return;   //此return必定被执行. 因此本程序不会使用"纯定位模式".

    // 后面已删除
}

bool Tracking::TrackWithMotionModel()
{
    ORBmatcher matcher(0.9,true);

    // Update last frame pose according to its reference keyframe
    // Create "visual odometry" points 【if in Localization Mode 】
    // 因此对本程序而言，只会更新下last帧的位姿
    UpdateLastFrame_fortrackmotion();

    mCurrentFrame.SetPose(mVelocity*mLastFrame.mTcw);

    fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));

    // Project points seen in previous frame。If few matches, uses a 【wider window search】
    int th;
    if(mSensor!=System::STEREO)
        th=15;
    else
        th=7;
    int nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,th,mSensor==System::MONOCULAR);
    if(nmatches<20)
    {
        fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));
        nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,2*th,mSensor==System::MONOCULAR);
    }
    if(nmatches<20)
        return false;

    // 平面的数据关联
    mpMap->AssociatePlanesByBoundary(mCurrentFrame);

     //[active slam]
    // TODO:  要不要把这一步放在【Discard outliers 丢弃异常值】后面。放在后面，会有部分mvpMapPoints[i]被清空，会不会影响物体的聚类
    // 但是如果要让object，参与优化，则放在Optimizer::PoseOptimization之前
    // 还是放在前面这。 但这样得记得， 时常检查outlier和 TODO: bad
    CreatObject_intrackmotion();

    // Optimize frame pose with all matches
    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers 丢弃异常值
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {   // mvbOutlier，它每一位表示对应索引的cv::KeyPoint 匹配到的地图点是否是外点(即根据地图点3D位置，当前帧位姿理论上看不到这个地图点，在相机视野之外)
            if(mCurrentFrame.mvbOutlier[i])
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;   //为什么要改回false？  这我还怎么区分outlier
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                nmatchesMap++;
        }
    }

    // add plane: 删除outlier平面
    int nDisgardPlane = 0;
    for (int i = 0; i < mCurrentFrame.mnPlaneNum; i++) {
        if (mCurrentFrame.mvpMapPlanes[i]) {
            if (mCurrentFrame.mvpMapPlanes[i]!= nullptr && mCurrentFrame.mvbPlaneOutlier[i])
            {   // 如果plane存在,且是outllier, 则将它清空
                mCurrentFrame.mvpMapPlanes[i] = static_cast<MapPlane *>(NULL);
                nmatches--;
                nDisgardPlane++;
            } else
                nmatchesMap++;
        }
    }
    // add plane end

    return nmatchesMap>=10;
}

// TODO: 这个函数应该放在object的构造函数中
void Tracking::CreatObject_intrackmotion(){

    // *****************************
    // STEP 1. construct 2D object *
    // *****************************
    vector<Object_2D *> obj_2ds;
    cv::Mat ColorImage = mCurrentFrame.mColorImage.clone();
    for (auto &box : mCurrentFrame.boxes)
    {
        Object_2D *obj2d = new Object_2D( mpMap, &mCurrentFrame, box);  //(Map* Map, Frame* CurrentFrame, const BoxSE &box)
        obj_2ds.push_back(obj2d);
    }
    // ***************************************
    // STEP 2. associate objects with points * 点
    // ***************************************
    //AssociateObjAndPoints(objs_2ds);
    for (int i = 0; i < mCurrentFrame.N; i++)
    {
        if (mCurrentFrame.mvpMapPoints[i])
        {
            MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];
            if (pMP->isBad())
                continue;

            for (size_t k = 0; k < obj_2ds.size(); ++k)
            {
                if (obj_2ds[k]->mBox_cvRect.contains(mCurrentFrame.mvKeysUn[i].pt))// in rect.
                {
                    //pMP->object_view = true;                  // the point is associated with an object.
                    //TODO: 存储的是pMP在当前帧的uv坐标。但是会不会在其他帧中，业匹配到了obj2d->mvMapPonits，但uv坐标没修改
                    //答： 不会。因此就是根据uv坐标，存入的obj2d->mvMapPonits。 所以必会被修改
                    pMP->feature_uvCoordinate = mCurrentFrame.mvKeysUn[i]; // coordinate in current frame.
                    obj_2ds[k]->AddObjectPoint(pMP);
                }
            }
        }
    }

    // ***************************************
    // STEP 3. associate objects with lines *  线
    // ***************************************
    // all lines in current frame.
    Eigen::MatrixXd AllLinesEigen = mCurrentFrame.all_lines_eigen;

    // step 1 make sure edges start from left to right.
    align_left_right_edges(AllLinesEigen);

    for(int i = 0; i < obj_2ds.size(); i++)
    {
        Object_2D* obj = obj_2ds[i];

        // step 2. expand the bounding box.
        double dLeftExpand = max(0.0, obj->mBox_cvRect.x - 15.0);
        double dRightExpand = min(mCurrentFrame.mColorImage.cols, obj->mBox_cvRect.x + obj->mBox_cvRect.width + 15);
        double dTopExpand = max(0.0, obj->mBox_cvRect.y - 15.0);
        double dBottomExpand = min(mCurrentFrame.mColorImage.rows, obj->mBox_cvRect.y + obj->mBox_cvRect.height + 15);
        Eigen::Vector2d ExpanLeftTop = Eigen::Vector2d(dLeftExpand, dTopExpand);			// lefttop.         bbox的左上角坐标
		Eigen::Vector2d ExpanRightBottom = Eigen::Vector2d(dRightExpand, dBottomExpand);  // rightbottom.     bbox的右下角坐标

        // step 3. 将物体和线关联在一起
        // associate object with lines.
        Eigen::MatrixXd ObjectLines(AllLinesEigen.rows(),AllLinesEigen.cols());
		int nInsideLinesNum = 0;
		for (int line_id = 0; line_id < AllLinesEigen.rows(); line_id++)   /* 当前帧中，第line_id条直线 */
        {
            // check endpoints of the lines, whether inside the box.
            if (check_inside_box(   AllLinesEigen.row(line_id).head<2>(),
                                    ExpanLeftTop,
                                    ExpanRightBottom ))
            {
                if(check_inside_box(AllLinesEigen.row(line_id).tail<2>(),
                                    ExpanLeftTop,
                                    ExpanRightBottom ))
                {
                    ObjectLines.row(nInsideLinesNum) = AllLinesEigen.row(line_id); /* 第line_id条直线属于第i个bbox，因此将这条线插入到ObjectLines集合中。 */
                    nInsideLinesNum++;
                }
            }
        }

        // step 4. merge lines.
        double pre_merge_dist_thre = 20;
		double pre_merge_angle_thre = 5;
		double edge_length_threshold = 30;
	    Eigen::MatrixXd ObjectLinesAfterMerge;
		merge_break_lines(	ObjectLines.topRows(nInsideLinesNum),
							ObjectLinesAfterMerge, 		// output lines after merge.
							pre_merge_dist_thre,		// the distance threshold between two line, 20 pixels.
							pre_merge_angle_thre, 		// angle threshold between two line, 5°.
							edge_length_threshold);		// length threshold, 30 pixels.

        // step 5. save object lines.
        obj->mObjLinesEigen = ObjectLinesAfterMerge;  //linedebug
        mCurrentFrame.vObjsLines.push_back(ObjectLinesAfterMerge);   /* vObjsLines的容量等于objs_2d.size()，  如果vObjsLines某一位为空，则代表这个物体没有关联到line。 */
    }


    // ***************************************************
    // STEP 4.
    // (1)compute the mean and standard of points.*
    // (2)Erase outliers (camera frame) by boxplot.*
    // **************************************************
    for (auto &obj2d : obj_2ds)
    {
        // compute the mean and standard.
        obj2d->ComputeMeanAndDeviation();

        // If the object has too few points, ignore.
        if (obj2d->mvMapPonits.size() < 8)
            continue;

        // Erase outliers by boxplot.
        obj2d->RemoveOutlier_ByHeightorDepth();
        obj2d->ComputeMeanAndDeviation();
    }

    // **************************************************************************
    // STEP 5. construct the bounding box by object feature points in the image.*
    // **************************************************************************
    // bounding box detected by yolo |  bounding box constructed by object points.
    //  _______________                 //  _____________
    // |   *        *  |                // |   *        *|
    // |    *  *       |                // |    *  *     |
    // |*      *  *    |                // |*      *  *  |
    // | *   *    *    |                // | *   *    *  |
    // |   *       *   |                // |___*_______*_|
    // |_______________|
    const cv::Mat Rcw = mCurrentFrame.mTcw.rowRange(0, 3).colRange(0, 3);
    const cv::Mat tcw = mCurrentFrame.mTcw.rowRange(0, 3).col(3);
    for (auto &obj2d : obj_2ds)
    {
        // record the coordinates of each point in the xy(uv) directions.
        vector<float> x_pt;
        vector<float> y_pt;
        for (auto &pMP : obj2d->mvMapPonits)
        {
            float u = pMP->feature_uvCoordinate.pt.x;
            float v = pMP->feature_uvCoordinate.pt.y;

            x_pt.push_back(u);
            y_pt.push_back(v);
        }

        if (x_pt.size() < 4) // ignore.
            continue;

        // extremum in xy(uv) direction
        sort(x_pt.begin(), x_pt.end());
        sort(y_pt.begin(), y_pt.end());
        float x_min = x_pt[0];
        float x_max = x_pt[x_pt.size() - 1];
        float y_min = y_pt[0];
        float y_max = y_pt[y_pt.size() - 1];

        // make insure in the image.
        if (x_min < 0)
            x_min = 0;
        if (y_min < 0)
            y_min = 0;
        if (x_max > ColorImage.cols)
            x_max = ColorImage.cols;
        if (y_max > ColorImage.rows)
            y_max = ColorImage.rows;

        // the bounding box constructed by object feature points.
        // notes: 视野范围内的特征点
        // 用于data associate中
        obj2d->mBox_cvRect_FeaturePoints = cv::Rect(x_min, y_min, x_max - x_min, y_max - y_min);
    }

    // **********************************************************************************************
    // STEP 6. remove 2d bad bounding boxes.
    // Due to the complex scene and Yolo error detection, some poor quality objects need to be removed.
    // The strategy can be adjusted and is not unique, such as:
    // 1. objects overlap with too many object;
    // 2. objects with too few points;
    // 3. objects with too few points and on the edge of the image;
    // 4. objects too large and take up more than half of the image;
    // and so on ......
    // **********************************************************************************************
    // overlap with too many objects.
    for (size_t f = 0; f < obj_2ds.size(); ++f)
    {
        int num = 0;
        for (size_t l = 0; l < obj_2ds.size(); ++l)
        {
            if (f == l)
                continue;

            if (Converter::bboxOverlapratioLatter(obj_2ds[f]->mBox_cvRect, obj_2ds[l]->mBox_cvRect) > 0.05)
                num++;
        }
        // overlap with more than 3 objects.  和三个物体有重叠
        if (num > 4)
            obj_2ds[f]->bad = true;
    }
    for (size_t f = 0; f < obj_2ds.size(); ++f)
    {
        if (obj_2ds[f]->bad)
            continue;

        // // 忽略错误的物体识别 ignore the error detect by yolo.
        // if ((objs_2d[f]->_class_id == 0) || (objs_2d[f]->_class_id == 63) || (objs_2d[f]->_class_id == 15))
        //     objs_2d[f]->bad = true;

        // 面积太大 too large in the image.
        if ((float)obj_2ds[f]->mBox_cvRect.area() / (float)(ColorImage.cols * ColorImage.rows) > 0.5)
            obj_2ds[f]->bad = true;

        // 特征点太少 too few object points.
        if (obj_2ds[f]->mvMapPonits.size() < 5)
            obj_2ds[f]->bad = true;

        // 特征点稍微多一些，但是检测框离边太近了   object points too few and the object on the edge of the image.
        else if  (obj_2ds[f]->mvMapPonits.size() < 10)
        {
            if ((obj_2ds[f]->mBox_cvRect.x < 20) || (obj_2ds[f]->mBox_cvRect.y < 20) ||
                (obj_2ds[f]->mBox_cvRect.x + obj_2ds[f]->mBox_cvRect.width > ColorImage.cols - 20) ||
                (obj_2ds[f]->mBox_cvRect.y + obj_2ds[f]->mBox_cvRect.height > ColorImage.rows - 20))
            {
                obj_2ds[f]->bad = true;
            }
        }

        // 如果两者重叠很大。则只保留一个。when the overlap is large, only one object remains.
        for (size_t l = 0; l < obj_2ds.size(); ++l)
        {
            if (obj_2ds[l]->bad)
                continue;

            if (f == l)
                continue;

            // retain objects which with high probability.
            if (Converter::bboxOverlapratio(obj_2ds[f]->mBox_cvRect, obj_2ds[l]->mBox_cvRect) > 0.3)
            {
                if (obj_2ds[f]->mScore < obj_2ds[l]->mScore)
                    obj_2ds[f]->bad = true;
                else if (obj_2ds[f]->mScore >= obj_2ds[l]->mScore)
                    obj_2ds[l]->bad = true;
            }
            // if one object surrounds another, keep the larger one.
            if (Converter::bboxOverlapratio(obj_2ds[f]->mBox_cvRect, obj_2ds[l]->mBox_cvRect) > 0.05)
            {
                if (Converter::bboxOverlapratioFormer(obj_2ds[f]->mBox_cvRect, obj_2ds[l]->mBox_cvRect) > 0.85)
                    obj_2ds[f]->bad = true;
                if (Converter::bboxOverlapratioLatter(obj_2ds[f]->mBox_cvRect, obj_2ds[l]->mBox_cvRect) > 0.85)
                    obj_2ds[l]->bad = true;
            }
        }
    }

    // erase the bad object.
    vector<Object_2D *>::iterator it;
    for (it = obj_2ds.begin(); it != obj_2ds.end(); )
    {
        if ((*it)->bad == true)
            it = obj_2ds.erase(it); // erase.
        else
            ++it;
    }



    // *************************************************************
    // STEP 7. copy objects in the last frame after initialization.*
    // *************************************************************
    if ((mbObjectIni == true) && (mCurrentFrame.mnId > mnObjectIniFrameID))
    {
        // 上一帧的object_2d信息   copy objects in the last frame.
        mCurrentFrame.mvLastObject_2ds = mLastFrame.mvObject_2ds;

        // 上上一帧的object_2d信息    copy objects in the penultimate frame.
        if (!mLastFrame.mvLastObject_2ds.empty())
            mCurrentFrame.mvLastLastObject_2ds = mLastFrame.mvLastObject_2ds;
    }


    // *******************************************************************************
    // STEP 8. Merges objects with 5-10 points  between two adjacent frames.
    // Advantage: Small objects with too few points, can be merged to keep them from being eliminated.
    // (The effect is not very significant.)
    // 将上一帧中的物体检测框中的点, 根据iou复制到对应的obj2d中.
    // 但是想问 mvLastLastObject_2ds 是哪一个物体啊???
    // *******************************************************************************
    bool bMergeTwoObj = true;
    if ((!mCurrentFrame.mvLastObject_2ds.empty()) && bMergeTwoObj)
    {
        // object in current frame.
        for (size_t k = 0; k < obj_2ds.size(); ++k)
        {
            // ignore objects with more than 10 points.
            if (obj_2ds[k]->mvMapPonits.size() >= 10)
                continue;

            // object in last frame.
            for (size_t l = 0; l < mCurrentFrame.mvLastObject_2ds.size(); ++l)
            {
                // ignore objects with more than 10 points.
                if (mCurrentFrame.mvLastObject_2ds[l]->mvMapPonits.size() >= 10)
                    continue;

                // merge two objects.  如果两个object_2d的重叠率大于0.5， 则将两者融合了一个物体
                if (Converter::bboxOverlapratio(obj_2ds[k]->mBox_cvRect, mCurrentFrame.mvLastObject_2ds[l]->mBox_cvRect) > 0.5)
                {
                    obj_2ds[k]->MergeTwo_Obj2D(mCurrentFrame.mvLastObject_2ds[l]);
                    obj_2ds[k]->ComputeMeanAndDeviation();
                    break;
                }
            }
        }
    }


    // ************************************
    // STEP 9. Initialize the object map  *
    // ************************************
    if ( mbObjectIni == false){
        int nGoodObjId = -1;        // object id.
        for (auto &obj2d : obj_2ds)
        {
            // Initialize the object map need enough points.
            if (obj2d->mvMapPonits.size() < 10)
            {  continue;    }

            nGoodObjId++;;

            // Create an object in the map.
            //std::cout<<"【debug】INIT前景物体"<<std::endl;
            Object_Map *Object3D = new Object_Map;
            Object3D->mvObject_2ds.push_back(obj2d);   // 2D objects in each frame associated with this 3D map object.
            Object3D->mnId = nGoodObjId;             // 3d objects in the map.
            Object3D->mnClass = obj2d->mclass_id;      // object class.
            Object3D->mnConfidence_foractive = 1;              // object confidence = mObjectFrame.size().
            Object3D->mnAddedID_nouse = mCurrentFrame.mnId;        // added id.
            Object3D->mnLastAddID = mCurrentFrame.mnId;      // last added id.
            Object3D->mnLastLastAddID = mCurrentFrame.mnId;  // last last added id.
            Object3D->mLastRect = obj2d->mBox_cvRect;             // last rect.
            //Object3D->mPredictRect = obj->mBoxRect;       // for iou.
            Object3D->mSumPointsPos = 0; //cv::Mat::zeros(3,1,CV_32F);
            Object3D->mAveCenter3D = obj2d->mPos_world;  ; //cv::Mat::zeros(3,1,CV_32F);

            //std::cout<<"【debug】INIT前景物体 存入特征点"<<std::endl;
            // add properties of the point and save it to the object.

            for (size_t i = 0; i < obj2d->mvMapPonits.size(); i++)
            {
                if(obj2d->mvMapPonits[i]->isBad())
                    continue;
                MapPoint *pMP = obj2d->mvMapPonits[i];
                pMP->object_mnId = Object3D->mnId;
                pMP->object_class = Object3D->mnClass;
                pMP->viewdCount_forObjectId.insert(make_pair(Object3D->mnId, 1)); // the point is first observed by the object.

                // save to the object.
                Object3D->mvpMapObjectMappoints.push_back(pMP);
                Object3D->mvpMapObjectMappoints_NewForActive.push_back(pMP);
            }

            // 2d object.
            obj2d->mnId = Object3D->mnId;

            // save this 2d object to current frame (associates with a 3d object in the map).
            mCurrentFrame.mvObject_2ds.push_back(obj2d);
            //std::cout<<"【debug】INIT前景物体 存入obj2d"<<std::endl;

            // updata map object.
            Object3D->ComputeMeanAndDeviation_3D();
            //std::cout<<"【debug】INIT前景物体 计算均值"<<std::endl;
            //mpMap->mvObjectMap.push_back(ObjectMapSingle);
            mpMap->AddObject(Object3D);
            //std::cout<<"【debug】INIT前景物体 存入map"<<std::endl;
            //std::cout<<"【debug】INIT前景物体 物体id:"<<Object3D->mnLastAddID<<", 帧id:"<<mCurrentFrame.mnId<<std::endl;
            //物体初始化完成
            mbObjectIni = true;
            mnObjectIniFrameID = mCurrentFrame.mnId;
        }
    }

    // **************************************************************
    // STEP 10. Data association and create objects*
    // **************************************************************
    if ((mCurrentFrame.mnId > mnObjectIniFrameID) && (mbObjectIni == true))
    {
        // step 10.1 points of the object that appeared in the last 30 frames
        // are projected into the image to form a projection bounding box.
        // 将地图中,在过去30帧内出现的物体, obj3d ∈ obj_3ds
        // 中的point投影到Currentframe中，计算obj3d在Currentframe中的投影边界框,并存储在obj3d的 mRectProject_forDataAssociate2D中
        // 从而查看obje3d,能够关联Currentframe中物体
        const std::vector<Object_Map*> obj_3ds = mpMap->GetObjects();
        for (int i = 0; i < (int)obj_3ds.size(); i++)
        {
            Object_Map* obj3d = obj_3ds[i];
            if (obj3d->bad_3d  || obj3d->backgroud_object)
                continue;

            // object appeared in the last 30 frames.
            if(ProIou_only30_flag) {
                if (obj3d->mnLastAddID > mCurrentFrame.mnId - 30)
                    obj3d->ComputeProjectRectFrameToCurrentFrame(mCurrentFrame);  //将obj3d中的point投影到当前帧中，计算投影边界框
                else {
                    obj3d->mRect_byProjectPoints = cv::Rect(0, 0, 0, 0);
                }
            }
            else{
                obj3d->ComputeProjectRectFrameToCurrentFrame(mCurrentFrame);  //将obj3d中的point投影到当前帧中，计算投影边界框
            }
        }

        // step 10.2 data association and creat new object
        // 将当前帧中的obj2d, 与地图中所有obj3d, 进行比较, 看是否数据关联 是同一个物体
        // 如果不是,则生成一个新的物体,并添加进地图Map中.
        for (size_t k = 0; k < obj_2ds.size(); ++k)
        {
            // ignore object with less than 5 points.
            if (obj_2ds[k]->mvMapPonits.size() < 5)
                continue;

            // 融入旧的物体或者生成新的物体
            int result = obj_2ds[k]->creatObject();
            //switch (result) {
            //    case -1:   cout << "检测框靠近边缘" << endl;   break;
            //    case 0:    cout << "融入旧的物体中" << endl;   break;
            //    case 1:    cout << "生成新的物体" << endl;     break;
            //}
        }


    // **************************************************************
    // STEP 11. 一轮物体生成后, 对地图中的物体进行管理*
    // **************************************************************
        // step 11.1 remove objects with too few observations.
        // 在更新完地图中的物体后, 重新获取地图中的物体, 并将观测次数较少的物体剔除(方法为将obj3d的bad_3d设置为true)
        const std::vector<Object_Map*> obj_3ds_new = mpMap->GetObjects();
        for (int i = (int)obj_3ds_new.size() - 1; i >= 0; i--)
        {
            Object_Map* obj3d = obj_3ds_new[i];
            if (obj3d->bad_3d  || obj3d->backgroud_object)
                continue;

            // not been observed in the last 30 frames.
            if (obj3d->mnLastAddID < (mCurrentFrame.mnId - 30))
            {
                int df = (int)obj3d->mvObject_2ds.size();
                if (df < 10)
                {
                    // 如果此物体过去30帧都没有被看到, 而且被观测到的帧数少于5, 则设置为bad_3d
                    if (df < 5){
                        //std::cout<<"object->bad 观测帧<5, 物体id:"<<obj3d->mnLastAddID<<", 帧id:"<<(mCurrentFrame.mnId - 30)<<std::endl;
                        obj3d->bad_3d = true;
                    }
                    // 如果此物体过去30帧都没有被看到, 而且被观测到的帧数少于10, 且与地图中的其他物体过于重合 ,则设置为bad_3d
                    else
                    {
                        for (int j = (int)obj_3ds_new.size() - 1; j >= 0; j--)
                        {
                            if (obj_3ds_new[j]->bad_3d || (i == j)   || obj_3ds_new[j]->backgroud_object)
                                continue;

                            bool overlap = obj_3ds_new[i]->WhetherOverlap(obj_3ds_new[j]) ;
                            if (overlap)
                            {
                                obj_3ds_new[i]->bad_3d = true;
                                //std::cout<<"object->bad 观测帧<10,且overlap" <<std::endl;
                                break;
                            }
                        }
                    }
                }
            }
        }

        // step 11.2 Update the co-view relationship between objects. (appears in the same frame).
        // 对于第一次被创建的物体, 构建共视关系.
        for (int i = (int)obj_3ds_new.size() - 1; i >= 0; i--)
        {
            if (obj_3ds_new[i]->mnLastAddID == mCurrentFrame.mnId)
            {
                for (int j = (int)obj_3ds_new.size() - 1; j >= 0; j--)
                {
                    if (i == j)
                        continue;

                    if (obj_3ds_new[j]->mnLastAddID == mCurrentFrame.mnId)
                    {
                        obj_3ds_new[i]->UpdateCoView(obj_3ds_new[j]);
                    }
                }
            }
        }

        // step 11.3 融合潜在关联的物体 Merge potential associate objects (see mapping thread).

        // step 11.4 估计物体的yaw方向 Estimate the orientation of objects.
        for (int i = (int)obj_3ds_new.size() - 1; i >= 0; i--)
        {
            Object_Map* obj3d = obj_3ds_new[i];

            if (obj3d->bad_3d   || obj3d->backgroud_object)
                continue;
            //如果超过5帧,此物体没有被观测到,略过
            if (obj3d->mnLastAddID < mCurrentFrame.mnId - 5)
                continue;

            // 对常规物体,位姿估计  estimate only regular objects. // if (((objMap->mnClass == 73) || (objMap->mnClass == 64) || (objMap->mnClass == 65//     || (objMap->mnClass == 66) || (objMap->mnClass == 56)))
            // objects appear in current frame.
            if(obj3d->mnLastAddID == mCurrentFrame.mnId)
            {
                SampleObjYaw(obj3d);
            }



            // step 11.5 project quadrics to the image (only for visualization).
            //cv::Mat axe = cv::Mat::zeros(3, 1, CV_32F);
            //axe.at<float>(0) = obj3d->mCuboid3D.lenth / 2;
            //axe.at<float>(1) = obj3d->mCuboid3D.width / 2;
            //axe.at<float>(2) = obj3d->mCuboid3D.height / 2;
            //// object pose (world).
            //cv::Mat Twq = (obj3d->mCuboid3D.pose_mat);
            //// Projection Matrix K[R|t].
            //cv::Mat P(3, 4, CV_32F);
            //Rcw.copyTo(P.rowRange(0, 3).colRange(0, 3));
            //tcw.copyTo(P.rowRange(0, 3).col(3));
            //P = mCurrentFrame.mK * P;
            //DrawQuadricProject( this->mCurrentFrame.mQuadricImage,
            //                            P,
            //                            axe,
            //                            Twq,
            //                            obj3d->mnClass).copyTo(this->mCurrentFrame.mQuadricImage);
        }

    }

}

void Tracking::AssociateObjAndPoints(vector<Object_2D *> obj_2ds)
{

} // AssociateObjAndPoints() END -----------------------------------

bool Tracking::TrackLocalMap()
{
    // We have an estimation of the camera pose and some map points tracked in the frame.
    // We retrieve the local map and try to find matches to points in the local map.

    UpdateLocalMap();

    SearchLocalPoints();

    // add plane
    mpMap->AssociatePlanesByBoundary(mCurrentFrame);
    // add plane end

    // Optimize Pose
    Optimizer::PoseOptimization(&mCurrentFrame);
    mnMatchesInliers = 0;

    // Update MapPoints Statistics
    for(int i=0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(!mCurrentFrame.mvbOutlier[i])
            {
                mCurrentFrame.mvpMapPoints[i]->IncreaseFound();

                {
                    if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                        mnMatchesInliers++;
                }

            }
            else if(mSensor==System::STEREO)
                mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);

        }
    }

    // add plane
    int nDisgardPlane = 0;
    for (int i = 0; i < mCurrentFrame.mnPlaneNum; i++)
    {
        if (mCurrentFrame.mvpMapPlanes[i])
        {
            if (mCurrentFrame.mvpMapPlanes[i] != nullptr && mCurrentFrame.mvbPlaneOutlier[i])
            {
                mCurrentFrame.mvpMapPlanes[i] = static_cast<MapPlane *>(NULL);
                nDisgardPlane++;
            }
            else
                mnMatchesInliers++;
        }
    }
    // add plane end

    // NBV MAM: check the camera model
    //map_data MD;
    //double theta_interval;
    //
    ////unique_lock<mutex> lock(mpMap->mMutexMapUpdate);
    //vector<MapPoint*> vpPts = mpMap->GetAllMapPoints();
    ////cout << "totally " << vpPts.size() << " points." << endl;
    //for(size_t i=0; i<vpPts.size(); i++){
    //    if(vpPts[i]->isBad())
    //        continue;
    //
    //    cv::Mat Tsc_curr = mCurrentFrame.mTcw.clone().inv();   //TODO: 如何使用运动模型 mCurrentFrame.SetPose(mVelocity * mLastFrame.mTcw);
    //
    //
    //    float minDist = vpPts[i]->GetMinDistanceInvariance();
    //    float maxDist = vpPts[i]->GetMaxDistanceInvariance();
    //    float foundRatio = vpPts[i]->GetFoundRatio();
    //
    //    //float Dist = sqrt((Tsc_curr.at<float>(0,3) - vpPts[i]->GetWorldPos().at<float>(0))*(Tsc_curr.at<float>(0,3) - vpPts[i]->GetWorldPos().at<float>(0))+
    //    //(Tsc_curr.at<float>(1,3) - vpPts[i]->GetWorldPos().at<float>(1))*(Tsc_curr.at<float>(1,3) - vpPts[i]->GetWorldPos().at<float>(2))+
    //    //(Tsc_curr.at<float>(2,3) - vpPts[i]->GetWorldPos().at<float>(2))*(Tsc_curr.at<float>(2,3) - vpPts[i]->GetWorldPos().at<float>(2)));
    //
    //    //if((Dist > maxDist)||(Dist < minDist))
    //    //    continue;
    //
    //    MD.Map.push_back(std::vector<double>{vpPts[i]->GetWorldPos().at<float>(0),   vpPts[i]->GetWorldPos().at<float>(1),  vpPts[i]->GetWorldPos().at<float>(2)});
    //
    //    if(vpPts[i]->theta_std * 2.5 < 10.0/57.3){
    //        theta_interval = 10.0/57.3;
    //    }else{
    //        theta_interval = vpPts[i]->theta_std * 2.5;
    //    }
    //
    //
    //
    //    MD.UB.push_back(double(vpPts[i]->theta_mean + theta_interval));
    //    MD.LB.push_back(double(vpPts[i]->theta_mean - theta_interval));
    //    MD.maxDist.push_back(double(maxDist));
    //    MD.minDist.push_back(double(minDist));
    //    MD.foundRatio.push_back(double(foundRatio));
    //}
    //
    //
    //camera camera_model(MD,20);    // zhang 这里的阈值20对于我的实验环境是不是 有点高??
    //cv::FileStorage fSettings(mStrSettingPath, cv::FileStorage::READ);
    //float fx = fSettings["Camera.fx"];
    //float fy = fSettings["Camera.fy"];
    //float cx = fSettings["Camera.cx"];
    //float cy = fSettings["Camera.cy"];
    //float width = fSettings["Camera.width"];
    //float heigh = fSettings["Camera.height"];
    //float max_dis = fSettings["Camera.max_dis"];
    //float min_dis = fSettings["Camera.min_dis"];
    //camera_model.setCamera(fx, fy, cx, cy, width, heigh, max_dis, min_dis );
    //// compute the pose in the body frame
    //if(!mCurrentFrame.mTcw.empty()){
    //
    //    //cv::Mat Tsc = mCurrentFrame.mTcw.clone().inv();
    //    //cv::Mat T_wb_mat = cv::Mat(T_ws_mat*Tsc*T_cb_mat);   //zhang: 世界到机器人身体的变换
    //    ////zhang    T_ws_mat是世界到相机start位姿的变换.  Tsc是相机start位姿到当前相机位姿的变换??   T_cb_mat是相机到机器人身体的变换.
    //    //cv::Mat Twb_cam = T_wb_initial_mat.inv()*T_wb_mat; //zhang:  第五帧位姿, 到当前帧的位姿变换
    //
    //    cv::Mat Twc_cam = mCurrentFrame.mTcw.clone().inv();
    //    //visible_info VI;
    //    int visible_pts = 0;
    //    double great_angle = -5.0;
    //
    //    cv::Mat body = cv::Mat::eye(4,4,CV_32F);
    //    body =Twc_cam * mT_baselink_cam.inv() ;
    //    //std::cout<<"result: "<<  body<<std::endl;
    //    cv::Mat body_new;
    //
    //    for(int i=0; i<=mDivide; i++){
    //        double angle = M_PI/mDivide * i - M_PI/2.0 ;
    //        //旋转
    //        Eigen::AngleAxisd rotation_vector (angle, Eigen::Vector3d(0,0,1));
    //        Eigen::Matrix3d rotation_matrix = rotation_vector.toRotationMatrix();  //分别加45度
    //        //Eigen::Isometry3d trans_matrix;
    //        //trans_matrix.rotate(rotation_vector);
    //        //trans_matrix.pretranslate(Vector3d(0,0,0));
    //        cv::Mat rotate_mat = Converter::toCvMat(rotation_matrix);
    //
    //        //平移
    //        cv::Mat t_mat = (cv::Mat_<float>(3, 1) << 0, 0, 0);
    //
    //        //总变换矩阵
    //        cv::Mat trans_mat = cv::Mat::eye(4, 4, CV_32F);
    //        rotate_mat.copyTo(trans_mat.rowRange(0, 3).colRange(0, 3));
    //        t_mat.copyTo(trans_mat.rowRange(0, 3).col(3));
    //
    //        body_new = body * trans_mat;   //旋转之后的机器人位姿
    //        Twc_cam = body_new * mT_baselink_cam;   //旋转之后的相机位姿
    //        int num = camera_model.countVisible(Twc_cam);
    //        cout<<"--agl:" << angle/M_PI*180<<",num:"<<num;
    //        if(num > visible_pts){
    //            great_angle = angle;
    //            visible_pts = num;
    //        }
    //
    //    }
    //    unique_lock<mutex> lock(mMutexMamAngle);
    //    mGreat_angle = great_angle;//   /M_PI*180 ;
    //    cout << "----number of points predicted=" << visible_pts <<", angle:"<<mGreat_angle/M_PI*180 << endl;
    //    mpMapPublisher->SetMAM(mGreat_angle);
    //}
    // NBV MAM: check the camera model end


    // Decide if the tracking was succesful
    // More restrictive if there was a relocalization recently
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && mnMatchesInliers<50)
        return false;

    if(mnMatchesInliers<30)
        return false;
    else
        return true;
}


int Tracking::NeedNewKeyFrame()
{
    // Step 1：纯VO模式下不插入关键帧

    // Step 2：如果局部地图线程被闭环检测使用，则不插入关键帧
    // If Local Mapping is freezed by a Loop Closure do not insert keyframes
    if(mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
        return false;
    // 获取当前地图中的关键帧数目
    const int nKFs = mpMap->KeyFramesInMap();

    // Step 3：如果距离上一次重定位比较近，并且关键帧数目超出最大限制，不插入关键帧
    // Do not insert keyframes if not enough frames have passed from last relocalisation
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && nKFs>mMaxFrames)
        return false;

    // Step 4：得到参考关键帧跟踪到的地图点数量. UpdateLocalKeyFrames函数中会将与当前关键帧共视程度最高的关键帧设定为当前帧的参考关键帧
    // Tracked MapPoints in the reference keyframe
    int nMinObs = 3;
    if(nKFs<=2)
        nMinObs=2;
    int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);

    // Step 5：查询局部地图线程是否繁忙，当前能否接受新的关键帧
    // Local Mapping accept keyframes?
    bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

    // Step 6：对于双目或RGBD摄像头，统计成功跟踪的近点的数量，如果跟踪到的近点太少，没有跟踪到的近点较多，可以插入关键帧
    // Check how many "close" points are being tracked and how many could be potentially created.
    int nNonTrackedClose = 0;       //双目或RGB-D中没有跟踪到的近点
    int nTrackedClose= 0;           //双目或RGB-D中成功跟踪的近点（三维点）
    if(mSensor!=System::MONOCULAR)
    {
        for(int i =0; i<mCurrentFrame.N; i++)
        {
            if(mCurrentFrame.mvDepth[i]>0 && mCurrentFrame.mvDepth[i]<mThDepth)
            {
                if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                    nTrackedClose++;
                else
                    nNonTrackedClose++;
            }
        }
    }

    // 双目或RGBD情况下：跟踪到的地图点中近点太少 同时 没有跟踪到的三维点太多，可以插入关键帧了
    bool bNeedToInsertClose = (nTrackedClose<100) && (nNonTrackedClose>70);

    // Step 7：决策是否需要插入关键帧
    // // Step 7.1：设定比例阈值Thresholds，当前帧和参考关键帧跟踪到点的比例，比例越大，越倾向于增加关键帧
    float thRefRatio = 0.75f;
    // 关键帧只有一帧，那么插入关键帧的阈值设置的低一点，插入频率较低
    if(nKFs<2)
        thRefRatio = 0.4f;

    //单目情况下插入关键帧的频率很高
    if(mSensor==System::MONOCULAR)
        thRefRatio = 0.9f;

    // Step 7.2：很长时间没有插入关键帧，可以插入
    // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
    const bool c1a = mCurrentFrame.mnId>=mnLastKeyFrameId+mMaxFrames;

    //Step 7.3：满足插入关键帧的最小间隔并且localMapper处于空闲状态，可以插入
    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
    const bool c1b = (mCurrentFrame.mnId>=mnLastKeyFrameId+mMinFrames && bLocalMappingIdle);

    // Step 7.4：在双目，RGB-D的情况下当前帧跟踪到的点比参考关键帧的0.25倍还少，或者满足bNeedToInsertClose
    //Condition 1c: tracking is weak
    const bool c1c =  mSensor!=System::MONOCULAR && (mnMatchesInliers<nRefMatches*0.25 || bNeedToInsertClose) ;

    // Step 7.5：和参考帧相比当前跟踪到的点太少 或者满足bNeedToInsertClose；同时跟踪到的内点还不能太少
    // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
    const bool c2 = ((mnMatchesInliers<nRefMatches*thRefRatio|| bNeedToInsertClose) && mnMatchesInliers>15);

    // [active slam] create new keyframe by object.
    bool c1d = false;
    if (mCurrentFrame.AppearNewObject)
        c1d = true;

    // 首先检查,以上"point的条件"是否满足, 如果满足则生成关键帧.否则检测"平面的条件"
    if((c1a||c1b||c1c)&&c2)
    {
        // If the mapping accepts keyframes, insert keyframe.
        // Otherwise send a signal to interrupt BA
        if(bLocalMappingIdle)
        {
            return 1;
        }
        else
        {
            mpLocalMapper->InterruptBA();
            if(mSensor!=System::MONOCULAR)
            {
                if(mpLocalMapper->KeyframesInQueue()<3)
                    return 1;
                else
                    return false;
            }
            else
                return false;
        }
    }
    //else
    //    return false;

    // 检查,"平面的条件"是否满足, 如果满足则生成关键帧.否则检测"物体的条件"
    if (mCurrentFrame.mbNewPlane)
    {
        // If the mapping accepts keyframes, insert keyframe.
        // Otherwise send a signal to interrupt BA
        if (bLocalMappingIdle)
        {
            return 1;
        }
        else
        {
            mpLocalMapper->InterruptBA();
            if (mSensor != System::MONOCULAR)
            {
                if (mpLocalMapper->KeyframesInQueue() < 3)
                    return 1;
                else
                    return 0;
            }
            else
                return 0;
        }
    }

    // 检测"物体的条件"是否满足, 是否由新的物体出现.
    if (c1d)
    {
        if (bLocalMappingIdle)
        {
            return 2;
        }
        else
        {
            mpLocalMapper->InterruptBA();
            if (mSensor != System::MONOCULAR)
            {
                if (mpLocalMapper->KeyframesInQueue() < 3)
                    return 2;
                else
                    return 0;
            }
            else
                return 0;
        }
    }

    return 0;
}

void Tracking::CreateNewKeyFrame(bool CreateByObjs)
{
    if(!mpLocalMapper->SetNotStop(true))
        return;

    // 将平面和物体信息添加到 keyFrame中
    KeyFrame* pKF = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);

    mpReferenceKF = pKF;
    mCurrentFrame.mpReferenceKF = pKF;

    // 如果当前帧有新的物体出现,则:
    if (CreateByObjs)
        pKF->mbByNewObj = true;

    if(mSensor!=System::MONOCULAR)
    {
        mCurrentFrame.UpdatePoseMatrices();

        if (!CreateByObjs){  //为什么关键帧中,不是由物体创建时,才添加平面
            mpMap->AssociatePlanesByBoundary(mCurrentFrame);

            for (int i = 0; i < mCurrentFrame.mnPlaneNum; ++i)
            {
                // TODO: 增加观测计数是为了什么？
                if (mCurrentFrame.mvpMapPlanes[i])
                // 如果当前帧中, 某平面经过多轮track后 没有被清空, 且此平面之前没有被看到. 则添加到Map中
                {
                    mCurrentFrame.mvpMapPlanes[i]->AddObservation(pKF, i);
                    if (!mCurrentFrame.mvpMapPlanes[i]->mbSeen)
                    {
                        mCurrentFrame.mvpMapPlanes[i]->mbSeen = true;
                        // 只有在关键帧中才更新平面系数
                        mpMap->AddMapPlane(mCurrentFrame.mvpMapPlanes[i]);
                        //Object_Map* ob =  mpMap->computeMinandMax(mCurrentFrame.mvpMapPlanes[i]);
                        //mpMap->AddObject(ob);
                    }
                    continue;
                }

                // 否则,创造新的平面,添加到地图中
                if (mCurrentFrame.mvbPlaneOutlier[i])
                    continue;
                cv::Mat p3D = mCurrentFrame.ComputePlaneWorldCoeff(i);
                MapPlane *pNewMP = new MapPlane(p3D, pKF, i, mpMap);  //输入了，平面的向量、关键帧的指针、平面在关键帧中的序号、全局地图的指针。
                mpMap->AddMapPlane(pNewMP);
                //Object_Map* ob =  mpMap->computeMinandMax(pNewMP);
                //mpMap->AddObject(ob);
                pKF->AddMapPlane(pNewMP, i);

            }
        }

        // We sort points by the measured depth by the stereo/RGBD sensor.
        // We create all those MapPoints whose depth < mThDepth.
        // If there are less than 100 close points we create the 100 closest.
        vector<pair<float,int> > vDepthIdx;
        vDepthIdx.reserve(mCurrentFrame.N);
        for(int i=0; i<mCurrentFrame.N; i++)
        {
            float z = mCurrentFrame.mvDepth[i];
            if(z>0)
            {
                vDepthIdx.push_back(make_pair(z,i));
            }
        }

        if(!vDepthIdx.empty())
        {
            sort(vDepthIdx.begin(),vDepthIdx.end());

            int nPoints = 0;
            for(size_t j=0; j<vDepthIdx.size();j++)
            {
                int i = vDepthIdx[j].second;

                bool bCreateNew = false;

                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(!pMP)
                    bCreateNew = true;
                else if(pMP->Observations()<1)
                {
                    bCreateNew = true;
                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
                }

                if(bCreateNew)
                {
                    cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                    MapPoint* pNewMP = new MapPoint(x3D,pKF,mpMap);
                    pNewMP->AddObservation(pKF,i);
                    pKF->AddMapPoint(pNewMP,i);
                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpMap->AddMapPoint(pNewMP);

                    mCurrentFrame.mvpMapPoints[i]=pNewMP;
                    nPoints++;
                }
                else
                {
                    nPoints++;
                }

                if(vDepthIdx[j].first>mThDepth && nPoints>100)
                    break;
            }
        }
    }

    mpLocalMapper->InsertKeyFrame(pKF);

    mpLocalMapper->SetNotStop(false);

    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKF;
}

void Tracking::SearchLocalPoints()
{
    // Do not search map points already matched
    for(vector<MapPoint*>::iterator vit=mCurrentFrame.mvpMapPoints.begin(), vend=mCurrentFrame.mvpMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP)
        {
            if(pMP->isBad())
            {
                *vit = static_cast<MapPoint*>(NULL);
            }
            else
            {
                pMP->IncreaseVisible();
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                pMP->mbTrackInView = false;
            }
        }
    }

    int nToMatch=0;

    // Project points in frame and check its visibility
    for(vector<MapPoint*>::iterator vit=mvpLocalMapPoints.begin(), vend=mvpLocalMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP->mnLastFrameSeen == mCurrentFrame.mnId)
            continue;
        if(pMP->isBad())
            continue;
        // Project (this fills MapPoint variables for matching)
        if(mCurrentFrame.isInFrustum(pMP,0.5))
        {
            pMP->IncreaseVisible();
            nToMatch++;
        }
    }

    if(nToMatch>0)
    {
        ORBmatcher matcher(0.8);
        int th = 1;
        if(mSensor==System::RGBD)
            th=3;
        // If the camera has been relocalised recently, perform a coarser search
        if(mCurrentFrame.mnId<mnLastRelocFrameId+2)
            th=5;
        matcher.SearchByProjection(mCurrentFrame,mvpLocalMapPoints,th);
    }
}

void Tracking::UpdateLocalMap()
{
    // This is for visualization
    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    // Update
    UpdateLocalKeyFrames();
    UpdateLocalPoints();
}

void Tracking::UpdateLocalPoints()
{
    mvpLocalMapPoints.clear();

    for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        KeyFrame* pKF = *itKF;
        const vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();

        for(vector<MapPoint*>::const_iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end(); itMP!=itEndMP; itMP++)
        {
            MapPoint* pMP = *itMP;
            if(!pMP)
                continue;
            if(pMP->mnTrackReferenceForFrame==mCurrentFrame.mnId)
                continue;
            if(!pMP->isBad())
            {
                mvpLocalMapPoints.push_back(pMP);
                pMP->mnTrackReferenceForFrame=mCurrentFrame.mnId;
            }
        }
    }
}


void Tracking::UpdateLocalKeyFrames()
{
    // Each map point vote for the keyframes in which it has been observed
    map<KeyFrame*,int> keyframeCounter;
    for(int i=0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
            if(!pMP->isBad())
            {
                const map<KeyFrame*,size_t> observations = pMP->GetObservations();
                for(map<KeyFrame*,size_t>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
                    keyframeCounter[it->first]++;
            }
            else
            {
                mCurrentFrame.mvpMapPoints[i]=NULL;
            }
        }
    }

    if(keyframeCounter.empty())
        return;

    int max=0;
    KeyFrame* pKFmax= static_cast<KeyFrame*>(NULL);

    mvpLocalKeyFrames.clear();
    mvpLocalKeyFrames.reserve(3*keyframeCounter.size());

    // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
    for(map<KeyFrame*,int>::const_iterator it=keyframeCounter.begin(), itEnd=keyframeCounter.end(); it!=itEnd; it++)
    {
        KeyFrame* pKF = it->first;

        if(pKF->isBad())
            continue;

        if(it->second>max)
        {
            max=it->second;
            pKFmax=pKF;
        }

        mvpLocalKeyFrames.push_back(it->first);
        pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
    }


    // Include also some not-already-included keyframes that are neighbors to already-included keyframes
    for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        // Limit the number of keyframes
        if(mvpLocalKeyFrames.size()>80)
            break;

        KeyFrame* pKF = *itKF;

        const vector<KeyFrame*> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);

        for(vector<KeyFrame*>::const_iterator itNeighKF=vNeighs.begin(), itEndNeighKF=vNeighs.end(); itNeighKF!=itEndNeighKF; itNeighKF++)
        {
            KeyFrame* pNeighKF = *itNeighKF;
            if(!pNeighKF->isBad())
            {
                if(pNeighKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pNeighKF);
                    pNeighKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        const set<KeyFrame*> spChilds = pKF->GetChilds();
        for(set<KeyFrame*>::const_iterator sit=spChilds.begin(), send=spChilds.end(); sit!=send; sit++)
        {
            KeyFrame* pChildKF = *sit;
            if(!pChildKF->isBad())
            {
                if(pChildKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pChildKF);
                    pChildKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        KeyFrame* pParent = pKF->GetParent();
        if(pParent)
        {
            if(pParent->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
            {
                mvpLocalKeyFrames.push_back(pParent);
                pParent->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                break;
            }
        }

    }

    if(pKFmax)
    {
        mpReferenceKF = pKFmax;
        mCurrentFrame.mpReferenceKF = mpReferenceKF;
    }
}

bool Tracking::Relocalization()
{
    // Compute Bag of Words Vector
    mCurrentFrame.ComputeBoW();

    // Relocalization is performed when tracking is lost
    // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
    vector<KeyFrame*> vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame);

    if(vpCandidateKFs.empty())
        return false;

    const int nKFs = vpCandidateKFs.size();

    // We perform first an ORB matching with each candidate
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.75,true);

    vector<PnPsolver*> vpPnPsolvers;
    vpPnPsolvers.resize(nKFs);

    vector<vector<MapPoint*> > vvpMapPointMatches;
    vvpMapPointMatches.resize(nKFs);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nKFs);

    int nCandidates=0;

    for(int i=0; i<nKFs; i++)
    {
        KeyFrame* pKF = vpCandidateKFs[i];
        if(pKF->isBad())
            vbDiscarded[i] = true;
        else
        {
            int nmatches = matcher.SearchByBoW(pKF,mCurrentFrame,vvpMapPointMatches[i]);
            if(nmatches<15)
            {
                vbDiscarded[i] = true;
                continue;
            }
            else
            {
                PnPsolver* pSolver = new PnPsolver(mCurrentFrame,vvpMapPointMatches[i]);
                pSolver->SetRansacParameters(0.99,10,300,4,0.5,5.991);
                vpPnPsolvers[i] = pSolver;
                nCandidates++;
            }
        }
    }

    // Alternatively perform some iterations of P4P RANSAC
    // Until we found a camera pose supported by enough inliers
    bool bMatch = false;
    ORBmatcher matcher2(0.9,true);

    while(nCandidates>0 && !bMatch)
    {
        for(int i=0; i<nKFs; i++)
        {
            if(vbDiscarded[i])
                continue;

            // Perform 5 Ransac Iterations
            vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;

            PnPsolver* pSolver = vpPnPsolvers[i];
            cv::Mat Tcw = pSolver->iterate(5,bNoMore,vbInliers,nInliers);

            // If Ransac reachs max. iterations discard keyframe
            if(bNoMore)
            {
                vbDiscarded[i]=true;
                nCandidates--;
            }

            // If a Camera Pose is computed, optimize
            if(!Tcw.empty())
            {
                Tcw.copyTo(mCurrentFrame.mTcw);

                set<MapPoint*> sFound;

                const int np = vbInliers.size();

                for(int j=0; j<np; j++)
                {
                    if(vbInliers[j])
                    {
                        mCurrentFrame.mvpMapPoints[j]=vvpMapPointMatches[i][j];
                        sFound.insert(vvpMapPointMatches[i][j]);
                    }
                    else
                        mCurrentFrame.mvpMapPoints[j]=NULL;
                }

                int nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                if(nGood<10)
                    continue;

                for(int io =0; io<mCurrentFrame.N; io++)
                    if(mCurrentFrame.mvbOutlier[io])
                        mCurrentFrame.mvpMapPoints[io]=static_cast<MapPoint*>(NULL);

                // If few inliers, search by projection in a coarse window and optimize again
                if(nGood<50)
                {
                    int nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,10,100);

                    if(nadditional+nGood>=50)
                    {
                        nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                        // If many inliers but still not enough, search by projection again in a narrower window
                        // the camera has been already optimized with many points
                        if(nGood>30 && nGood<50)
                        {
                            sFound.clear();
                            for(int ip =0; ip<mCurrentFrame.N; ip++)
                                if(mCurrentFrame.mvpMapPoints[ip])
                                    sFound.insert(mCurrentFrame.mvpMapPoints[ip]);
                            nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,3,64);

                            // Final optimization
                            if(nGood+nadditional>=50)
                            {
                                nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                                for(int io =0; io<mCurrentFrame.N; io++)
                                    if(mCurrentFrame.mvbOutlier[io])
                                        mCurrentFrame.mvpMapPoints[io]=NULL;
                            }
                        }
                    }
                }


                // If the pose is supported by enough inliers stop ransacs and continue
                if(nGood>=50)
                {
                    bMatch = true;
                    break;
                }
            }
        }
    }

    if(!bMatch)
    {
        return false;
    }
    else
    {
        mnLastRelocFrameId = mCurrentFrame.mnId;
        return true;
    }

}

void Tracking::Reset()
{

    cout << "System Reseting" << endl;
    if(mpViewer)
    {
        mpViewer->RequestStop();
        while(!mpViewer->isStopped())
            usleep(3000);
    }

    // Reset Local Mapping
    cout << "Reseting Local Mapper...";
    mpLocalMapper->RequestReset();
    cout << " done" << endl;

    // Reset Loop Closing
    cout << "Reseting Loop Closing...";
    mpLoopClosing->RequestReset();
    cout << " done" << endl;

    // Clear BoW Database
    cout << "Reseting Database...";
    mpKeyFrameDB->clear();
    cout << " done" << endl;

    // Clear Map (this erase MapPoints and KeyFrames)
    mpMap->clear();

    KeyFrame::nNextId = 0;
    Frame::nNextId = 0;
    mState = NO_IMAGES_YET;

    if(mpInitializer)
    {
        delete mpInitializer;
        mpInitializer = static_cast<Initializer*>(NULL);
    }

    mlRelativeFramePoses.clear();
    mlpReferences.clear();
    mlFrameTimes.clear();
    mlbLost.clear();

    if(mpViewer)
        mpViewer->Release();
}

void Tracking::ChangeCalibration(const string &strSettingPath)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    mbf = fSettings["Camera.bf"];

    Frame::mbInitialComputations = true;
}

void Tracking::InformOnlyTracking(const bool &flag)
{
    mbOnlyTracking = flag;
}


cv::Mat Tracking::DrawQuadricProject(cv::Mat &im,
                                     const cv::Mat &P,   // projection matrix.
                                     const cv::Mat &axe, // axis length.
                                     const cv::Mat &Twq, // object pose.
                                     int nClassid,
                                     bool isGT,
                                     int nLatitudeNum,
                                     int nLongitudeNum)
{
    // color.
    std::vector<cv::Scalar> colors = {  cv::Scalar(135,0,248),
                                        cv::Scalar(255,0,253),
                                        cv::Scalar(4,254,119),
                                        cv::Scalar(255,126,1),
                                        cv::Scalar(0,112,255),
                                        cv::Scalar(0,250,250),
                                        };

    // draw params
    cv::Scalar sc = colors[nClassid % 6];

    int nLineWidth = 2;

    // generate angluar grid -> xyz grid (vertical half sphere)
    vector<float> vfAngularLatitude;  // (-90, 90)
    vector<float> vfAngularLongitude; // [0, 180]
    cv::Mat pointGrid(nLatitudeNum + 2, nLongitudeNum + 1, CV_32FC4);

    for (int i = 0; i < nLatitudeNum + 2; i++)
    {
        float fThetaLatitude = -M_PI_2 + i * M_PI / (nLatitudeNum + 1);
        cv::Vec4f *p = pointGrid.ptr<cv::Vec4f>(i);
        for (int j = 0; j < nLongitudeNum + 1; j++)
        {
            float fThetaLongitude = j * M_PI / nLongitudeNum;
            p[j][0] = axe.at<float>(0, 0) * cos(fThetaLatitude) * cos(fThetaLongitude);
            p[j][1] = axe.at<float>(1, 0) * cos(fThetaLatitude) * sin(fThetaLongitude);
            p[j][2] = axe.at<float>(2, 0) * sin(fThetaLatitude);
            p[j][3] = 1.;
        }
    }

    // draw latitude
    for (int i = 0; i < pointGrid.rows; i++)
    {
        cv::Vec4f *p = pointGrid.ptr<cv::Vec4f>(i);
        // [0, 180]
        for (int j = 0; j < pointGrid.cols - 1; j++)
        {
            cv::Mat spherePt0 = (cv::Mat_<float>(4, 1) << p[j][0], p[j][1], p[j][2], p[j][3]);
            cv::Mat spherePt1 = (cv::Mat_<float>(4, 1) << p[j + 1][0], p[j + 1][1], p[j + 1][2], p[j + 1][3]);
            cv::Mat conicPt0 = P * Twq * spherePt0;
            cv::Mat conicPt1 = P * Twq * spherePt1;
            cv::Point pt0(conicPt0.at<float>(0, 0) / conicPt0.at<float>(2, 0), conicPt0.at<float>(1, 0) / conicPt0.at<float>(2, 0));
            cv::Point pt1(conicPt1.at<float>(0, 0) / conicPt1.at<float>(2, 0), conicPt1.at<float>(1, 0) / conicPt1.at<float>(2, 0));
            cv::line(im, pt0, pt1, sc, nLineWidth); // [0, 180]
        }
        // [180, 360]
        for (int j = 0; j < pointGrid.cols - 1; j++)
        {
            cv::Mat spherePt0 = (cv::Mat_<float>(4, 1) << -p[j][0], -p[j][1], p[j][2], p[j][3]);
            cv::Mat spherePt1 = (cv::Mat_<float>(4, 1) << -p[j + 1][0], -p[j + 1][1], p[j + 1][2], p[j + 1][3]);
            cv::Mat conicPt0 = P * Twq * spherePt0;
            cv::Mat conicPt1 = P * Twq * spherePt1;
            cv::Point pt0(conicPt0.at<float>(0, 0) / conicPt0.at<float>(2, 0), conicPt0.at<float>(1, 0) / conicPt0.at<float>(2, 0));
            cv::Point pt1(conicPt1.at<float>(0, 0) / conicPt1.at<float>(2, 0), conicPt1.at<float>(1, 0) / conicPt1.at<float>(2, 0));
            cv::line(im, pt0, pt1, sc, nLineWidth); // [180, 360]
        }
    }

    // draw longitude
    cv::Mat pointGrid_t = pointGrid.t();
    for (int i = 0; i < pointGrid_t.rows; i++)
    {
        cv::Vec4f *p = pointGrid_t.ptr<cv::Vec4f>(i);
        // [0, 180]
        for (int j = 0; j < pointGrid_t.cols - 1; j++)
        {
            cv::Mat spherePt0 = (cv::Mat_<float>(4, 1) << p[j][0], p[j][1], p[j][2], p[j][3]);
            cv::Mat spherePt1 = (cv::Mat_<float>(4, 1) << p[j + 1][0], p[j + 1][1], p[j + 1][2], p[j + 1][3]);
            cv::Mat conicPt0 = P * Twq * spherePt0;
            cv::Mat conicPt1 = P * Twq * spherePt1;
            cv::Point pt0(conicPt0.at<float>(0, 0) / conicPt0.at<float>(2, 0), conicPt0.at<float>(1, 0) / conicPt0.at<float>(2, 0));
            cv::Point pt1(conicPt1.at<float>(0, 0) / conicPt1.at<float>(2, 0), conicPt1.at<float>(1, 0) / conicPt1.at<float>(2, 0));
            cv::line(im, pt0, pt1, sc, nLineWidth); // [0, 180]
        }
        // [180, 360]
        for (int j = 0; j < pointGrid_t.cols - 1; j++)
        {
            cv::Mat spherePt0 = (cv::Mat_<float>(4, 1) << -p[j][0], -p[j][1], p[j][2], p[j][3]);
            cv::Mat spherePt1 = (cv::Mat_<float>(4, 1) << -p[j + 1][0], -p[j + 1][1], p[j + 1][2], p[j + 1][3]);
            cv::Mat conicPt0 = P * Twq * spherePt0;
            cv::Mat conicPt1 = P * Twq * spherePt1;
            cv::Point pt0(conicPt0.at<float>(0, 0) / conicPt0.at<float>(2, 0), conicPt0.at<float>(1, 0) / conicPt0.at<float>(2, 0));
            cv::Point pt1(conicPt1.at<float>(0, 0) / conicPt1.at<float>(2, 0), conicPt1.at<float>(1, 0) / conicPt1.at<float>(2, 0));
            cv::line(im, pt0, pt1, sc, nLineWidth); // [180, 360]
        }
    }

    return im;
}

// 计算世界中的点，投影到相机帧中的uv坐标。
// project points to image.
cv::Point2f Tracking::WorldToImg(cv::Mat &PointPosWorld)
{
    // world.
    const cv::Mat Rcw = mCurrentFrame.mTcw.rowRange(0, 3).colRange(0, 3);
    const cv::Mat tcw = mCurrentFrame.mTcw.rowRange(0, 3).col(3);

    // camera.
    cv::Mat PointPosCamera = Rcw * PointPosWorld + tcw;

    const float xc = PointPosCamera.at<float>(0);
    const float yc = PointPosCamera.at<float>(1);
    const float invzc = 1.0 / PointPosCamera.at<float>(2);

    // image.
    float u = mCurrentFrame.fx * xc * invzc + mCurrentFrame.cx;
    float v = mCurrentFrame.fy * yc * invzc + mCurrentFrame.cy;

    return cv::Point2f(u, v);
}


void Tracking::SampleObjYaw(Object_Map* obj3d)
{
    int numMax = 0;
    float fError = 0.0;
    float fErrorYaw;
    float minErrorYaw = 360.0;
    float sampleYaw = 0.0;
    int nAllLineNum = obj3d->mvObject_2ds.back()->mObjLinesEigen.rows();  //获取obj3d最新的一帧中获取到的线段的数量

    // 将90°分为30份。前15份为反转，后15份正转。
    for(int i = 0; i < 30; i++)
    {
        // initial angle.
        float roll, pitch, yaw;
        roll = 0.0;
        pitch = 0.0;
        yaw = 0.0;
        float error = 0.0;
        float errorYaw = 0.0;

        // 1 -> 15: -45° - 0°
        // 16 -> 30: 0° - 45°
        if(i < 15)
            yaw = (0.0 - i*3.0)/180.0 * M_PI;
        else
            yaw = (0.0 + (i-15)*3.0)/180.0 * M_PI;

        // 此角度下，物体在自身坐标系下的位姿
        // object pose in object frame. (Ryaw)
        float cp = cos(pitch);
        float sp = sin(pitch);
        float sr = sin(roll);
        float cr = cos(roll);
        float sy = sin(yaw);
        float cy = cos(yaw);
        Eigen::Matrix<double,3,3> REigen;
        REigen<<   cp*cy, (sr*sp*cy)-(cr*sy), (cr*sp*cy)+(sr*sy),
                cp*sy, (sr*sp*sy)+(cr*cy), (cr*sp*sy)-(sr*cy),
                    -sp,    sr*cp,              cr * cp;
        cv::Mat Ryaw = Converter::toCvMat(REigen);

        // 物体坐标系下，没有yaw的corner坐标
        // 8 vertices of the 3D box, world --> object frame.
        cv::Mat corner_1 = Converter::toCvMat(obj3d->mCuboid3D.corner_1_w) - Converter::toCvMat(obj3d->mCuboid3D.cuboidCenter);
        cv::Mat corner_2 = Converter::toCvMat(obj3d->mCuboid3D.corner_2_w) - Converter::toCvMat(obj3d->mCuboid3D.cuboidCenter);
        cv::Mat corner_3 = Converter::toCvMat(obj3d->mCuboid3D.corner_3_w) - Converter::toCvMat(obj3d->mCuboid3D.cuboidCenter);
        cv::Mat corner_4 = Converter::toCvMat(obj3d->mCuboid3D.corner_4_w) - Converter::toCvMat(obj3d->mCuboid3D.cuboidCenter);
        cv::Mat corner_5 = Converter::toCvMat(obj3d->mCuboid3D.corner_5_w) - Converter::toCvMat(obj3d->mCuboid3D.cuboidCenter);
        cv::Mat corner_6 = Converter::toCvMat(obj3d->mCuboid3D.corner_6_w) - Converter::toCvMat(obj3d->mCuboid3D.cuboidCenter);
        cv::Mat corner_7 = Converter::toCvMat(obj3d->mCuboid3D.corner_7_w) - Converter::toCvMat(obj3d->mCuboid3D.cuboidCenter);
        cv::Mat corner_8 = Converter::toCvMat(obj3d->mCuboid3D.corner_8_w) - Converter::toCvMat(obj3d->mCuboid3D.cuboidCenter);

        // 世界坐标系下，有yaw的corner坐标
        // rotate in object frame  + object frame --> world frame.
        corner_1 = Ryaw * corner_1 + Converter::toCvMat(obj3d->mCuboid3D.cuboidCenter);
        corner_2 = Ryaw * corner_2 + Converter::toCvMat(obj3d->mCuboid3D.cuboidCenter);
        corner_3 = Ryaw * corner_3 + Converter::toCvMat(obj3d->mCuboid3D.cuboidCenter);
        corner_4 = Ryaw * corner_4 + Converter::toCvMat(obj3d->mCuboid3D.cuboidCenter);
        corner_5 = Ryaw * corner_5 + Converter::toCvMat(obj3d->mCuboid3D.cuboidCenter);
        corner_6 = Ryaw * corner_6 + Converter::toCvMat(obj3d->mCuboid3D.cuboidCenter);
        corner_7 = Ryaw * corner_7 + Converter::toCvMat(obj3d->mCuboid3D.cuboidCenter);
        corner_8 = Ryaw * corner_8 + Converter::toCvMat(obj3d->mCuboid3D.cuboidCenter);

        // step 1. uv坐标
        // project 8 vertices to image.
        cv::Point2f point1, point2, point3, point4, point5, point6, point7, point8;
        point1 = WorldToImg(corner_1);
        point2 = WorldToImg(corner_2);
        point3 = WorldToImg(corner_3);
        point4 = WorldToImg(corner_4);
        point5 = WorldToImg(corner_5);
        point6 = WorldToImg(corner_6);
        point7 = WorldToImg(corner_7);
        point8 = WorldToImg(corner_8);

        // step 2. 计算cube 3条边的角度
        // angle of 3 edges(lenth, width, height).
        float angle1;
        float angle2;
        float angle3;
        // left -> right.
        if(point6.x > point5.x)
            angle1 = atan2(point6.y - point5.y, point6.x - point5.x);
        else
            angle1 = atan2(point5.y - point6.y, point5.x - point6.x);
        float lenth1 = sqrt((point6.y - point5.y) * (point6.y - point5.y) + (point6.x - point5.x) * (point6.x - point5.x));

        if(point7.x > point6.x)
            angle2 = atan2(point7.y - point6.y, point7.x - point6.x);
        else
            angle2 = atan2(point6.y - point7.y, point6.x - point7.x);
        float lenth2 = sqrt((point7.y - point6.y) * (point7.y - point6.y) + (point7.x - point6.x) * (point7.x - point6.x));

        if(point6.x > point2.x)
            angle3 = atan2(point6.y - point2.y, point6.x - point2.x);
        else
            angle3 = atan2(point2.y - point6.y, point2.x - point6.x);
        float lenth3 = sqrt((point6.y - point2.y) * (point6.y - point2.y) + (point6.x - point2.x) * (point6.x - point2.x));

        // step 3. 计算最新一帧中 线段 和 cube边 的角度偏差
        // 为什么只用最新一帧?? 万一不满足之前帧的中线段 怎么办?
        // compute angle between detected lines and cube edges.
        int num = 0;  // number parallel lines.
        for(int line_id = 0; line_id < obj3d->mvObject_2ds.back()->mObjLinesEigen.rows(); line_id++)
        {
            // angle of detected lines.
            double x1 = obj3d->mvObject_2ds.back()->mObjLinesEigen(line_id, 0);
            double y1 = obj3d->mvObject_2ds.back()->mObjLinesEigen(line_id, 1);
            double x2 = obj3d->mvObject_2ds.back()->mObjLinesEigen(line_id, 2);
            double y2 = obj3d->mvObject_2ds.back()->mObjLinesEigen(line_id, 3);
            float angle = atan2(y2 - y1, x2 - x1);

            // lenth.
            float lenth = sqrt((y2 - y1)*(y2 - y1) + (x2 - x1)*(x2 - x1));

            // angle between line and 3 edges.
            float dis_angle1 = abs(angle * 180/M_PI - angle1 * 180/M_PI);
            float dis_angle2 = abs(angle * 180/M_PI - angle2 * 180/M_PI);
            float dis_angle3 = abs(angle * 180/M_PI - angle3 * 180/M_PI);

            float th = 5.0;             // threshold of the angle.
            if(obj3d->mnClass == 56)   // chair.
            {
                if((dis_angle2 < th) || (dis_angle3 < th))
                    num++;
                if(dis_angle1 < th)
                {
                    num+=3;
                }
            }
            else
            {
                // the shortest edge is lenth1.
                if( min(min(lenth1, lenth2), lenth3) == lenth1)
                {
                    // error with other two edges.
                    if((dis_angle2 < th) || (dis_angle3 < th))
                    {
                        num++;
                        if(dis_angle2 < th)
                            error += dis_angle2;
                        if(dis_angle3 < th)
                            error += dis_angle3;
                    }

                    // angle error.
                    errorYaw+=min(dis_angle2, dis_angle3);
                }
                // the shortest edge is lenth2.
                if( min(min(lenth1, lenth2), lenth3) == lenth2)
                {
                    if((dis_angle1 < th) || (dis_angle3 < th))
                    {
                        num++;
                        if(dis_angle1 < th)
                            error += dis_angle1;
                        if(dis_angle3 < th)
                            error += dis_angle3;
                    }
                    errorYaw+=min(dis_angle3, dis_angle1);
                }
                // the shortest edge is lenth3.
                if( min(min(lenth1, lenth2), lenth3) == lenth3)
                {
                    if((dis_angle1 < th) || (dis_angle2 < th))
                    {
                        num++;
                        if(dis_angle1 < th)
                            error += dis_angle1;
                        if(dis_angle2 < th)
                            error += dis_angle2;
                    }
                    errorYaw+=min(dis_angle2, dis_angle1);
                }
            }
        }
        if(num == 0)
        {
            num = 1;
            errorYaw = 10.0;
        }

        // record the angle with max number parallel lines.
        if(num > numMax)
        {
            numMax = num;
            sampleYaw = yaw;

            fError = error; // no used in this version.
            // average angle error.
            fErrorYaw = (errorYaw/(float)num)/10.0;
        }
    }

    // step 4. 得分等于，(平行线的数量/图片中线段数量)* (10 - fErrorYaw) /10
    // scoring.
    float fScore;
    fScore = ((float)numMax / (float)nAllLineNum) * (1.0 - 0.1 * fErrorYaw);
    if(isinf(fScore))
        fScore = 0.0;

    // 存储下 各个角度的得分和次数
    // measurement： yaw, times, score, angle, angle error.
    Eigen::Matrix<float,5,1> AngleTimesAndScore;
    AngleTimesAndScore[0] = sampleYaw;
    AngleTimesAndScore[1] = 1.0;
    AngleTimesAndScore[2] = fScore;
    AngleTimesAndScore[3] = fError;     // no used in this version.
    AngleTimesAndScore[4] = fErrorYaw;

    // update multi-frame measurement.
    bool bNewMeasure = true;
    for (auto &row : obj3d->mvAngleTimesAndScore)
    {
        if(row[0] == AngleTimesAndScore[0])
        {
            row[1] += 1.0;
            row[2] = AngleTimesAndScore[2] * (1/row[1]) + row[2] * (1 - 1/row[1]);
            row[3] = AngleTimesAndScore[3] * (1/row[1]) + row[3] * (1 - 1/row[1]);
            row[4] = AngleTimesAndScore[4] * (1/row[1]) + row[4] * (1 - 1/row[1]);

            bNewMeasure = false;
        }
    }
    if(bNewMeasure == true)
    {
        obj3d->mvAngleTimesAndScore.push_back(AngleTimesAndScore);
    }


    // step 5. rank: 将mvAngleTimesAndScore按照次数从大到小进行排序
    //index = 1;
    std::sort(obj3d->mvAngleTimesAndScore.begin(), obj3d->mvAngleTimesAndScore.end(), VIC);
     //for (auto &row : obj3d->mvAngleTimesAndScore)
     //{
     //    std::cout << row[0] * 180.0 / M_PI  << "\t" <<  row[1] << "\t" <<  row[2] << std::endl;
     //}

    // score最高的是the best yaw.
    // 那为什么上面还要按次数排序? 答:使得score相同是, 次数最高的优先被选取
    int best_num = 0;
    float best_score = 0;
    for(int i = 0; i < std::min(3, (int)obj3d->mvAngleTimesAndScore.size()); i++)
    {
        float fScore = obj3d->mvAngleTimesAndScore[i][2];
        if(fScore >= best_score)
        {
            best_score = fScore;
            best_num = i;
        }
    }

    // step 6. update object yaw.
    obj3d->mCuboid3D.rotY = obj3d->mvAngleTimesAndScore[best_num][0];
    obj3d->mCuboid3D.mfErrorParallel = obj3d->mvAngleTimesAndScore[best_num][3];
    obj3d->mCuboid3D.mfErroeYaw = obj3d->mvAngleTimesAndScore[best_num][4];
}

void Tracking::SetNbvGenerator(NbvGenerator *pNbvGenerator)
{
    mpNbvGenerator = pNbvGenerator;
}

//double Tracking::getMamGreadAngle(){
//    unique_lock<mutex> lock(mMutexMamAngle);
//    return  mGreat_angle;
//}

} //namespace ORB_SLAM
