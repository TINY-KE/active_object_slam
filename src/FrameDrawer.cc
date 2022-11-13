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

#include "FrameDrawer.h"
#include "Tracking.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include<mutex>

namespace ORB_SLAM2
{

FrameDrawer::FrameDrawer(Map* pMap):mpMap(pMap)
{
    mState=Tracking::SYSTEM_NOT_READY;
    mIm = cv::Mat(480,640,CV_8UC3, cv::Scalar(0,0,0));
}

cv::Mat FrameDrawer::DrawFrame()
{
    cv::Mat im;
    vector<cv::KeyPoint> vIniKeys; // Initialization: KeyPoints in reference frame
    vector<int> vMatches; // Initialization: correspondeces with reference keypoints
    vector<cv::KeyPoint> vCurrentKeys; // KeyPoints in current frame
    vector<bool> vbVO, vbMap; // Tracked MapPoints in current frame
    int state; // Tracking state

    //Copy variables within scoped mutex
    {
        unique_lock<mutex> lock(mMutex);
        state=mState;
        if(mState==Tracking::SYSTEM_NOT_READY)
            mState=Tracking::NO_IMAGES_YET;

        mIm.copyTo(im);

        if(mState==Tracking::NOT_INITIALIZED)
        {
            vCurrentKeys = mvCurrentKeys;
            vIniKeys = mvIniKeys;
            vMatches = mvIniMatches;
        }
        else if(mState==Tracking::OK)
        {
            vCurrentKeys = mvCurrentKeys;
            vbVO = mvbVO;
            vbMap = mvbMap;
        }
        else if(mState==Tracking::LOST)
        {
            vCurrentKeys = mvCurrentKeys;
        }
    } // destroy scoped mutex -> release mutex

    if(im.channels()<3) //this should be always true
        cvtColor(im,im,CV_GRAY2BGR);

    //Draw
    if(state==Tracking::NOT_INITIALIZED) //INITIALIZING
    {
        // 关键特征点
        for(unsigned int i=0; i<vCurrentKeys.size(); i++)
        {
            cv::circle(im,vCurrentKeys[i].pt,1,cv::Scalar(0,0,255),-1);
        }

        for(unsigned int i=0; i<vMatches.size(); i++)
        {
            if(vMatches[i]>=0)
            {
                cv::line(im,vIniKeys[i].pt,vCurrentKeys[vMatches[i]].pt,
                        cv::Scalar(0,255,0));
            }
        }

    }
    else if(state==Tracking::OK) //TRACKING
    {
        mnTracked=0;
        mnTrackedVO=0;
        const float r = 5;
        const int n = vCurrentKeys.size();
        for(int i=0;i<n;i++)
        {
            if(vbVO[i] || vbMap[i])
            {
                cv::Point2f pt1,pt2;
                pt1.x=vCurrentKeys[i].pt.x-r;
                pt1.y=vCurrentKeys[i].pt.y-r;
                pt2.x=vCurrentKeys[i].pt.x+r;
                pt2.y=vCurrentKeys[i].pt.y+r;

                // This is a match to a MapPoint in the map
                if(vbMap[i])
                {
                    bool bInBox = false;
                    //if(mbshow_yolo_result)
                    {
                        // NOTE [EAO-SLAM] points in the bounding box.
                        for (auto&box : Dboxes)
                        {
                            int left = box.x;
                            int right = box.x+box.width;
                            int top = box.y;
                            int bottom = box.y+box.height;

                            if((vCurrentKeys[i].pt.x > left)&&(vCurrentKeys[i].pt.x < right)
                                &&(vCurrentKeys[i].pt.y > top)&&(vCurrentKeys[i].pt.y < bottom))
                            {
                                cv::circle(im, vCurrentKeys[i].pt, 2, colors[box.m_class%6], -1);
                                bInBox = true;
                                break;
                            }
                        }
                    }
                    if(bInBox == false)
                    {
                        cv::rectangle(im,pt1,pt2,cv::Scalar(0,255,0));
                        cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(255,0,255),-1);
                    }
                    mnTracked++;
                }
                else // This is match to a "visual odometry" MapPoint created in the last frame
                {
                    cv::rectangle(im,pt1,pt2,cv::Scalar(255,0,0));
                    cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(255,0,0),-1);
                    mnTrackedVO++;
                }
            }
        }
    }

    //cv::Mat imWithInfo;
    //DrawTextInfo(im,state, imWithInfo);
    //return imWithInfo;

    // yolo detection.
    DrawYoloInfo(im, true);
    return im;
}


void FrameDrawer::DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText)
{
    stringstream s;
    if(nState==Tracking::NO_IMAGES_YET)
        s << " WAITING FOR IMAGES";
    else if(nState==Tracking::NOT_INITIALIZED)
        s << " TRYING TO INITIALIZE ";
    else if(nState==Tracking::OK)
    {
        if(!mbOnlyTracking)
            s << "SLAM MODE |  ";
        else
            s << "LOCALIZATION | ";
        int nKFs = mpMap->KeyFramesInMap();
        int nMPs = mpMap->MapPointsInMap();
        s << "KFs: " << nKFs << ", MPs: " << nMPs << ", Matches: " << mnTracked;
        if(mnTrackedVO>0)
            s << ", + VO matches: " << mnTrackedVO;
    }
    else if(nState==Tracking::LOST)
    {
        s << " TRACK LOST. TRYING TO RELOCALIZE ";
    }
    else if(nState==Tracking::SYSTEM_NOT_READY)
    {
        s << " LOADING ORB VOCABULARY. PLEASE WAIT...";
    }

    int baseline=0;
    cv::Size textSize = cv::getTextSize(s.str(),cv::FONT_HERSHEY_PLAIN,1,1,&baseline);

    imText = cv::Mat(im.rows+textSize.height+10,im.cols,im.type());
    im.copyTo(imText.rowRange(0,im.rows).colRange(0,im.cols));
    imText.rowRange(im.rows,imText.rows) = cv::Mat::zeros(textSize.height+10,im.cols,im.type());
    cv::putText(imText,s.str(),cv::Point(5,imText.rows-5),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(255,255,255),1,8);

}

void FrameDrawer::Update(Tracking *pTracker)
{
    unique_lock<mutex> lock(mMutex);
    pTracker->mImGray.copyTo(mIm);
    mvCurrentKeys=pTracker->mCurrentFrame.mvKeys;
    N = mvCurrentKeys.size();
    mvbVO = vector<bool>(N,false);
    mvbMap = vector<bool>(N,false);
    mbOnlyTracking = pTracker->mbOnlyTracking;

    //[active slam]
    //pTracker->mCurrentFrame.mColorImage.copyTo(mRGBIm);
    pTracker->mCurrentFrame.mQuadricImage.copyTo(mQuadricIm);
    Dboxes = pTracker->mCurrentFrame.boxes;
    this->mTcw = pTracker->mCurrentFrame.mTcw;
    this->mK = pTracker->mCurrentFrame.mK;

    if(pTracker->mLastProcessedState==Tracking::NOT_INITIALIZED)
    {
        mvIniKeys=pTracker->mInitialFrame.mvKeys;
        mvIniMatches=pTracker->mvIniMatches;
    }
    else if(pTracker->mLastProcessedState==Tracking::OK)
    {
        for(int i=0;i<N;i++)
        {
            MapPoint* pMP = pTracker->mCurrentFrame.mvpMapPoints[i];
            if(pMP)
            {
                if(!pTracker->mCurrentFrame.mvbOutlier[i])
                {
                    if(pMP->Observations()>0)
                        mvbMap[i]=true;
                    else
                        mvbVO[i]=true;
                }
            }
        }
    }
    mState=static_cast<int>(pTracker->mLastProcessedState);    /* 关键： 更新mstate */
}


// [active slam]
cv::Mat FrameDrawer::DrawQuadricImage()
{

    cv::Mat imRGB = mQuadricIm.clone();
    // Projection Matrix K[R|t].  相机帧到世界的变化关系
    cv::Mat Pcw(3, 4, CV_32F);
    if(mTcw.size().height!=4){
        std::cout<<"相机位姿为空" <<std::endl;
        return imRGB;
    }
    const cv::Mat Rcw = mTcw.rowRange(0, 3).colRange(0, 3);
    const cv::Mat tcw = mTcw.rowRange(0, 3).col(3);
    Rcw.copyTo(Pcw.rowRange(0, 3).colRange(0, 3));
    tcw.copyTo(Pcw.rowRange(0, 3).col(3));
    cv::Mat Pfw(3, 4, CV_32F);
    Pfw = mK * Pcw;

    const std::vector<Object_Map*> obj_3ds_new = mpMap->GetObjects();
    for(int i = (int)obj_3ds_new.size() - 1; i >= 0; i--){

        //cv::Mat DrawQuadricProject( cv::Mat &im,
        //                        const cv::Mat &P,
        //                        const cv::Mat &axe,
        //                        const cv::Mat &Twq,
        //                        int nClassid,
        //                        bool isGT=true,
        //                        int nLatitudeNum = 7,
        //                        int nLongitudeNum = 6);

        //ColorImage = DrawQuadricProject(this->mCurrentFrame.mQuadricImage,
        //                                    P,
        //                                    axe,
        //                                    Twq,
        //                                    obj_3ds_new[i]->mnClass);
        Object_Map* obj = obj_3ds_new[i];
        if(obj->bad_3d)
            continue;

        // 尺寸
        cv::Mat axe = cv::Mat::zeros(3, 1, CV_32F);
        axe.at<float>(0) = obj->mCuboid3D.lenth / 2;
        axe.at<float>(1) = obj->mCuboid3D.width / 2;
        axe.at<float>(2) = obj->mCuboid3D.height / 2;

        // object pose (world).
        cv::Mat Twq = obj->mCuboid3D.pose_mat;//Converter::toCvMat(obj_3ds_new[i]->mCuboid3D.pose);

        // draw params
        cv::Scalar sc = colors[ obj->mnClass % 6];
        int nLatitudeNum = 7, nLongitudeNum = 6;
        bool isGT=true;
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
                cv::Mat conicPt0 = Pfw * Twq * spherePt0;
                cv::Mat conicPt1 = Pfw * Twq * spherePt1;
                cv::Point pt0(conicPt0.at<float>(0, 0) / conicPt0.at<float>(2, 0), conicPt0.at<float>(1, 0) / conicPt0.at<float>(2, 0));
                cv::Point pt1(conicPt1.at<float>(0, 0) / conicPt1.at<float>(2, 0), conicPt1.at<float>(1, 0) / conicPt1.at<float>(2, 0));
                cv::line(imRGB, pt0, pt1, sc, nLineWidth); // [0, 180]
            }
            // [180, 360]
            for (int j = 0; j < pointGrid.cols - 1; j++)
            {
                cv::Mat spherePt0 = (cv::Mat_<float>(4, 1) << -p[j][0], -p[j][1], p[j][2], p[j][3]);
                cv::Mat spherePt1 = (cv::Mat_<float>(4, 1) << -p[j + 1][0], -p[j + 1][1], p[j + 1][2], p[j + 1][3]);
                cv::Mat conicPt0 = Pfw * Twq * spherePt0;
                cv::Mat conicPt1 = Pfw * Twq * spherePt1;
                cv::Point pt0(conicPt0.at<float>(0, 0) / conicPt0.at<float>(2, 0), conicPt0.at<float>(1, 0) / conicPt0.at<float>(2, 0));
                cv::Point pt1(conicPt1.at<float>(0, 0) / conicPt1.at<float>(2, 0), conicPt1.at<float>(1, 0) / conicPt1.at<float>(2, 0));
                cv::line(imRGB, pt0, pt1, sc, nLineWidth); // [180, 360]
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
                cv::Mat conicPt0 = Pfw * Twq * spherePt0;
                cv::Mat conicPt1 = Pfw * Twq * spherePt1;
                cv::Point pt0(conicPt0.at<float>(0, 0) / conicPt0.at<float>(2, 0), conicPt0.at<float>(1, 0) / conicPt0.at<float>(2, 0));
                cv::Point pt1(conicPt1.at<float>(0, 0) / conicPt1.at<float>(2, 0), conicPt1.at<float>(1, 0) / conicPt1.at<float>(2, 0));
                cv::line(imRGB, pt0, pt1, sc, nLineWidth); // [0, 180]
            }
            // [180, 360]
            for (int j = 0; j < pointGrid_t.cols - 1; j++)
            {
                cv::Mat spherePt0 = (cv::Mat_<float>(4, 1) << -p[j][0], -p[j][1], p[j][2], p[j][3]);
                cv::Mat spherePt1 = (cv::Mat_<float>(4, 1) << -p[j + 1][0], -p[j + 1][1], p[j + 1][2], p[j + 1][3]);
                cv::Mat conicPt0 = Pfw * Twq * spherePt0;
                cv::Mat conicPt1 = Pfw * Twq * spherePt1;
                cv::Point pt0(conicPt0.at<float>(0, 0) / conicPt0.at<float>(2, 0), conicPt0.at<float>(1, 0) / conicPt0.at<float>(2, 0));
                cv::Point pt1(conicPt1.at<float>(0, 0) / conicPt1.at<float>(2, 0), conicPt1.at<float>(1, 0) / conicPt1.at<float>(2, 0));
                cv::line(imRGB, pt0, pt1, sc, nLineWidth); // [180, 360]
            }
        }
    }

    return imRGB;
}

cv::Mat FrameDrawer::DrawYoloInfo(cv::Mat &im, bool bText)
{
    for (auto&box : Dboxes)
    {
        if(bText)
        {
            cv::putText(im,
                        class_names[box.m_class],
                        box.tl(),
                        cv::FONT_HERSHEY_DUPLEX  ,
                        1.0,
                        colors[box.m_class%4],
                        // cv::Scalar(0,255,0),
                        2);
        }



        // draw bounding box.
        cv::rectangle(  im,
                        box,
                        colors[box.m_class%6],
                        2);

        // cv::putText( im, text, cv::Point(x, y + label_size.height),
        //             cv::FONT_HERSHEY_SIMPLEX, 0.4, colors[box.m_class % 4], 1);
    }

    return im;
}

} //namespace ORB_SLAM
