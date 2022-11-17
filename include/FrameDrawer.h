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

#ifndef FRAMEDRAWER_H
#define FRAMEDRAWER_H

#include "Tracking.h"
#include "MapPoint.h"
#include "Map.h"

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include<mutex>


namespace ORB_SLAM2
{

class Tracking;
class Viewer;

class FrameDrawer
{
public:
    FrameDrawer(Map* pMap);

    // Update info from the last processed frame.
    void Update(Tracking *pTracker);

    // Draw last processed frame.
    cv::Mat DrawFrame();

protected:

    void DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText);

    // Info of the frame to be drawn
    cv::Mat mIm;
    int N;
    vector<cv::KeyPoint> mvCurrentKeys;
    vector<bool> mvbMap, mvbVO;
    bool mbOnlyTracking;
    int mnTracked, mnTrackedVO;
    vector<cv::KeyPoint> mvIniKeys;
    vector<int> mvIniMatches;
    int mState;

    Map* mpMap;

    std::mutex mMutex;

//[active slam]
private:
    std::vector<cv::Scalar> colors = {  cv::Scalar(135,0,248),
                                        cv::Scalar(255,0,253),
                                        cv::Scalar(4,254,119),
                                        cv::Scalar(255,126,1),
                                        cv::Scalar(0,112,255),
                                        cv::Scalar(0,250,250),    };
    std::vector<std::string> class_names = {
                                        "person",
                                        "bicycle",
                                        "car",
                                        "motorbike",
                                        "aeroplane",
                                        "bus",
                                        "train",
                                        "truck",
                                        "boat",
                                        "traffic light",
                                        "fire hydrant",
                                        "stop sign",
                                        "parking meter",
                                        "bench",
                                        "bird",
                                        "cat",
                                        "dog",
                                        "horse",
                                        "sheep",
                                        "cow",
                                        "elephant",
                                        "bear",
                                        "zebra",
                                        "giraffe",
                                        "backpack",
                                        "umbrella",
                                        "handbag",
                                        "tie",
                                        "suitcase",
                                        "frisbee",
                                        "skis",
                                        "snowboard",
                                        "sports ball",
                                        "kite",
                                        "baseball bat",
                                        "baseball glove",
                                        "skateboard",
                                        "surfboard",
                                        "tennis racket",
                                        "bottle",
                                        "wine glass",
                                        "cup",
                                        "fork",
                                        "knife",
                                        "spoon",
                                        "bowl",
                                        "banana",
                                        "apple",
                                        "sandwich",
                                        "orange",
                                        "broccoli",
                                        "carrot",
                                        "hot dog",
                                        "pizza",
                                        "donut",
                                        "cake",
                                        "chair",
                                        "sofa",
                                        "pottedplant",
                                        "bed",
                                        "diningtable",
                                        "toilet",
                                        "tvmonitor",
                                        "laptop",
                                        "mouse",
                                        "remote",
                                        "keyboard",
                                        "cell phone",
                                        "microwave",
                                        "oven",
                                        "toaster",
                                        "sink",
                                        "refrigerator",
                                        "book",
                                        "clock",
                                        "vase",
                                        "scissors",
                                        "teddy bear",
                                        "hair drier",
                                        "toothbrush"
                                        };
public:

    // color image
    cv::Mat mQuadricIm;
    bool mbshow_yolo_result;
    cv::Mat GetQuadricImage();
    cv::Mat DrawQuadricImage();

    // bounding box.
    std::vector<BoxSE> Dboxes;
    cv::Mat DrawYoloInfo(cv::Mat &im, bool bText);
    cv::Mat mTcw; // 相机帧到世界的变化关系  用于绘制物体在相机帧中的投影
    cv::Mat mK;   // Calibration matrix  用于绘制物体在相机帧中的投影
    int CurFrameId = -1;

    // lines.
    std::vector< KeyLine> Dkeylines_raw_nouse, Dkeylines_out_nouse;
    double DTimeStamp_nouse;
    std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd> >  DObjsLines;    // object lines.  std::vector<Eigen::MatrixXd>
};

} //namespace ORB_SLAM

#endif // FRAMEDRAWER_H
