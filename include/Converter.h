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

#ifndef CONVERTER_H
#define CONVERTER_H

#include<opencv2/core/core.hpp>

#include<Eigen/Dense>
#include"Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include"Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

//eigen cv的convert
#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>
#include<opencv2/core/eigen.hpp>

//#include "sophus/so3.h"
//#include "sophus/se3.h"


namespace ORB_SLAM2
{

class Converter
{
public:
    static std::vector<cv::Mat> toDescriptorVector(const cv::Mat &Descriptors);

    static g2o::SE3Quat toSE3Quat(const cv::Mat &cvT);
    static g2o::SE3Quat toSE3Quat(const g2o::Sim3 &gSim3);

    static cv::Mat toCvMat(const g2o::SE3Quat &SE3);
    static cv::Mat toCvMat(const g2o::Sim3 &Sim3);
    static cv::Mat toCvMat(const Eigen::Matrix<double,4,4> &m);
    static cv::Mat toCvMat(const Eigen::Matrix3d &m);

    static cv::Mat toCvMat(const Eigen::Matrix<double,3,1> &m);
    static cv::Mat toCvSE3(const Eigen::Matrix<double,3,3> &R, const Eigen::Matrix<double,3,1> &t);

    static Eigen::Matrix<double,3,1> toVector3d(const cv::Mat &cvVector);
    static Eigen::Matrix<double,3,1> toVector3d(const cv::Point3f &cvPoint);
    static Eigen::Matrix<double,3,3> toMatrix3d(const cv::Mat &cvMat3);

    static std::vector<float> toQuaternion(const cv::Mat &M);

//[active slam]
    static float bboxOverlapratio(const cv::Rect& rect1, const cv::Rect& rect2);
    static float bboxOverlapratioLatter(const cv::Rect& rect1, const cv::Rect& rect2);
    static float bboxOverlapratioFormer(const cv::Rect& rect1, const cv::Rect& rect2);



//位姿的形式变换

    static cv::Mat Quation2CvMat(const double qx, const double qy, const double qz, const double qw, const double tx, const double ty, const double tz  );
    static Eigen::Matrix4d Quation2Eigen(const double qx, const double qy, const double qz, const double qw, const double tx, const double ty, const double tz  );

    static Eigen::Quaterniond ExtractQuaterniond(const Eigen::Isometry3d &Iso );
    static Eigen::Quaterniond ExtractQuaterniond(const Eigen::Matrix4d &matrix );
    static Eigen::Quaterniond ExtractQuaterniond(const cv::Mat &mat );

    static Eigen::Isometry3d Matrix4dtoIsometry3d(const Eigen::Matrix4d &matrix );
    static Eigen::Matrix4d Isometry3dtoMatrix4d(const Eigen::Isometry3d &Iso );
    static Eigen::Matrix4d cvMattoMatrix4d(const cv::Mat &cvMat4);
    static Eigen::Isometry3d cvMattoIsometry3d(const cv::Mat &cvMat4);
    static g2o::SE3Quat cvMattoG2oSE3Quat(const cv::Mat &cvMat4);

};

}// namespace ORB_SLAM

#endif // CONVERTER_H
