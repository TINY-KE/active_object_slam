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


#include "Converter.h"

namespace ORB_SLAM2
{

std::vector<cv::Mat> Converter::toDescriptorVector(const cv::Mat &Descriptors)
{
    std::vector<cv::Mat> vDesc;
    vDesc.reserve(Descriptors.rows);
    for (int j=0;j<Descriptors.rows;j++)
        vDesc.push_back(Descriptors.row(j));

    return vDesc;
}

g2o::SE3Quat Converter::toSE3Quat(const cv::Mat &cvT)
{
    Eigen::Matrix<double,3,3> R;
    R << cvT.at<float>(0,0), cvT.at<float>(0,1), cvT.at<float>(0,2),
         cvT.at<float>(1,0), cvT.at<float>(1,1), cvT.at<float>(1,2),
         cvT.at<float>(2,0), cvT.at<float>(2,1), cvT.at<float>(2,2);

    Eigen::Matrix<double,3,1> t(cvT.at<float>(0,3), cvT.at<float>(1,3), cvT.at<float>(2,3));

    return g2o::SE3Quat(R,t);
}

cv::Mat Converter::toCvMat(const g2o::SE3Quat &SE3)
{
    Eigen::Matrix<double,4,4> eigMat = SE3.to_homogeneous_matrix();
    return toCvMat(eigMat);
}

cv::Mat Converter::toCvMat(const g2o::Sim3 &Sim3)
{
    Eigen::Matrix3d eigR = Sim3.rotation().toRotationMatrix();
    Eigen::Vector3d eigt = Sim3.translation();
    double s = Sim3.scale();
    return toCvSE3(s*eigR,eigt);
}

cv::Mat Converter::toCvMat(const Eigen::Matrix<double,4,4> &m)
{
    cv::Mat cvMat(4,4,CV_32F);
    for(int i=0;i<4;i++)
        for(int j=0; j<4; j++)
            cvMat.at<float>(i,j)=m(i,j);

    return cvMat.clone();
    //cv::Mat cv_mat_32f;
    //cv::eigen2cv(GroundtruthPose_eigen, cv_mat_32f);
    //cv_mat_32f.convertTo(mT_body_cam, CV_32F);
}

cv::Mat Converter::toCvMat(const Eigen::Matrix3d &m)
{
    cv::Mat cvMat(3,3,CV_32F);
    for(int i=0;i<3;i++)
        for(int j=0; j<3; j++)
            cvMat.at<float>(i,j)=m(i,j);

    return cvMat.clone();
}

cv::Mat Converter::toCvMat(const Eigen::Matrix<double,3,1> &m)
{
    cv::Mat cvMat(3,1,CV_32F);
    for(int i=0;i<3;i++)
            cvMat.at<float>(i)=m(i);

    return cvMat.clone();
}

cv::Mat Converter::toCvSE3(const Eigen::Matrix<double,3,3> &R, const Eigen::Matrix<double,3,1> &t)
{
    cv::Mat cvMat = cv::Mat::eye(4,4,CV_32F);
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            cvMat.at<float>(i,j)=R(i,j);
        }
    }
    for(int i=0;i<3;i++)
    {
        cvMat.at<float>(i,3)=t(i);
    }

    return cvMat.clone();
}

Eigen::Matrix<double,3,1> Converter::toVector3d(const cv::Mat &cvVector)
{
    Eigen::Matrix<double,3,1> v;
    v << cvVector.at<float>(0), cvVector.at<float>(1), cvVector.at<float>(2);

    return v;
}

Eigen::Matrix<double,3,1> Converter::toVector3d(const cv::Point3f &cvPoint)
{
    Eigen::Matrix<double,3,1> v;
    v << cvPoint.x, cvPoint.y, cvPoint.z;

    return v;
}

Eigen::Matrix<double,3,3> Converter::toMatrix3d(const cv::Mat &cvMat3)
{
    Eigen::Matrix<double,3,3> M;

    M << cvMat3.at<float>(0,0), cvMat3.at<float>(0,1), cvMat3.at<float>(0,2),
         cvMat3.at<float>(1,0), cvMat3.at<float>(1,1), cvMat3.at<float>(1,2),
         cvMat3.at<float>(2,0), cvMat3.at<float>(2,1), cvMat3.at<float>(2,2);

    return M;
}

std::vector<float> Converter::toQuaternion(const cv::Mat &M)
{
    Eigen::Matrix<double,3,3> eigMat = toMatrix3d(M);
    Eigen::Quaterniond q(eigMat);

    std::vector<float> v(4);
    v[0] = q.x();
    v[1] = q.y();
    v[2] = q.z();
    v[3] = q.w();

    return v;
}

//[active slam]
float Converter::bboxOverlapratio(const cv::Rect& rect1, const cv::Rect& rect2)
{
    int overlap_area = (rect1&rect2).area();
    return (float)overlap_area/((float)(rect1.area()+rect2.area()-overlap_area));
}

float Converter::bboxOverlapratioLatter(const cv::Rect& rect1, const cv::Rect& rect2)
{
    int overlap_area = (rect1&rect2).area();
    return (float)overlap_area/((float)(rect2.area()));
}

float Converter::bboxOverlapratioFormer(const cv::Rect& rect1, const cv::Rect& rect2)
{
    int overlap_area = (rect1&rect2).area();
    return (float)overlap_area/((float)(rect1.area()));
}

//cv::Mat Converter::toCvMat(const Eigen::Matrix3d &m)
//{
//    cv::Mat cvMat(3,3,CV_32F);
//    for(int i=0;i<3;i++)
//        for(int j=0; j<3; j++)
//            cvMat.at<float>(i,j)=m(i,j);
//
//    return cvMat.clone();
//}

Eigen::Matrix4d Converter::cvMattoMatrix4d(const cv::Mat &cvMat4) {
    Eigen::Matrix4d M;

    M << cvMat4.at<float>(0, 0), cvMat4.at<float>(0, 1), cvMat4.at<float>(0, 2), cvMat4.at<float>(0, 3),
         cvMat4.at<float>(1, 0), cvMat4.at<float>(1, 1), cvMat4.at<float>(1, 2), cvMat4.at<float>(1, 3),
         cvMat4.at<float>(2, 0), cvMat4.at<float>(2, 1), cvMat4.at<float>(2, 2), cvMat4.at<float>(2, 3),
         cvMat4.at<float>(3, 0), cvMat4.at<float>(3, 1), cvMat4.at<float>(3, 2), cvMat4.at<float>(3, 3);

    return M;
}


Eigen::Matrix4d Converter::Quation2Eigen(const double qx, const double qy, const double qz, const double qw, const double tx,
                                 const double ty, const double tz) {

    Eigen::Quaterniond quaternion(Eigen::Vector4d(qx, qy, qz, qw));
    Eigen::AngleAxisd rotation_vector(quaternion);
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.rotate(rotation_vector);
    T.pretranslate(Eigen::Vector3d(tx, ty, tz));
    Eigen::Matrix4d Pose_eigen = T.matrix();
    return Pose_eigen;
}

cv::Mat Converter::Quation2CvMat(const double qx, const double qy, const double qz, const double qw, const double tx, const double ty, const double tz  ) {
    return toCvMat(
            Quation2Eigen(qx, qy, qz, qw, tx, ty, tz )
    );
}

Eigen::Isometry3d  Converter::Matrix4dtoIsometry3d(const Eigen::Matrix4d &matrix) {
    Eigen::Isometry3d Iso=Eigen::Isometry3d::Identity();                // 虽然称为3d，实质上是4＊4的矩阵
    //不能直接赋值
    //    T1<< 1.000000e+00, 1.197624e-11, 1.704639e-10, 3.214096e-14,
    //            1.197625e-11, 1.197625e-11, 3.562503e-10, -1.998401e-15,
    //            1.704639e-10, 3.562503e-10, 1.000000e+00, -4.041212e-14,
    //                       0,            0,            0,              1;

    //----1.对各个元素赋值----
    Iso(0, 0) = matrix(0, 0), Iso(0, 1) = matrix(0, 1), Iso(0, 2) = matrix(0, 2), Iso(0, 3) = matrix(0, 3);
    Iso(1, 0) = matrix(1, 0), Iso(1, 1) = matrix(1, 1), Iso(1, 2) = matrix(1, 2), Iso(1, 3) = matrix(1, 3);
    Iso(2, 0) = matrix(2, 0), Iso(2, 1) = matrix(2, 1), Iso(2, 2) = matrix(2, 2), Iso(2, 3) = matrix(2, 3);
    Iso(3, 0) = matrix(3, 0), Iso(3, 1) = matrix(3, 1), Iso(3, 2) = matrix(3, 2), Iso(3, 3) = matrix(3, 3);

    return Iso;
}

Eigen::Matrix4d Converter::Isometry3dtoMatrix4d(const Eigen::Isometry3d &Iso ){
    return Iso.matrix();
}

Eigen::Isometry3d Converter::cvMattoIsometry3d(const cv::Mat &cvMat4){
    return Matrix4dtoIsometry3d(
            cvMattoMatrix4d( cvMat4 )
            );
}

Eigen::Quaterniond Converter::toQuaterniond(const Eigen::Isometry3d &Iso){
    Eigen::Quaterniond q = Eigen::Quaterniond(Iso.rotation());
    return q;
}

Eigen::Quaterniond Converter::toQuaterniond(const Eigen::Matrix4d &matrix ){
    return toQuaterniond(
            Matrix4dtoIsometry3d(matrix)
    );
}

Eigen::Quaterniond Converter::toQuaterniond(const cv::Mat &mat ){
    return toQuaterniond(
            cvMattoIsometry3d(mat)
    );
}

} //namespace ORB_SLAM
