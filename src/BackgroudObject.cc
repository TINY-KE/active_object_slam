//
// Created by zhjd on 5/19/23.
//

#include "BackgroudObject.h"

namespace ORB_SLAM2{

    bool BackgroudObject::include(Object_Map* fo){
        //前景物体的中心
        Eigen::Vector3d  fo_centor;
        fo_centor = Eigen::Vector3d(fo->mCuboid3D.cuboidCenter.x(), fo->mCuboid3D.cuboidCenter.y(), fo->mCuboid3D.cuboidCenter.z());

        //背景物体的中心
        cv::Mat T_w_bo = this->pose_mat;
        cv::Mat T_bo_w = T_w_bo.inv();

        // 将点从世界坐标系变换到椭球体坐标系下（即通过椭球体的位姿将点变换到与椭球体同一坐标系）
        Eigen::Vector4d centor_w = real_to_homo_coord<double>(fo_centor);
        Eigen::Vector4d centor_bo = Converter::cvMattoMatrix4d(T_bo_w) * centor_w;
        Eigen::Vector3d centor_bo_3 = homo_to_real_coord<double>(centor_bo);

        double x = std::abs(centor_bo_3[0]);
        double y = std::abs(centor_bo_3[1]);
        double z = std::abs(centor_bo_3[2]);
        //std::cout<<"[debug IOU,x]:" <<x<<std::endl;
        //std::cout<<"[debug IOU,y]:" <<y<<std::endl;
        //std::cout<<"[debug IOU,z]:" <<z<<std::endl;
        // 将点在各个坐标轴上的坐标与椭球体在各个坐标轴上的半径进行比较，若点在三个坐标轴上的坐标都小于椭球体在各个坐标轴上的半径，则返回true，否则返回false。
        if(x <this->length/2.0 && y < this->width/2.0 && z < this->height/2.0)
            return true;
        else
            return false;
    }

    void BackgroudObject::computeValue(std::vector<Object_Map*> FOs) {
        FO_num = 0;
        FO_num_notend = 0;
        for(auto fo: FOs){
            //先默认已经结束建图
            this->end_activemapping = true;
            //如果还有前景物体没有完成，则将 end_.. 改回false
            if( this->include(fo) )
            {
                FO_num ++;
                if( fo->end_build == false){
                    this->end_activemapping = false;
                    FO_num_notend ++;
                }
            }
        }
    }

    bool BackgroudObject::return_end_active_mapping()
    {
        return  end_activemapping;
    }

    void BackgroudObject::computePose() {
        float cp = cos(0.0);
        float sp = sin(0.0);
        float sr = sin(0.0);
        float cr = cos(0.0);
        float sy = sin(0.0);
        float cy = cos(0.0);
        Eigen::Matrix<double, 3, 3> REigen;
        REigen << cp * cy, (sr * sp * cy) - (cr * sy), (cr * sp * cy) + (sr * sy),
            cp * sy, (sr * sp * sy) + (cr * cy), (cr * sp * sy) - (sr * cy),
            -sp, sr * cp, cr * cp;
        cv::Mat Ryaw = Converter::toCvMat(REigen);

        // Transformation matrix.
        cv::Mat Twobj = cv::Mat::eye(4, 4, CV_32F);
        const cv::Mat Rcw = Twobj.rowRange(0, 3).colRange(0, 3);
        const cv::Mat tcw = Twobj.rowRange(0, 3).col(3);

        cv::Mat R_result = Rcw * Ryaw;

        Twobj.at<float>(0, 0) = R_result.at<float>(0, 0);
        Twobj.at<float>(0, 1) = R_result.at<float>(0, 1);
        Twobj.at<float>(0, 2) = R_result.at<float>(0, 2);
        Twobj.at<float>(0, 3) = mean_x;

        Twobj.at<float>(1, 0) = R_result.at<float>(1, 0);
        Twobj.at<float>(1, 1) = R_result.at<float>(1, 1);
        Twobj.at<float>(1, 2) = R_result.at<float>(1, 2);
        Twobj.at<float>(1, 3) = mean_y;

        Twobj.at<float>(2, 0) = R_result.at<float>(2, 0);
        Twobj.at<float>(2, 1) = R_result.at<float>(2, 1);
        Twobj.at<float>(2, 2) = R_result.at<float>(2, 2);
        Twobj.at<float>(2, 3) = mean_z;

        Twobj.at<float>(3, 0) = 0;
        Twobj.at<float>(3, 1) = 0;
        Twobj.at<float>(3, 2) = 0;
        Twobj.at<float>(3, 3) = 1;

        pose_mat = Twobj;
    }
}