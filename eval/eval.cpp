//
// Created by zhjd on 5/12/23.
//

//#include "evo.h"

#include <iostream>
#include <fstream>
#include <Eigen/Core>
#include <algorithm>
#include "Object.h"
#include <cmath>

std::string WORK_SPACE_PATH = "";
std::string yamlfile_object = "";
int mam_loop = 1;
bool MotionIou_flag = true;
bool NoPara_flag = true;
bool ProIou_flag = true;
bool Ttest_flag = true;
bool iforest_flag = true;
bool little_mass_flag = true;
bool ProIou_only30_flag = true;

using namespace ORB_SLAM2;

typedef Eigen::Matrix<double, 9, 1> Vector9d;


Eigen::Vector3d TransformPoint(Eigen::Vector3d &point, const Eigen::Matrix4d &T)
{
    Eigen::Vector4d Xc = real_to_homo_coord<double>(point);
    Eigen::Vector4d Xw = T * Xc;
    Eigen::Vector3d xyz_w = homo_to_real_coord<double>(Xw);
    return xyz_w;
}

bool JudgeInsideRec(Eigen::Vector3d& X, ORB_SLAM2::Object_Map* e)
{
    cv::Mat Two = e->mCuboid3D.pose_mat;
    cv::Mat Tow = Two.inv();
    // 将点从世界坐标系变换到椭球体坐标系下（即通过椭球体的位姿将点变换到与椭球体同一坐标系）
    Eigen::Vector3d X_local = TransformPoint(X, Converter::cvMattoMatrix4d(Tow));

    double x = std::abs(X_local[0]);
    double y = std::abs(X_local[1]);
    double z = std::abs(X_local[2]);
    //std::cout<<"[debug IOU,x]:" <<x<<std::endl;
    //std::cout<<"[debug IOU,y]:" <<y<<std::endl;
    //std::cout<<"[debug IOU,z]:" <<z<<std::endl;
    // 将点在各个坐标轴上的坐标与椭球体在各个坐标轴上的半径进行比较，若点在三个坐标轴上的坐标都小于椭球体在各个坐标轴上的半径，则返回true，否则返回false。
    if(x < e->mCuboid3D.lenth/2.0 && y < e->mCuboid3D.width/2.0 && z < e->mCuboid3D.height/2.0)
        return true;
    else
        return false;
}

// obj : instance x y z r p yaw a b c
//           0    1 2 3 4 5  6  7 8 9
// Resolution: 点的分辨率 m. 总共Sample的点为 (Volumn/Resolution^3)
// Shape: 0 Quadrics, 1 Rectangle
// ORB_SLAM2::Object_Map* ob_gt, ORB_SLAM2::Object_Map* ob_my
double MonteCarloIoU(ORB_SLAM2::Object_Map* ob1, ORB_SLAM2::Object_Map* ob2, int sample_num = 10, int shape = 1)
{
    bool bDebug = false;

    // 获得立方体.
    // 取 x,y,z最大值
    double x_max = MAX(ob1->mCuboid3D.x_max, ob2->mCuboid3D.x_max);
    double x_min = MIN(ob1->mCuboid3D.x_min, ob2->mCuboid3D.x_min);
    double y_max = MAX(ob1->mCuboid3D.y_max, ob2->mCuboid3D.y_max);
    double y_min = MIN(ob1->mCuboid3D.y_min, ob2->mCuboid3D.y_min);
    double z_max = MAX(ob1->mCuboid3D.z_max, ob2->mCuboid3D.z_max);
    double z_min = MIN(ob1->mCuboid3D.z_min, ob2->mCuboid3D.z_min);
    //std::cout<<"[ob1->mCuboid3D.z_min, ob2->mCuboid3D.z_min]:" <<ob1->mCuboid3D.z_min<<" "<< ob2->mCuboid3D.z_min<<std::endl;
    double length_x = x_max - x_min;   //debug 0.323043 0.272007 0.103261
    double length_y = y_max - y_min;
    double length_z = z_max - z_min;

    double total_volumn = length_x * length_y * length_z;
    //double resolution = pow(total_volumn / sample_num, 1.0/3.0);
    double resolution_x = length_x / sample_num;
    double resolution_y = length_y / sample_num;
    double resolution_z = length_z / sample_num;

    // 随机 Sample 点云
    int total_num = 0;

    int ob1_num = 0;   //只在ob1中的点
    int ob2_num = 0;   //只在ob2中的点
    int both_num = 0;   //同时在ob1和2中的点

    //EllipsoidSLAM::PointCloud *pCloud = new EllipsoidSLAM::PointCloud;
    for (double x = x_min; x < x_max; x += resolution_x)
    {
        for (double y = y_min; y < y_max; y += resolution_y)
        {
            for (double z = z_min; z < z_max; z += resolution_z)
            {
                Eigen::Vector3d point_vec(x, y, z);   //在世界坐标系下
                Eigen::Vector4d X = real_to_homo_coord_vec<double>(point_vec);

                bool isInside_1, isInside_2;
                bool bUseQuadric = (shape==0); //shape 0时，启用二次曲面

                {
                    isInside_1 = JudgeInsideRec(point_vec, ob1);
                    isInside_2 = JudgeInsideRec(point_vec, ob2);
                }


                if (isInside_1)
                    ob1_num++;
                if (isInside_2)
                    ob2_num++;
                if (isInside_1 && isInside_2)
                    both_num++;

                total_num++;

                //可视化
                //EllipsoidSLAM::PointXYZRGB p;
                //p.x = x;
                //p.y = y;
                //p.z = z;
                //
                //// 默认颜色
                //p.r = 0;
                //p.g = 0;
                //p.b = 0;
                //
                //if (isInside_1)
                //    p.r = 255;
                //if (isInside_2)
                //    p.b = 255;
                //if (isInside_1 && isInside_2)
                //    p.g = 128;
                //p.size = 5;
                //pCloud->push_back(p);
            }
        }
    }

    // 统计
    if(bDebug){
        std::cout << "ob1/ob2/both : " << ob1_num << "/" << ob2_num << "/" << both_num << std::endl;
        std::cout << "Total Num : " << total_num << std::endl;
        //
        //// 可视化部分. 显示椭球体、所有数据点, 并按类别标记颜色
        //g2o::ellipsoid *pE1 = new g2o::ellipsoid(e1);
        //pE1->setColor(Eigen::Vector3d(0, 0, 1));
        //g2o::ellipsoid *pE2 = new g2o::ellipsoid(e2);
        //pE2->setColor(Eigen::Vector3d(1, 0, 0));
        ////pMap->ClearEllipsoidsVisual();
        ////pMap->addEllipsoidVisual(pE1);
        ////pMap->addEllipsoidVisual(pE2);
        ////
        ////pMap->clearPointCloud();
        ////pMap->addPointCloud(pCloud);
        //
        //// 显示大区域
        //Vector9d area_vec;
        //double x_center = (x_min + x_max) / 2;
        //double y_center = (y_min + y_max) / 2;
        //double z_center = (z_min + z_max) / 2;
        //double x_halfSize = (x_max - x_min) / 2;
        //double y_halfSize = (y_max - y_min) / 2;
        //double z_halfSize = (z_max - z_min) / 2;
        //area_vec << x_center, y_center, z_center, 0, 0, 0, x_halfSize, y_halfSize, z_halfSize;
        //g2o::ellipsoid *pE_Area = new g2o::ellipsoid;
        //pE_Area->fromMinimalVector(area_vec);
        //pE_Area->setColor(Eigen::Vector3d(0, 1, 0));
        ////pMap->addEllipsoidVisual(pE_Area);
        //
        //std::cout << "Show result of sampling for IoU ... Push any key to continue ... " << std::endl;
        //getchar();
    }

    // 输出结果.
    double IoU = both_num / double(ob1_num + ob2_num - both_num);
    if(bDebug) std::cout<<"[debug IOU]:" <<IoU<<std::endl;
    return IoU;
}

double compute_dis(ORB_SLAM2::Object_Map* ob_gt, ORB_SLAM2::Object_Map* ob_my){
   double dis_x =  ob_gt->mCuboid3D.pose_mat.at<float>(0,3)-ob_my->mCuboid3D.pose_mat.at<float>(0,3);
   double dis_y =  ob_gt->mCuboid3D.pose_mat.at<float>(1,3)-ob_my->mCuboid3D.pose_mat.at<float>(1,3);
   double dis_z =  ob_gt->mCuboid3D.pose_mat.at<float>(2,3)-ob_my->mCuboid3D.pose_mat.at<float>(2,3);
   double dis = sqrt( dis_x*dis_x + dis_y*dis_y + dis_z*dis_z   );
   return dis;
}

double compute_IOU(ORB_SLAM2::Object_Map* ob_gt, ORB_SLAM2::Object_Map* ob_my){
    return MonteCarloIoU( ob_gt, ob_my);
}

double compute_theta(ORB_SLAM2::Object_Map* ob_gt, ORB_SLAM2::Object_Map* ob_my){
    Eigen::Quaterniond q_gt = Converter::ExtractQuaterniond(ob_gt->mCuboid3D.pose_mat);
    Eigen::Quaterniond q_my = Converter::ExtractQuaterniond(ob_my->mCuboid3D.pose_mat);
    double cos_theta = q_gt.dot(q_my);
    double theta = std::acos(cos_theta)/M_PI * 180.0;
    return theta;
}

double mean(const std::vector<double>& vec) {
    double sum = std::accumulate(vec.begin(), vec.end(), 0.0);
    double mean = sum / vec.size();
    return mean;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "RGBD");
    ros::start();
    //当前文件路径
    std::string current_file_path = __FILE__;
    std::string current_folder_path = current_file_path.substr(0, current_file_path.find_last_of("/\\"));
    WORK_SPACE_PATH = current_folder_path + "/" + "../";
    yamlfile_object = "kinectv1.yaml";



    vector<ORB_SLAM2::Object_Map* > obs_gt, obs_my;

    string groud_truth_path = current_folder_path + "/" + "groudtruth.txt";
    ReadLocalObjects(groud_truth_path, obs_gt);
    std::cout<<"[真实物体的数量]:" <<obs_gt.size()<<std::endl;
    string my_objects_path = current_folder_path + "/" + "Objects.txt";
    ReadLocalObjects(my_objects_path, obs_my);
    std::cout<<"[建模物体的数量]:" <<obs_my.size()<<std::endl;

    //物体数量的准确性性
    double num_percent = 1.0 - (double)std::abs(static_cast<int>(obs_gt.size()-obs_my.size())) / obs_gt.size();


    //计算IOU和theta。从my objects中挑选，最合适的一个，作为比较的对象
    vector<vector<ORB_SLAM2::Object_Map* > > obs_iou_match, obs_dis_match;
    vector<double > IOUs, DISs, thetas;  //用于存储各个“真值物体”指标，计算得到的最佳值。
    obs_iou_match.resize(obs_gt.size());
    obs_dis_match.resize(obs_gt.size());
    IOUs.resize(obs_gt.size());
    DISs.resize(obs_gt.size());
    thetas.resize(obs_gt.size());


    for(int i=0; i<obs_gt.size(); i++){
        auto ob_gt = obs_gt[i];

        //计算最大IOU，并将最大IOU的几个物体作为最匹配的物体
        double IOU_max = 0;

        //计算最小中心差距，并将最小中心差距的几个物体作为最匹配的物体
        double dis_min = 1000;

        for(auto ob_my : obs_my){
            //计算最大IOU
            double IOU = compute_IOU(ob_gt, ob_my);

            if(IOU>IOU_max){
                IOU_max = IOU;
                obs_iou_match[i].clear();
                obs_iou_match[i].push_back(ob_my);
                IOUs[i]=IOU_max;
            }
            else if(IOU==IOU_max)
                obs_iou_match[i].push_back(ob_my);

            //计算最小中心差距
            double dis = compute_dis(ob_gt, ob_my);

            if(dis<dis_min){
                dis_min = dis;
                obs_dis_match[i].clear();
                obs_dis_match[i].push_back(ob_my);
                DISs[i]=dis_min;
            }
            else if(dis==dis_min)
                obs_dis_match[i].push_back(ob_my);
        }
        std::cout << "[计算IOU]:" << yolo_id[obs_iou_match[i][0]->mnClass] << "的IOU为[" << IOU_max << "]" << std::endl;
        std::cout << "[计算DIS]:" << yolo_id[obs_iou_match[i][0]->mnClass] << "的DIS为[" << dis_min << "]" << std::endl;

        //计算最小夹角
        double theta_min = 100;
        for(int j=0; j < obs_iou_match[i].size(); j++) {
            double theta = compute_theta(obs_gt[i], obs_iou_match[i][j]);
            if(theta<theta_min){
                theta_min = theta;
                thetas[i] = theta;
            }
        }


    }

    std::cout<<"[物体数量的准确性]:" <<num_percent<<std::endl;

    std::cout<<"[物体IOU平均值]:" <<accumulate(IOUs.begin(), IOUs.end(), 0.0)/ (double)IOUs.size()<<std::endl;

    std::cout<<"[物体DIS平均值]:" <<accumulate(DISs.begin(), DISs.end(), 0.0)/ (double)DISs.size()<<std::endl;

    std::cout<<"[物体偏角平均值]:" <<accumulate(thetas.begin(), thetas.end(), 0.0)/ (double)thetas.size()<<std::endl;

    ros::shutdown();
}
