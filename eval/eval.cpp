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

float fx ;
float fy ;
float cx ;
float cy ,ImageWidth,ImageHeight;

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
    //Eigen::Quaterniond q_gt = Converter::ExtractQuaterniond(ob_gt->mCuboid3D.pose_mat);
    //Eigen::Quaterniond q_my = Converter::ExtractQuaterniond(ob_my->mCuboid3D.pose_mat);
    //double cos_theta = q_gt.dot(q_my);
    //double theta = std::acos(cos_theta)/M_PI * 180.0;

    Eigen::Quaterniond q_gt = Converter::ExtractQuaterniond(ob_gt->mCuboid3D.pose_mat);
    Eigen::Quaterniond q_my = Converter::ExtractQuaterniond(ob_my->mCuboid3D.pose_mat);
    Eigen::Vector3d euler_gt = q_gt.toRotationMatrix().eulerAngles(2,1,0);
    Eigen::Vector3d euler_my = q_my.toRotationMatrix().eulerAngles(2,1,0);
    double theta = std::abs(euler_gt[0]-euler_my[0]);
    return theta;
}

double mean(const std::vector<double>& vec) {
    double sum = std::accumulate(vec.begin(), vec.end(), 0.0);
    double mean = sum / vec.size();
    return mean;
}

void read_view(const std::string filePath, std::vector<cv::Mat>& views){
    std::ifstream infile(filePath, std::ios::in);
    if (!infile.is_open())
    {
        std::cout << "open fail: " << filePath << " " << endl;
        exit(233);
    }
    else
    {
        std::cout << "read VIEWs.txt" << std::endl;
    }

    std::vector<double> row;

    cv::Mat cam_pose_mat;
    int mnid_current = -1;
    //string s0;
    //getline(infile, s0);  注销掉无用的line
    views.clear();
    string line;

    while (getline(infile, line))
    {
        istringstream istr(line);
        double num,tx,ty,tz,qx,qy,qz,qw;

        //存nbv
        // 四元数   v[0] = q.x();
        //v[1] = q.y();
        //v[2] = q.z();
        //v[3] = q.w();
        //<< i << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
        //  << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

        //存物体< q[0] << " " << q[1] << " " << q[2] << " " << q[3]
        //f_point     << "1 "  //物体
        //            << object->mnId << "   "
        //            << object->mnClass << " "
        //            << object->mnConfidence_foractive << " "
        //            << object->mvpMapObjectMappoints.size() << "     "
        //            << pose.translation().x() << " "
        //            << pose.translation().y() << " "
        //            << pose.translation().z()<< "     "
        //            << pose.rotation().x() << " "
        //            << pose.rotation().y() << " "
        //            << pose.rotation().z() << " "
        //            << pose.rotation().w() << "     "
        //            << object->mCuboid3D.lenth << " "
        //            << object->mCuboid3D.width << " "
        //            << object->mCuboid3D.height << " "
        //            << endl;

        //说明四元数的顺序都是xyzw

        istr >> num;

        double temp;
        Eigen::MatrixXd object_poses(1, 8); ;
        istr >> temp;  object_poses(0) = temp;  //obj->mCuboid3D.cuboidCenter0 = temp;
        istr >> temp;  object_poses(1) = temp;  //obj->mCuboid3D.cuboidCenter1 = temp;
        istr >> temp;  object_poses(2) = temp;  //obj->mCuboid3D.cuboidCenter2 = temp;
        istr >> temp;  object_poses(3) = temp;
        istr >> temp;  object_poses(4) = temp;
        istr >> temp;  object_poses(5) = temp;
        istr >> temp;  object_poses(6) = temp;
        g2o::SE3Quat cam_pose_se3(object_poses.row(0).head(7));

        cv::Mat nbv_pose = ORB_SLAM2::Converter::toCvMat(cam_pose_se3);

        views.push_back(nbv_pose);

        row.clear();
        istr.clear();
        line.clear();
    }
}

cv::Point2f WorldToImg(cv::Mat &PointPosWorld, cv::Mat& T_world_to_frame)
{

    //std::cout<<"  3D point: "<<PointPosWorld<<std::endl;
    //std::cout<<"  Camera Pose: "<<T_world_to_frame<<std::endl;
    // world.
    cv::Mat T_frame_to_world = T_world_to_frame.inv();
    const cv::Mat Rcw = T_frame_to_world.rowRange(0, 3).colRange(0, 3);
    const cv::Mat tcw = T_frame_to_world.rowRange(0, 3).col(3);

    // camera.
    cv::Mat PointPosCamera = Rcw * PointPosWorld + tcw;

    const float xc = PointPosCamera.at<float>(0);
    const float yc = PointPosCamera.at<float>(1);
    const float invzc = 1.0 / PointPosCamera.at<float>(2);
    //std::cout<<"  相机内参: fx:"<<fx<<", fy:"<<fy<<", cx:"<<cx<<", cy:"<<cy<<std::endl;
    //std::cout<<"  xc:"<<xc<<", yc:"<<yc<<", invzc"<<invzc<<std::endl;
    // image.
    float u = fx * xc * invzc + cx;
    float v = fy * yc * invzc + cy;
    //std::cout<<"  2D point: u:"<<u<<", v:"<<v<<std::endl<<std::endl<<std::endl;;

    return cv::Point2f(u, v);
}

cv::Point2f object_to_frame(geometry_msgs::Point& p_ob, ORB_SLAM2::Object_Map* ob, cv::Mat& T_world_to_frame){
        // 坐标系变换矩阵
        Eigen::Matrix4d T_world_to_ob = Converter::cvMattoMatrix4d(ob->mCuboid3D.pose_mat);

        // 坐标点P在A坐标系下的坐标
        double x_a = p_ob.x;
        double y_a = p_ob.y;
        double z_a = p_ob.z;

        // 将点P从A坐标系变换到B坐标系
        Eigen::Vector4d P_ob_4(x_a, y_a, z_a, 1.0);  // 注意点P_a需要补一个1，才能与矩阵T相乘
        Eigen::Vector4d P_world_4 = T_world_to_ob * P_ob_4;
        //std::cout<<"  3D EIGEN point: "<<P_world_4 <<std::endl;

        Eigen::Vector3d P_world_3_eigen(P_world_4(0), P_world_4(1), P_world_4(2));
        cv::Mat P_world_3 = Converter::toCvMat(P_world_3_eigen);
        //std::cout<<"  3D CVMAT point: "<<P_world_3 <<std::endl;
        cv::Point2f point = WorldToImg(P_world_3, T_world_to_frame);
        //std::cout<<"  2D point: "<<point<<std::endl;

        return point;
}

cv::Rect project(Object_Map* ob, cv::Mat& KF){
    //     8------7
    //    /|     /|
    //   / |    / |
    //  5------6  |
    //  |  4---|--3
    //  | /    | /
    //  1------2
    // lenth ：corner_2[0] - corner_1[0]
    // width ：corner_2[1] - corner_3[1]
    // height：corner_2[2] - corner_6[2]
    double width_half = ob->mCuboid3D.width  /2.0;
    double length_half = ob->mCuboid3D.lenth / 2.0;
    double height_half = ob->mCuboid3D.height / 2.0;

    geometry_msgs::Point p1;   p1.x = -1 * length_half; p1.y = width_half; p1.z = -1*height_half;
    geometry_msgs::Point p2;   p2.x = -1 * length_half; p2.y = -1 * width_half; p2.z = -1*height_half;
    geometry_msgs::Point p3;   p3.x = length_half; p3.y = -1 * width_half; p3.z = -1*height_half;
    geometry_msgs::Point p4;   p4.x = length_half; p4.y = width_half; p4.z =  -1*height_half;
    geometry_msgs::Point p5;   p5.x = -1 * length_half; p5.y = width_half; p5.z =  height_half;
    geometry_msgs::Point p6;   p6.x = -1 * length_half; p6.y = -1 * width_half; p6.z =  height_half;
    geometry_msgs::Point p7;   p7.x = length_half; p7.y = -1 * width_half; p7.z =  height_half;
    geometry_msgs::Point p8;   p8.x = length_half; p8.y = width_half; p8.z =  height_half;

    cv::Point2f p1_frame = object_to_frame(p1, ob, KF);
    cv::Point2f p2_frame = object_to_frame(p2, ob, KF);
    cv::Point2f p3_frame = object_to_frame(p3, ob, KF);
    cv::Point2f p4_frame = object_to_frame(p4, ob, KF);
    cv::Point2f p5_frame = object_to_frame(p5, ob, KF);
    cv::Point2f p6_frame = object_to_frame(p6, ob, KF);
    cv::Point2f p7_frame = object_to_frame(p7, ob, KF);
    cv::Point2f p8_frame = object_to_frame(p8, ob, KF);

    //c++有没有从一堆double中选出最大的函数
    std::vector<double> x_pt = {p1_frame.x, p2_frame.x, p3_frame.x, p4_frame.x, p5_frame.x, p6_frame.x, p7_frame.x, p8_frame.x };
    std::vector<double> y_pt = {p1_frame.y, p2_frame.y, p3_frame.y, p4_frame.y, p5_frame.y, p6_frame.y, p7_frame.y, p8_frame.y };
    //double left_top_x = *std::min_element(values_x.begin(), values_x.end());
    //double left_top_y = *std::min_element(values_y.begin(), values_y.end());
    //double right_bottom_x = *std::max_element(values_x.begin(), values_x.end());
    //double right_bottom_y = *std::max_element(values_y.begin(), values_y.end());
    //
    //// make insure in the image.
    //if (left_top_x < 0)
    //    left_top_x = 0;
    //if (left_top_y < 0)
    //    left_top_y = 0;
    //if (right_bottom_x > ColorImage.cols)
    //    right_bottom_x = ColorImage.cols;
    //if (right_bottom_y > ColorImage.rows)
    //    right_bottom_y = ColorImage.rows;

    //double width = right_bottom_x - left_top_x;
    //double height= right_bottom_y - left_top_y;
    //
    //cv::Rect RectPredict = cv::Rect(left_top_x, left_top_y, width, height);

    sort(x_pt.begin(), x_pt.end());
    sort(y_pt.begin(), y_pt.end());
    float x_min = x_pt[0];
    float x_max = x_pt[x_pt.size() - 1];
    float y_min = y_pt[0];
    float y_max = y_pt[y_pt.size() - 1];

    std::cout<<"  x_min:"<<x_min<<", x_max:"<<x_max<<", y_min"<<y_min<<", y_max"<<y_max;

    if(x_max < 0 || y_max < 0 || x_min > ImageWidth || y_min > ImageHeight){
        std::cout<<"超出视场外"<<std::endl;
        return cv::Rect(0,0,0,0);
    }



    // make insure in the image.
    if (x_min < 0)
        x_min = 0;
    if (y_min < 0)
        y_min = 0;
    if (x_max > ImageWidth)
        x_max = ImageWidth;
    if (y_max > ImageHeight)
        y_max = ImageHeight;

    // the bounding box constructed by object feature points.
    // notes: 视野范围内的特征点
    // 用于data associate中
    cv::Rect RectPredict = cv::Rect(x_min, y_min, x_max - x_min, y_max - y_min);

    std::cout<<", 修正后  RectPredict: "<<RectPredict<<std::endl<<std::endl;;
    return RectPredict;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "RGBD");
    ros::start();
    //当前文件路径
    std::string current_file_path = __FILE__;
    std::string current_folder_path = current_file_path.substr(0, current_file_path.find_last_of("/\\"));
    WORK_SPACE_PATH = current_folder_path + "/" + "../";
    yamlfile_object = "kinectv1.yaml";

    cv::FileStorage fSettings(WORK_SPACE_PATH + "/config/"+ yamlfile_object,  cv::FileStorage::READ );
    fx = fSettings["Camera.fx"];
    fy = fSettings["Camera.fy"];
    cx = fSettings["Camera.cx"];
    cy = fSettings["Camera.cy"];
    ImageWidth = fSettings["Camera.width"];
    ImageHeight = fSettings["Camera.height"];

    vector<ORB_SLAM2::Object_Map* > obs_gt, obs_model;

    string groud_truth_path = current_folder_path + "/" + "groudtruth.txt";
    ReadLocalObjects(groud_truth_path, obs_gt);
    std::cout<<"[真实物体的数量]:" <<obs_gt.size()<<std::endl;
    string my_objects_path = current_folder_path + "/" + "Objects_with_points_for_read.txt";
    ReadLocalObjects(my_objects_path, obs_model);
    std::cout << "[建模物体的数量]:" << obs_model.size() << std::endl;
    std::string NBV_filePath, camera_filePath;
    NBV_filePath = "/home/zhjd/active_eao/src/active_eao/eval/GlobalNBV.txt";
    camera_filePath = "/home/zhjd/active_eao/src/active_eao/eval/KeyFrameTrajectory.txt";
    std::vector<cv::Mat> NBVs, cameras;
    read_view(NBV_filePath, NBVs);
    read_view(camera_filePath, cameras);

    //计算轨迹长度
    double distance_all = 0;
    for(int i=1; i<cameras.size(); i++){
        auto KF_1 = cameras[i].clone();
        auto KF_0 = cameras[i-1].clone();
        double dis = sqrt(
                (KF_1.at<float>(0,3)-KF_0.at<float>(0,3)) * (KF_1.at<float>(0,3)-KF_0.at<float>(0,3))  +
                (KF_1.at<float>(1,3)-KF_0.at<float>(1,3)) * (KF_1.at<float>(1,3)-KF_0.at<float>(1,3))  +
                (KF_1.at<float>(2,3)-KF_0.at<float>(2,3)) * (KF_1.at<float>(2,3)-KF_0.at<float>(2,3))
                );
        distance_all += dis;
    }

    //计算非遮挡度
    double non_occlusion_all = 0;

    double IouThreshold = 0.6;
    for (size_t i = 0, iend = cameras.size(); i < iend; i++) {

        cv::Mat KF = cameras[i].clone();
        double non_occlusion = 0;
        int num=0; //当前帧中有重叠的次数
        for(auto obj3d: obs_gt){

            cv::Rect obj2d = project(obj3d, KF);

            for(auto obj3d_2: obs_gt) {
                cv::Rect obj2d_2 = project(obj3d_2, KF);

                float Iou = Converter::bboxOverlapratio(obj2d, obj2d_2);
                //std::cout<<" bboxOverlapratio: "<<Iou<<std::endl;
                if(Iou>0){
                    num ++;
                    non_occlusion += (1-Iou);
                }
            }
        }
        if(num>0)
            non_occlusion /= (double)num;
        std::cout<<" 【one frame non_occlusion】: "<<non_occlusion<<std::endl;

        non_occlusion_all += non_occlusion;

        bool debug_iou_view= false;
        if(debug_iou_view){
            //cv::Mat mat_test = mpCurrentFrame->mColorImage.clone();
            //cv::Scalar color = (200, 0, 0);
            //cv::rectangle(mat_test, RectCurrent, (0, 0, 255), 2);
            //cv::rectangle(mat_test, RectPredict, (255, 0, 0), 2);
            //cv::putText(mat_test, std::to_string(Iou), cv::Point(0, 500), cv::FONT_HERSHEY_DUPLEX, 1.0,
            //            (0, 255, 0), 2);
            //cv::resize(mat_test, mat_test, cv::Size(640 * 0.5, 480 * 0.5), 0, 0, cv::INTER_CUBIC);
            //cv::imshow("[MotionIou]", mat_test);
        }
    }
    non_occlusion_all /= (double)cameras.size();
    std::cout<<" size: NBV "<<NBVs.size()<<",  camera "<<cameras.size()<<std::endl;
    std::cout<<" Object GT数量: "<<obs_gt.size()<<std::endl;
    std::cout<<" 注意核对物体真值的数量，是否正确。 "<<std::endl;
    std::cout<<" distance: "<<distance_all<<std::endl;
    std::cout<<" 非遮挡度: "<<non_occlusion_all<<std::endl<<std::endl;


    //计算IOU和theta。从my objects中挑选，最合适的一个，作为比较的对象
    //vector<vector<ORB_SLAM2::Object_Map* > > obs_iou_match, obs_dis_match;
    vector<double > IOUs, DISs, thetas;  //用于存储各个“真值物体”指标，计算得到的最佳值。
    //obs_iou_match.resize(obs_gt.size());
    //obs_dis_match.resize(obs_gt.size());
    IOUs.resize(obs_gt.size());
    DISs.resize(obs_gt.size());
    thetas.resize(obs_gt.size());

    int theta_valid = 0;
    for(int i=0; i<obs_gt.size(); i++){

        //
        //auto ob_gt = obs_gt[i];
        //
        ////计算最大IOU，并将最大IOU的几个物体作为最匹配的物体
        //double IOU_max = 0;
        //
        ////计算最小中心差距，并将最小中心差距的几个物体作为最匹配的物体
        //double dis_min = 1000;
        //
        //for(auto ob_my : obs_my){
        //    //计算最大IOU
        //    double IOU = compute_IOU(ob_gt, ob_my);
        //
        //    if(IOU>IOU_max){
        //        IOU_max = IOU;
        //        obs_iou_match[i].clear();
        //        obs_iou_match[i].push_back(ob_my);
        //        IOUs[i]=IOU_max;
        //    }
        //    else if(IOU==IOU_max)
        //        obs_iou_match[i].push_back(ob_my);
        //
        //    //计算最小中心差距
        //    double dis = compute_dis(ob_gt, ob_my);
        //
        //    if(dis<dis_min){
        //        dis_min = dis;
        //        obs_dis_match[i].clear();
        //        obs_dis_match[i].push_back(ob_my);
        //        DISs[i]=dis_min;
        //    }
        //    else if(dis==dis_min)
        //        obs_dis_match[i].push_back(ob_my);
        //}
        //std::cout << "[计算IOU]:" << yolo_id[obs_iou_match[i][0]->mnClass] << "的IOU为[" << IOU_max << "]" << std::endl;
        //std::cout << "[计算DIS]:" << yolo_id[obs_iou_match[i][0]->mnClass] << "的DIS为[" << dis_min << "]" << std::endl;
        //



        auto ob_gt = obs_gt[i];
        auto ob_model = obs_model[i];

        //计算IOU
        ORB_SLAM2::Object_Map* ob1 = new Object_Map();
        ob1->mCuboid3D.x_max = ob_gt->mCuboid3D.lenth/2.0 * (1.0);
        ob1->mCuboid3D.x_min = ob_gt->mCuboid3D.lenth/2.0 * (-1.0);
        ob1->mCuboid3D.y_max = ob_gt->mCuboid3D.width/2.0 * (1.0);
        ob1->mCuboid3D.y_min = ob_gt->mCuboid3D.width/2.0 * (-1.0);
        ob1->mCuboid3D.z_max = ob_gt->mCuboid3D.height/2.0 * (1.0);
        ob1->mCuboid3D.z_min = ob_gt->mCuboid3D.height/2.0 * (-1.0);
        ob1->mCuboid3D.lenth = ob_gt->mCuboid3D.lenth;
        ob1->mCuboid3D.width = ob_gt->mCuboid3D.width;
        ob1->mCuboid3D.height = ob_gt->mCuboid3D.height;

        ORB_SLAM2::Object_Map* ob2 = new Object_Map();
        ob2->mCuboid3D.x_max = ob_model->mCuboid3D.lenth/2.0 * (1.0);
        ob2->mCuboid3D.x_min = ob_model->mCuboid3D.lenth/2.0 * (-1.0);
        ob2->mCuboid3D.y_max = ob_model->mCuboid3D.width/2.0 * (1.0);
        ob2->mCuboid3D.y_min = ob_model->mCuboid3D.width/2.0 * (-1.0);
        ob2->mCuboid3D.z_max = ob_model->mCuboid3D.height/2.0 * (1.0);
        ob2->mCuboid3D.z_min = ob_model->mCuboid3D.height/2.0 * (-1.0);
        ob2->mCuboid3D.lenth = ob_model->mCuboid3D.lenth;
        ob2->mCuboid3D.width = ob_model->mCuboid3D.width;
        ob2->mCuboid3D.height = ob_model->mCuboid3D.height;

        double IOU = compute_IOU(ob1, ob2);
        IOUs[i]=IOU;
        std::cout <<  yolo_id[ob_gt->mnClass] << "的IOU为[" << IOU << "]" ;

        //计算中心差距
        double dis = compute_dis(ob_gt, ob_model);
        DISs[i]=dis;
        std::cout << ",  DIS为[" << dis << "]" ;

        //计算夹角
        if( ob_gt->mnClass==63 || ob_gt->mnClass==66 || ob_gt->mnClass==73){
            double theta = compute_theta(ob_gt, ob_model);
            thetas[i] = theta;
            std::cout << ", 偏角为[" << theta << "]" << std::endl;
            theta_valid++;
        }
        else
            std::cout<< std::endl;

        //计算尺寸
    }

    std::cout<< std::endl;
    std::cout<< std::endl;
    std::cout<< std::endl;
    std::cout<<"[物体IOU平均值]:" <<accumulate(IOUs.begin(), IOUs.end(), 0.0)/ (double)IOUs.size()<<std::endl;

    std::cout<<"[物体DIS平均值]:" <<accumulate(DISs.begin(), DISs.end(), 0.0)/ (double)DISs.size()<<std::endl;

    std::cout<<"[物体偏角平均值]:" <<accumulate(thetas.begin(), thetas.end(), 0.0)/ (double)theta_valid<<std::endl;

    ros::shutdown();
}
