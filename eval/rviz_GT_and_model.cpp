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

ros::Publisher publisher_GT;
ros::Publisher publisher_model;

Eigen::Vector3d TransformPoint(Eigen::Vector3d &point, const Eigen::Matrix4d &T)
{
    Eigen::Vector4d Xc = real_to_homo_coord<double>(point);
    Eigen::Vector4d Xw = T * Xc;
    Eigen::Vector3d xyz_w = homo_to_real_coord<double>(Xw);
    return xyz_w;
}

double angle_tune = 0;
geometry_msgs::Point world_to_frame(geometry_msgs::Point& oldp, ORB_SLAM2::Object_Map* ob){
        geometry_msgs::Point newp;

        // 坐标系变换矩阵
        Eigen::Matrix4d T = Converter::cvMattoMatrix4d(ob->mCuboid3D.pose_mat);

        // 坐标点P在A坐标系下的坐标
        double x_a = oldp.x;
        double y_a = oldp.y;
        double z_a = oldp.z;

        // 将点P从A坐标系变换到B坐标系
        Eigen::Vector4d P_a(x_a, y_a, z_a, 1.0);  // 注意点P_a需要补一个1，才能与矩阵T相乘
        Eigen::Vector4d P_b = T * P_a;

        // 输出结果
        double x_b = P_b(0);
        double y_b = P_b(1);
        double z_b = P_b(2);

        newp.x = x_b ;
        newp.y = y_b ;
        newp.z = z_b ;
        return newp;
}



int main(int argc, char **argv) {

    bool rviz_model = true;
    if(argc == 2)
        rviz_model = false;

    ros::init ( argc, argv, "rviz_GT_and_model" );
    ros::NodeHandle nh;
    publisher_GT = nh.advertise<visualization_msgs::Marker>("objectmap_groudtruth", 1000);
    publisher_model = nh.advertise<visualization_msgs::Marker>("objectmap", 1000);
    ros::start();
    //当前文件路径
    std::string current_file_path = __FILE__;
    std::string current_folder_path = current_file_path.substr(0, current_file_path.find_last_of("/\\"));
    WORK_SPACE_PATH = current_folder_path + "/" + "../";
    yamlfile_object = "kinectv1.yaml";



    vector<ORB_SLAM2::Object_Map* > obs_gt, obs_model;

    string groud_truth_path = current_folder_path + "/" + "groudtruth.txt";
    ReadLocalObjects(groud_truth_path, obs_gt);
    std::cout<<"[真实物体的数量]:" <<obs_gt.size()<<std::endl;
    string my_objects_path = current_folder_path + "/" + "Objects_with_points_for_read.txt";
    ReadLocalObjects(my_objects_path, obs_model);
    std::cout << "[建模物体的数量]:" << obs_model.size() << std::endl;


    ros::Rate rate(10);
    std::cout << "Publishing GT and Object Model"<< std::endl;
    while (nh.ok()){

        int id = 0;

        for(auto ob: obs_gt) {

            id++;

            //publish rviz
            visualization_msgs::Marker marker;
            marker.id = id;//++object_id_init;//object_id_init + i;
            float mObject_Duration=1;
            // marker.lifetime = ros::Duration(mObject_Duration);
            marker.header.frame_id= "map";
            marker.header.stamp=ros::Time::now();
            marker.type = visualization_msgs::Marker::LINE_LIST; //LINE_STRIP;
            marker.action = visualization_msgs::Marker::ADD;
            marker.color.r = 255.0; marker.color.g = 0.0; marker.color.b = 0.0; marker.color.a = 1.0;
            marker.scale.x = 0.01;
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



            marker.points.push_back(world_to_frame(p1, ob));
            marker.points.push_back(world_to_frame(p2, ob));
            marker.points.push_back(world_to_frame(p2, ob));
            marker.points.push_back(world_to_frame(p3, ob));
            marker.points.push_back(world_to_frame(p3, ob));
            marker.points.push_back(world_to_frame(p4, ob));
            marker.points.push_back(world_to_frame(p4, ob));
            marker.points.push_back(world_to_frame(p1, ob));

            marker.points.push_back(world_to_frame(p5, ob));
            marker.points.push_back(world_to_frame(p1, ob));
            marker.points.push_back(world_to_frame(p6, ob));
            marker.points.push_back(world_to_frame(p2, ob));
            marker.points.push_back(world_to_frame(p7, ob));
            marker.points.push_back(world_to_frame(p3, ob));
            marker.points.push_back(world_to_frame(p8, ob));
            marker.points.push_back(world_to_frame(p4, ob));

            marker.points.push_back(world_to_frame(p5, ob));
            marker.points.push_back(world_to_frame(p6, ob));
            marker.points.push_back(world_to_frame(p6, ob));
            marker.points.push_back(world_to_frame(p7, ob));
            marker.points.push_back(world_to_frame(p7, ob));
            marker.points.push_back(world_to_frame(p8, ob));
            marker.points.push_back(world_to_frame(p8, ob));
            marker.points.push_back(world_to_frame(p5, ob));

            publisher_GT.publish(marker);
            //std::cout << "publish GT"<< std::endl;
        }

        if(rviz_model)
        for(auto ob: obs_model) {

            id++;
            // color.
            std::vector<vector<float> > colors_bgr{ {135,0,248},  {255,0,253},  {4,254,119},  {255,126,1},  {0,112,255},  {0,250,250}   };
            vector<float> color = colors_bgr[ (ob->mnClass +4) % 6];  //+1还是粉色/红色    +2绿色  +3蓝色  +4橙色   +5黄色


            //publish rviz
            visualization_msgs::Marker marker;
            marker.id = id;//++object_id_init;//object_id_init + i;
            float mObject_Duration=1;
            // marker.lifetime = ros::Duration(mObject_Duration);
            marker.header.frame_id= "map";
            marker.header.stamp=ros::Time::now();
            marker.type = visualization_msgs::Marker::LINE_LIST; //LINE_STRIP;
            marker.action = visualization_msgs::Marker::ADD;
            marker.color.r = color[2]/255.0; marker.color.g = color[1]/255.0; marker.color.b = color[0]/255.0; marker.color.a = 1.0;
            marker.scale.x = 0.01;
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



            marker.points.push_back(world_to_frame(p1, ob));
            marker.points.push_back(world_to_frame(p2, ob));
            marker.points.push_back(world_to_frame(p2, ob));
            marker.points.push_back(world_to_frame(p3, ob));
            marker.points.push_back(world_to_frame(p3, ob));
            marker.points.push_back(world_to_frame(p4, ob));
            marker.points.push_back(world_to_frame(p4, ob));
            marker.points.push_back(world_to_frame(p1, ob));

            marker.points.push_back(world_to_frame(p5, ob));
            marker.points.push_back(world_to_frame(p1, ob));
            marker.points.push_back(world_to_frame(p6, ob));
            marker.points.push_back(world_to_frame(p2, ob));
            marker.points.push_back(world_to_frame(p7, ob));
            marker.points.push_back(world_to_frame(p3, ob));
            marker.points.push_back(world_to_frame(p8, ob));
            marker.points.push_back(world_to_frame(p4, ob));

            marker.points.push_back(world_to_frame(p5, ob));
            marker.points.push_back(world_to_frame(p6, ob));
            marker.points.push_back(world_to_frame(p6, ob));
            marker.points.push_back(world_to_frame(p7, ob));
            marker.points.push_back(world_to_frame(p7, ob));
            marker.points.push_back(world_to_frame(p8, ob));
            marker.points.push_back(world_to_frame(p8, ob));
            marker.points.push_back(world_to_frame(p5, ob));

            publisher_model.publish(marker);
            //std::cout << "publish Model"<< std::endl;
        }
    }
    ros::shutdown();
}
