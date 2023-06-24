#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
// #include <gazebo/gazebo_client.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/sensors/sensors.hh>
// #include <gazebo/math/gzmathc.hh>
// #include <gazebo/gui/gui.hh>
#include <ignition/math/Vector3.hh>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <iostream>
#include <fstream>
#include <Eigen/Core>
#include <algorithm>
#include "Object.h"
#include <cmath>

ros::Publisher publisher_object;
using namespace std;
using namespace ORB_SLAM2;
#include <iostream>
#include <Eigen/Dense>

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


double angle_tune = 0;
class object{

    public:
        object(){}
        object(std::string name_,
               double x_, double y_, double z_,
               double roll_, double pitch_, double yaw_,
               double w_, double l_, double d_ ):
                name(name_),
                x(x_), y(y_), z(z_),
                yaw(yaw_), pitch(pitch_), roll(roll_),
                width(w_), length(l_), depth(d_)
                {
                    //计算pose_mat
                    // 计算旋转矩阵
                    Eigen::Matrix3d rotation_matrix;
                    rotation_matrix = Eigen::AngleAxisd(yaw + angle_tune, Eigen::Vector3d::UnitZ());
                                    // * Eigen::AngleAxisd(ob.pitch, Eigen::Vector3d::UnitY())
                                    // * Eigen::AngleAxisd(ob.roll, Eigen::Vector3d::UnitX());

                    // 坐标系变换矩阵
                    Eigen::Matrix4d T;
                    T   <<  1, 0, 0, x,  // 假设T为平移矩阵，将A坐标系原点(1, 1, 1)平移到B坐标系原点
                            0, 1, 0, y,
                            0, 0, 1, z,
                            0, 0, 0, 1;
                    T.block<3,3>(0,0) = rotation_matrix;

                    pose_mat = Converter::toCvMat(T);
                }

        ~object(){}

    public:
        std::string name;
        int class_id;
        double x,y,z;
        double roll, pitch, yaw;
        double width,  length, depth;
        cv::Mat pose_mat;

    public:

};


geometry_msgs::Point corner_to_marker(geometry_msgs::Point& oldp, object& ob){
        geometry_msgs::Point newp;


        // 计算旋转矩阵
        Eigen::Matrix3d rotation_matrix;
        rotation_matrix = Eigen::AngleAxisd(ob.yaw + angle_tune, Eigen::Vector3d::UnitZ());
                        // * Eigen::AngleAxisd(ob.pitch, Eigen::Vector3d::UnitY())
                        // * Eigen::AngleAxisd(ob.roll, Eigen::Vector3d::UnitX());


        // 坐标系变换矩阵
        Eigen::Matrix4d T;
        T << 1, 0, 0, ob.x,  // 假设T为平移矩阵，将A坐标系原点(1, 1, 1)平移到B坐标系原点
            0, 1, 0, ob.y,
            0, 0, 1, ob.z,
            0, 0, 0, 1;
        T.block<3,3>(0,0) = rotation_matrix;


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
        // std::cout << "Point P in A coordinate system: (" << x_a << ", " << y_a << ", " << z_a << ")" << std::endl;
        // std::cout << "Point P in B coordinate system: (" << x_b << ", " << y_b << ", " << z_b << ")" << std::endl;

        newp.x = x_b ;
        newp.y = y_b ;
        newp.z = z_b ;
        return newp;
}

int main(int argc, char **argv)
{

    std::vector<object> obs;
    ros::init ( argc, argv, "gazebo_world_parser" );
    ros::NodeHandle nh;
    publisher_object = nh.advertise<visualization_msgs::Marker>("objectmap_groudtruth", 1000);
    //当前文件路径
    std::string current_file_path = __FILE__;
    std::string current_folder_path = current_file_path.substr(0, current_file_path.find_last_of("/\\"));
    WORK_SPACE_PATH = current_folder_path + "/" + "../";
    yamlfile_object = "kinectv1.yaml";

    if(argc!=2){
        std::cerr<<"没有world文件"<<std::endl;
    }

    //gazebo world文件
    string gazebo_file = argv[1];
    //string gazebo_file = "/home/zhjd/fabo_gazebo/src/fabo_moveit_gazebo/ASLAM_gazebo_world/world/nine_highdesk.world";
    //string gazebo_file = "/home/zhjd/workspace/ws_huchunxu/src/ros_exploring/my_mobilearm/my_gazebo/world/twodesk_wall.world";

    // Load gazebo
    gazebo::setupServer(argc, argv);

    // Create a world and get the models
    // gazebo::physics::WorldPtr world = gazebo::physics::get_world();
    gazebo::physics::WorldPtr world = gazebo::loadWorld(gazebo_file);//worlds/empty.world
    gazebo::physics::Model_V models = world->Models();


    // // Loop through each model and get its pose and size
    int h = 0 ;
    for (auto model : models) {
            h++;
            // Get model pose
            ignition::math::Pose3d pose = model->WorldPose();
            double x = pose.Pos().X();
            double y = pose.Pos().Y();
            double z = pose.Pos().Z();

            double roll = pose.Rot().Roll();
            double pitch = pose.Rot().Pitch();
            double yaw = pose.Rot().Yaw();

            // Get model size
            double width ;
            double length ;
            double height ;
            gazebo::physics::Link_V links = model->GetLinks();
            int i = 0;
            // for (auto link : links)
            auto link = links[0];  //因为据我观察，links的后续内容也一样，所以只需要读取第一个
            {
                i ++ ;
                gazebo::physics::Collision_V collisions = link->GetCollisions();
                int j = 0;
                //for (auto collision : collisions)   //据我观察，collisions只包含一个
                auto collision = collisions[0];
                // auto collision = collisions[0];
                {
                    j++;
                    ignition::math::Box box = collision->BoundingBox();
                    length = box.XLength();
                    width = box.YLength();
                    height = box.ZLength();
                    // lenth ：corner_2[0] - corner_1[0]
                    // width ：corner_2[1] - corner_3[1]
                    // height：corner_2[2] - corner_6[2]


                    // Print model information
                    // std::cout << "hij:"<< h << i << j << std::endl;
                    // std::cout << "Model num:"<< " ,name: " << model->GetName() << std::endl;
                    // std::cout << "Position: x=" << x << " y=" << y << " z=" << z << std::endl;
                    // std::cout << "Orientation: roll=" << roll << " ,pitch=" << pitch << " ,yaw=" << yaw << std::endl;
                    // std::cout << "Size: width=" << width << " ,heighth=" << height << " ,depth=" << depth << std::endl;
                }

            }
            //修正部分gazeobmodel
            if( model->GetName() == "book_2"){
                float temp = length;
                length = width;
                width = temp;
            }
            if( model->GetName() == "labtop_mac_3"){
                x = x + length/2.0;
                y = y - width/2.0;
            }
            if( model->GetName() == "vase_large_3"){
                //x = x + length/2.0;
                y = y - width/2.0;
            }
            if( model->GetName() == "cafe_table"){
                //x = x + length/2.0;
                length = 0.913;
                width = 0.913;
                height = 0.785;
            }
            if( model->GetName() == "mouse"){
                length = length/5.0*2.0;
            }
            //if( model->GetName() == "book_11"){
            //    float length_old = length;
            //    float width_old = width;
            //    float height_old = height;
            //    length = height_old;
            //    height = length_old;
            //    roll = 0.0;
            //    pitch = 0.0;
            //    yaw = 0.0;
            //}
            //if( model->GetName() == "book_15"){
            //    length = 0.245;
            //    width = 0.16;
            //}

            z = z+height/2.0;

            object ob(model->GetName(), x, y, z , roll, pitch, yaw, width, length, height);

            ob.class_id = 0;
            if ("bottle_red_wine" == ob.name  || "bottle_white_wine" == ob.name  ||"beer" == ob.name || "beer_0" == ob.name)  ob.class_id = 39;
            if ("cup_green" == ob.name || "cup_green_clone" == ob.name || "cup_blue" == ob.name || "trash_bin" == ob.name || "can_pepsi" == ob.name || "can_fanta" == ob.name)  ob.class_id = 41;
            if ("vase_violet" == ob.name || "vase_violet_clone" == ob.name ||"vase_large_3" == ob.name)  ob.class_id = 75;
            if ("desk_yellow" == ob.name || "desk_yellow_clone" == ob.name || "drawer_white" == ob.name || "cafe_table" == ob.name  || "desk_drawer" == ob.name )  ob.class_id = 60;
            if ("book_2" == ob.name || "book_16" == ob.name || "book_15" == ob.name || "book_16_clone" == ob.name  || "book_11" == ob.name)  ob.class_id = 73;
            if ("laptop_pc_1" == ob.name || "labtop_mac_3" == ob.name)  ob.class_id = 63;
            if ("keyboard" == ob.name)  ob.class_id = 66;
            if ("mouse" == ob.name)  ob.class_id = 64;
            if ("chair_2" == ob.name)  ob.class_id = 57;

            obs.push_back(ob);
    }


    ros::Rate rate(10);
    //while (nh.ok())


    int id = 0;
    //真值文件的路径
    std::string groud_truth_path = current_folder_path + "/" + "groudtruth.txt";

    cout << endl << "Saving Objects to " << groud_truth_path << " ..." << endl;

    ofstream file_gt;
    file_gt.open(groud_truth_path.c_str());
    file_gt << fixed;

    std::sort(obs.begin(), obs.end(), [](const object obj1, const object obj2) {
            return obj1.class_id < obj2.class_id;
        });
    for(auto ob: obs) {

        if( ob.name == "ground_plane" || ob.name == "desk_yellow" || ob.name == "desk_yellow_clone" || ob.name == "desk_white" || ob.name == "desk_white_clone" || ob.name == "wall" || ob.name == "grey_wall" || ob.name == "wall6x45_4window_cloor"
            || ob.name == "ball_bearing"     ){
            std::cout << "invalid"<< std::endl;
            continue;
        }
        //if(  ob.name != "desk_yellow" && ob.name != "desk_yellow_clone"){
        //    std::cout << "invalid"<< std::endl;
        //    continue;
        //}
        id++;
        // Print model information
        std::cout <<  std::endl;
        std::cout << "Model num:" << ob.name << std::endl;
        std::cout << "Position: x=" << ob.x << " ,y=" << ob.y << " ,z=" << ob.z << std::endl;
        std::cout << "Orientation: roll=" << ob.roll << " ,pitch=" << ob.pitch << " ,yaw=" << ob.yaw << std::endl;
        std::cout << "Size: width=" << ob.width << " ,length=" << ob.length << " ,depth=" << ob.depth << std::endl;

        g2o::SE3Quat pose ;//= object->mCuboid3D.pose_mat;
        pose = Converter::cvMattoG2oSE3Quat(ob.pose_mat.clone());
        file_gt     << "1 "  //物体
                    << id << "   "

                    << ob.class_id << " "
                    << "1" << " "
                    << "0" << "     "

                    //位移
                    << pose.translation().x() << " "
                    << pose.translation().y() << " "
                    << pose.translation().z() << "     "

                    //方向
                    << pose.rotation().x() << " "
                    << pose.rotation().y() << " "
                    << pose.rotation().z() << " "
                    << pose.rotation().w() << "     "

                    //尺寸
                    << ob.length << " "
                    << ob.width << " "
                    << ob.depth << " "
                    << "#" <<yolo_id[ob.class_id]<< "  "
                    << "%" <<ob.name
                    << endl;
    }

    //单独处理桌子
    //double  desk_x_min=100, desk_x_max=-100,
    //        desk_y_min=100, desk_y_max=-100,
    //        desk_z_min=100, desk_z_max=-100;
    //for(auto ob: obs) {
    //    if( ob.name != "desk_yellow" && ob.name != "desk_yellow_clone")
    //        continue;
    //
    //    if()
    //}


    file_gt.close();
    cout << endl << "Saved Objects End " << endl;



    // Clean up
    gazebo::shutdown();
    return 0;

}
