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
ros::Publisher publisher_object;


#include <iostream>
#include <Eigen/Dense>

// geometry_msgs::Point corner_to_marker(geometry_msgs::Point& oldp, ignition::math::Pose3d& pose){
//         geometry_msgs::Point newp;
//         newp.x = oldp.x + pose.Pos().X() ;
//         newp.y = oldp.y + pose.Pos().Y() ;
//         newp.z = oldp.z + pose.Pos().Z() ;
//         return newp;
// }

double angle_tune = 0;
class object{
    
    public:
        object(){}
        object( std::string name_,
                double x_,  double y_, double z_, 
                double roll_, double pitch_, double yaw_, 
                double w_, double h_, double d_ ):
                name(name_),
                x(x_),y(y_),z(z_),
                yaw(yaw_), pitch(pitch_), roll(roll_),
                width(w_), height(h_),depth(d_)  {     }
        ~object(){}
    public:
        std::string name;
        double x,y,z;
        double roll, pitch, yaw;
        double width,  height, depth;
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
{   std::string worldfile;
    if(argc != 2){
        std::cout<<"【注意】：没有输入被读取的world文件地址"<<std::endl;    
        std::cout<<"读取默认文件：/home/zhjd/fabo_gazebo/src/fabo_moveit_gazebo/ASLAM_gazebo_world/world/nine_highdesk.world"<<std::endl;    
        worldfile = "/home/zhjd/fabo_gazebo/src/fabo_moveit_gazebo/ASLAM_gazebo_world/world/nine_highdesk.world";    
    }
    else
        worldfile = argv[1];
        
    std::vector<object> obs;
    ros::init ( argc, argv, "gazebo_world_parser" );
    ros::NodeHandle nh;
    publisher_object = nh.advertise<visualization_msgs::Marker>("objectmap_groudtruth", 1000);
    
    // Load gazebo
    gazebo::setupServer(argc, argv);

    // Create a world and get the models
    // gazebo::physics::WorldPtr world = gazebo::physics::get_world();
    gazebo::physics::WorldPtr world = gazebo::loadWorld(worldfile);//worlds/empty.world
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
            double height ;
            double depth ;
            gazebo::physics::Link_V links = model->GetLinks();
            int i = 0;
            // for (auto link : links)   
            auto link = links[0];  //因为据我观察，links的后续内容也一样，所以只需要读取第一个
            {   
                i ++ ;
                gazebo::physics::Collision_V collisions = link->GetCollisions();
                int j = 0;
                for (auto collision : collisions)   //据我观察，collisions只包含一个
                // auto collision = collisions[0];   
                {
                    j++;
                    ignition::math::Box box = collision->BoundingBox();
                    if( model->GetName() == "book_2"){
                        height = box.YLength();
                        width = box.XLength();
                        depth = box.ZLength();    

                    }
                    else{
                        height = box.XLength();
                        width = box.YLength();
                        depth = box.ZLength();
                    }
                    


                    // Print model information
                    // std::cout << "hij:"<< h << i << j << std::endl;
                    // std::cout << "Model num:"<< " ,name: " << model->GetName() << std::endl;
                    // std::cout << "Position: x=" << x << " y=" << y << " z=" << z << std::endl;
                    // std::cout << "Orientation: roll=" << roll << " ,pitch=" << pitch << " ,yaw=" << yaw << std::endl;
                    // std::cout << "Size: width=" << width << " ,heighth=" << height << " ,depth=" << depth << std::endl;
                }
                    
            }

            object ob(model->GetName(), x,y,z,  roll,pitch,yaw,  width,height,depth);
            obs.push_back(ob);  
    }


    ros::Rate rate(10);
    while (nh.ok()){    
        
        int id = 0;

        for(auto ob: obs) {
            if( ob.name == "ground_plane"){
                std::cout << "ground"<< std::endl;
                continue;
            }
            
            id++;
            // Print model information
            std::cout <<  std::endl;
            std::cout << "not ground"<< std::endl;
            std::cout << "Model num:" << ob.name << std::endl;
            std::cout << "Position: x=" << ob.x << " ,y=" << ob.y << " ,z=" << ob.z << std::endl;
            std::cout << "Orientation: roll=" << ob.roll << " ,pitch=" << ob.pitch << " ,yaw=" << ob.yaw << std::endl;
            std::cout << "Size: width=" << ob.width << " ,heighth=" << ob.height << " ,depth=" << ob.depth << std::endl;
            
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
            double width_half = ob.width/2.0;
            double height_half = ob.height/2.0;
            double depth_half = ob.depth/2.0;

            geometry_msgs::Point p1;   p1.x = -1*height_half;   p1.y = width_half;      p1.z = 0; //-1*depth_half; 
            geometry_msgs::Point p2;   p2.x = -1*height_half;   p2.y = -1*width_half;   p2.z = 0; //-1*depth_half; 
            geometry_msgs::Point p3;   p3.x = height_half;      p3.y = -1*width_half;   p3.z = 0; //-1*depth_half; 
            geometry_msgs::Point p4;   p4.x = height_half;      p4.y = width_half;      p4.z = 0; //-1*depth_half; 
            geometry_msgs::Point p5;   p5.x = -1*height_half;   p5.y = width_half;      p5.z = 2*depth_half; 
            geometry_msgs::Point p6;   p6.x = -1*height_half;   p6.y = -1*width_half;   p6.z = 2*depth_half; 
            geometry_msgs::Point p7;   p7.x = height_half;      p7.y = -1*width_half;   p7.z = 2*depth_half; 
            geometry_msgs::Point p8;   p8.x = height_half;      p8.y = width_half;      p8.z = 2*depth_half; 
            
            
            
            marker.points.push_back(corner_to_marker(p1, ob));
            marker.points.push_back(corner_to_marker(p2, ob));
            marker.points.push_back(corner_to_marker(p2, ob));
            marker.points.push_back(corner_to_marker(p3, ob));
            marker.points.push_back(corner_to_marker(p3, ob));
            marker.points.push_back(corner_to_marker(p4, ob));
            marker.points.push_back(corner_to_marker(p4, ob));
            marker.points.push_back(corner_to_marker(p1, ob));

            marker.points.push_back(corner_to_marker(p5, ob));
            marker.points.push_back(corner_to_marker(p1, ob));
            marker.points.push_back(corner_to_marker(p6, ob));
            marker.points.push_back(corner_to_marker(p2, ob));
            marker.points.push_back(corner_to_marker(p7, ob));
            marker.points.push_back(corner_to_marker(p3, ob));
            marker.points.push_back(corner_to_marker(p8, ob));
            marker.points.push_back(corner_to_marker(p4, ob));

            marker.points.push_back(corner_to_marker(p5, ob));
            marker.points.push_back(corner_to_marker(p6, ob));
            marker.points.push_back(corner_to_marker(p6, ob));
            marker.points.push_back(corner_to_marker(p7, ob));
            marker.points.push_back(corner_to_marker(p7, ob));
            marker.points.push_back(corner_to_marker(p8, ob));
            marker.points.push_back(corner_to_marker(p8, ob));
            marker.points.push_back(corner_to_marker(p5, ob));

            publisher_object.publish(marker);
            std::cout << "publish rviz"<< std::endl;

            // publish rviz end
            // rate.sleep();

        }
        
    }  












    // int id = 0;

    // while (nh.ok()){    
        

    //         //publish rviz 
    //         visualization_msgs::Marker marker;
    //         marker.id = id++;//++object_id_init;//object_id_init + i;
    //         float mObject_Duration=1;
    //         // marker.lifetime = ros::Duration(mObject_Duration);
    //         marker.header.frame_id= "map";
    //         marker.header.stamp=ros::Time::now();
    //         marker.type = visualization_msgs::Marker::LINE_LIST; //LINE_STRIP;
    //         marker.action = visualization_msgs::Marker::ADD;
    //         marker.color.r = 255.0; marker.color.g = 0.0; marker.color.b = 0.0; marker.color.a = 1.0;
    //         marker.scale.x = 0.01;
    //         //     8------7
    //         //    /|     /|
    //         //   / |    / |
    //         //  5------6  |
    //         //  |  4---|--3
    //         //  | /    | /
    //         //  1------2
    //         // lenth ：corner_2[0] - corner_1[0]
    //         // width ：corner_2[1] - corner_3[1]
    //         // height：corner_2[2] - corner_6[2]
    //         double width_half = width/2.0;
    //         double height_half = height/2.0;
    //         double depth_half = depth/2.0;

    //         geometry_msgs::Point p1;   p1.x = -1*height_half;   p1.y = width_half;      p1.z = -1*depth_half; 
    //         geometry_msgs::Point p2;   p2.x = -1*height_half;   p2.y = -1*width_half;   p2.z = -1*depth_half; 
    //         geometry_msgs::Point p3;   p3.x = height_half;      p3.y = -1*width_half;   p3.z = -1*depth_half; 
    //         geometry_msgs::Point p4;   p4.x = height_half;      p4.y = width_half;      p4.z = -1*depth_half; 
    //         geometry_msgs::Point p5;   p5.x = -1*height_half;   p5.y = width_half;      p5.z = 1*depth_half; 
    //         geometry_msgs::Point p6;   p6.x = -1*height_half;   p6.y = -1*width_half;   p6.z = 1*depth_half; 
    //         geometry_msgs::Point p7;   p7.x = height_half;      p7.y = -1*width_half;   p7.z = 1*depth_half; 
    //         geometry_msgs::Point p8;   p8.x = height_half;      p8.y = width_half;      p8.z = 1*depth_half; 
            
            
            
    //         marker.points.push_back(corner_to_marker(p1, pose));
    //         marker.points.push_back(corner_to_marker(p2, pose));
    //         marker.points.push_back(corner_to_marker(p2, pose));
    //         marker.points.push_back(corner_to_marker(p3, pose));
    //         marker.points.push_back(corner_to_marker(p3, pose));
    //         marker.points.push_back(corner_to_marker(p4, pose));
    //         marker.points.push_back(corner_to_marker(p4, pose));
    //         marker.points.push_back(corner_to_marker(p1, pose));

    //         marker.points.push_back(corner_to_marker(p5, pose));
    //         marker.points.push_back(corner_to_marker(p1, pose));
    //         marker.points.push_back(corner_to_marker(p6, pose));
    //         marker.points.push_back(corner_to_marker(p2, pose));
    //         marker.points.push_back(corner_to_marker(p7, pose));
    //         marker.points.push_back(corner_to_marker(p3, pose));
    //         marker.points.push_back(corner_to_marker(p8, pose));
    //         marker.points.push_back(corner_to_marker(p4, pose));

    //         marker.points.push_back(corner_to_marker(p5, pose));
    //         marker.points.push_back(corner_to_marker(p6, pose));
    //         marker.points.push_back(corner_to_marker(p6, pose));
    //         marker.points.push_back(corner_to_marker(p7, pose));
    //         marker.points.push_back(corner_to_marker(p7, pose));
    //         marker.points.push_back(corner_to_marker(p8, pose));
    //         marker.points.push_back(corner_to_marker(p8, pose));
    //         marker.points.push_back(corner_to_marker(p5, pose));

    //         publisher_object.publish(marker);
    //         // publish rviz end
    //         rate.sleep();
    // }  
    

    // Clean up
    gazebo::shutdown();
    return 0;

}
