//
// Created by zhjd on 5/14/23.
//

#include "read_local_objects.h"

namespace ORB_SLAM2 {

    void cmpute_corner(Object_Map* object) {

            float x_min_obj = (-0.5)*object->mCuboid3D.lenth;
            float x_max_obj = (0.5)*object->mCuboid3D.lenth;
            float y_min_obj = (-0.5)*object->mCuboid3D.width;
            float y_max_obj = (0.5)*object->mCuboid3D.width;
            float z_min_obj = (-0.5)*object->mCuboid3D.height;
            float z_max_obj = (0.5)*object->mCuboid3D.height;

            g2o::SE3Quat pose =  Converter::toSE3Quat( object->mCuboid3D.pose_mat);
            object->mCuboid3D.corner_1 = pose * Eigen::Vector3d(x_min_obj, y_min_obj, z_min_obj) ;
            object->mCuboid3D.corner_2 = pose * Eigen::Vector3d(x_max_obj, y_min_obj, z_min_obj) ;
            object->mCuboid3D.corner_3 = pose * Eigen::Vector3d(x_max_obj, y_max_obj, z_min_obj) ;
            object->mCuboid3D.corner_4 = pose * Eigen::Vector3d(x_min_obj, y_max_obj, z_min_obj) ;
            object->mCuboid3D.corner_5 = pose * Eigen::Vector3d(x_min_obj, y_min_obj, z_max_obj) ;
            object->mCuboid3D.corner_6 = pose * Eigen::Vector3d(x_max_obj, y_min_obj, z_max_obj) ;
            object->mCuboid3D.corner_7 = pose * Eigen::Vector3d(x_max_obj, y_max_obj, z_max_obj) ;
            object->mCuboid3D.corner_8 = pose * Eigen::Vector3d(x_min_obj, y_max_obj, z_max_obj) ;
    }

    void ReadLocalObjects( const std::string& filePath, vector<Object_Map*>& vObjects){

        ifstream infile(filePath, ios::in);
        if (!infile.is_open())
        {
            cout << "open fail: "<< filePath <<" " << endl;
            exit(233);
        }
        else
        {
            std::cout << "read Objects_with_points.txt" << std::endl;
        }

        vector<double> row;

        cv::Mat cam_pose_mat;
        int mnid_current = -1;
        //string s0;
        //getline(infile, s0);  注销掉无用的line
        vObjects.clear();
        string line;
        int object_num = -1;
        int type = 1;
        while (getline(infile, line))
        {   //std::cout<<line<<std::endl;
            istringstream istr(line);
            istr >> type;

            if( type == 1){
                Object_Map *obj = new Object_Map();
                object_num ++;
                //std::cout<<"物体"<<object_num<<std::endl;
                double temp;
                istr >> temp;    obj->mnId = temp;
                istr >> temp;    obj->mnClass = temp;
                istr >> temp;    obj->mnConfidence_foractive = temp;
                istr >> temp ;  //物体中特征点的数量

                Eigen::MatrixXd object_poses(1, 8); ;
                istr >> temp;  object_poses(0) = temp;  //obj->mCuboid3D.cuboidCenter0 = temp;
                istr >> temp;  object_poses(1) = temp;  //obj->mCuboid3D.cuboidCenter1 = temp;
                istr >> temp;  object_poses(2) = temp;  //obj->mCuboid3D.cuboidCenter2 = temp;
                istr >> temp;  object_poses(3) = temp;
                istr >> temp;  object_poses(4) = temp;
                istr >> temp;  object_poses(5) = temp;
                istr >> temp;  object_poses(6) = temp;
                g2o::SE3Quat cam_pose_se3(object_poses.row(0).head(7));

                obj->mCuboid3D.pose_mat = Converter::toCvMat(cam_pose_se3);
                istr >> temp;   obj->mCuboid3D.lenth = temp;
                istr >> temp;   obj->mCuboid3D.width = temp;
                istr >> temp;   obj->mCuboid3D.height = temp;

                cmpute_corner(obj);

                vObjects.push_back( obj );

                std::cout<<  "mnId: "<<vObjects[ object_num ]->mnId
                        <<  ", Class: " << vObjects[ object_num ]->mnClass <<std::endl;

            }
            else if( type == 0)
            {
                //std::cout<<"特征点"<<object_num<<std::endl;
                double temp;
                istr >> temp;
                istr >> temp;

                MapPoint* point = new MapPoint();
                float x_p, y_p, z_p;
                istr >> temp;  x_p = temp;
                istr >> temp;  y_p = temp;
                istr >> temp;  z_p = temp;
                std::vector<float> vec{x_p, y_p, z_p};
                cv::Mat WorldPos(vec);

                point->SetWorldPos(WorldPos) ;
                // 设置随机数种子,生成 1 到 3之间的随机数
                std::srand(std::time(0));
                int random_num = std::rand() % 3 + 1;
                point->viewdCount_forObjectId.insert(make_pair(vObjects[ object_num ]->mnId, random_num));
                vObjects[ object_num ]-> mvpMapObjectMappoints.push_back( point );
                //mpMapPub -> mpMap->mvObjectMap[ object_num ]->mvpMapObjectMappoints.push_back( &point );
            }


            row.clear();
            type = -1;
            istr.clear();
            line.clear();
        }

        for(int i=0; i<vObjects.size(); i++){
            vObjects[i]->ComputeIE();
        }
    }

}