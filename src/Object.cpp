//
// Created by zhjd on 11/8/22.
//

#include "Object.h"
#include "Converter.h"

namespace ORB_SLAM2
{

mutex Object_2D::mGlobalMutex;  //crash bug
mutex Object_Map::mMutex_front_back;

bool debug_iou_view = 0;
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

        object->mCuboid3D.x_max = std::max( std::max(object->mCuboid3D.corner_1[0],  object->mCuboid3D.corner_2[0]),
                                            std::max(object->mCuboid3D.corner_3[0],  object->mCuboid3D.corner_4[0]));
        object->mCuboid3D.x_min = std::min( std::min(object->mCuboid3D.corner_1[0],  object->mCuboid3D.corner_2[0]),
                                            std::min(object->mCuboid3D.corner_3[0],  object->mCuboid3D.corner_4[0]));
        object->mCuboid3D.y_max = std::max( std::max(object->mCuboid3D.corner_1[1],  object->mCuboid3D.corner_2[1]),
                                            std::max(object->mCuboid3D.corner_3[1],  object->mCuboid3D.corner_4[1]));
        object->mCuboid3D.y_min = std::min( std::min(object->mCuboid3D.corner_1[1],  object->mCuboid3D.corner_2[1]),
                                            std::min(object->mCuboid3D.corner_3[1],  object->mCuboid3D.corner_4[1]));
        object->mCuboid3D.z_max = object->mCuboid3D.corner_5[2];
        object->mCuboid3D.z_min = object->mCuboid3D.corner_1[2];
}

void ReadLocalObjects( const std::string& filePath, std::vector<Object_Map*>& vObjects){

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

            std::cout<<  "[读取本地物体] Id: "<< vObjects[object_num]->mnId <<  ", Class: " << yolo_id[ vObjects[object_num]->mnClass ] <<std::endl;

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

Object_2D::Object_2D() {
    //std::cout<<"Object_2D  construct 1   ";
    //std::cout<<">>>   End"<<std::endl;
}

Object_2D::Object_2D(Map* Map, Frame* CurrentFrame, const BoxSE &box) {
    //std::cout<<"Object_2D  construct 3   ";
    mclass_id = box.m_class;
    mScore = box.m_score;
    mleft = box.x;
    mright = box.x + box.width;
    mtop = box.y;
    mbottom = box.y + box.height;
    mWidth = box.width;
    mHeight = box.height;
    // 2d center.

    mBoxCenter_2d = cv::Point2f(box.x + box.width / 2, box.y + box.height / 2);
    // opencv Rect format.
    mBox_cvRect = cv::Rect(box.x, box.y, box.width, box.height);

    this->mpCurrentFrame = CurrentFrame;
    this->mpMap = Map;

    //初始化位姿
    this->sum_pos_3d = cv::Mat::zeros(3, 1, CV_32F);
    //std::cout<<">>>   End"<<std::endl;
}

void Object_2D::AddYoloBoxes(const BoxSE &box) {
    //废弃
}

void Object_2D::ComputeMeanAndDeviation()
{
    unique_lock<mutex> lock(mMutexPos);
    unique_lock<mutex> lock2(mMutexObjMapPoints);  //一帧中会添加新的物体点   还是只会被剔除吗？？？
    // remove bad points.
    vector<MapPoint *>::iterator pMP;
    sum_pos_3d = cv::Mat::zeros(3,1,CV_32F);
    for (pMP = mvMapPonits.begin();
         pMP != mvMapPonits.end();)
    {
        cv::Mat pos = (*pMP)->GetWorldPos();
        if ((*pMP)->isBad())
        {
            //sum_pos_3d -= pos;
            pMP = mvMapPonits.erase(pMP);
        }
        else{
            sum_pos_3d += pos;
            ++pMP;
        }
    }

    // 计算平均值
    mPos_world = sum_pos_3d / (mvMapPonits.size());

    //// 标准偏差
    //float sum_x2 = 0, sum_y2 = 0, sum_z2 = 0;
    //size_t i = 0;
    //for (; i < mvMapPonits.size(); i++)
    //{
    //    MapPoint *pMP = mvMapPonits[i];
    //    if (pMP->isBad())
    //        continue;
    //
    //    cv::Mat pos = pMP->GetWorldPos();
    //    cv::Mat pos_ave = mPos_world;
    //
    //    sum_x2 += (pos.at<float>(0)-pos_ave.at<float>(0))     *   (pos.at<float>(0)-pos_ave.at<float>(0));
    //    sum_y2 += (pos.at<float>(1)-pos_ave.at<float>(1))     *   (pos.at<float>(1)-pos_ave.at<float>(1));
    //    sum_z2 += (pos.at<float>(2)-pos_ave.at<float>(2))     *   (pos.at<float>(2)-pos_ave.at<float>(2));
    //}
    //mStandar_x = sqrt(sum_x2 / i);
    //mStandar_y = sqrt(sum_y2 / i);
    //mStandar_z = sqrt(sum_z2 / i);
}

void Object_2D::RemoveOutlier_ByHeightorDepth()
{
    unique_lock<mutex> lock(mMutexObjMapPoints);
    const cv::Mat Rcw = cv::Mat::zeros(3,3,CV_32F);
    const cv::Mat tcw = cv::Mat::eye(3,1,CV_32F);  //深拷贝
    mpCurrentFrame->mTcw.rowRange(0, 3).colRange(0, 3).copyTo(Rcw);
    mpCurrentFrame->mTcw.rowRange(0, 3).col(3).copyTo(tcw);

    // world -> camera.
    vector<float> x_c;
    vector<float> y_c;
    vector<float> z_c;
    for (size_t i = 0; i < mvMapPonits.size(); i++)
    {
        MapPoint *pMP = mvMapPonits[i];

        cv::Mat PointPosWorld = pMP->GetWorldPos();
        cv::Mat PointPosCamera = Rcw * PointPosWorld + tcw;

        x_c.push_back(PointPosCamera.at<float>(0));
        y_c.push_back(PointPosCamera.at<float>(1));
        z_c.push_back(PointPosCamera.at<float>(2));
    }

    // sort.
    sort(x_c.begin(), x_c.end());
    sort(y_c.begin(), y_c.end());
    sort(z_c.begin(), z_c.end());
    // notes: 点的数量需要大于4
    if ((z_c.size() / 4 <= 0) || (z_c.size() * 3 / 4 >= z_c.size() - 1))
        return;
    // notes: 取排序1/4和3/4处的深度并对其进行扩展，获取最大最小阈值
    float Q1 = z_c[(int)(z_c.size() / 4)];
    float Q3 = z_c[(int)(z_c.size() * 3 / 4)];
    float IQR = Q3 - Q1;
    float min_th = Q1 - 1.5 * IQR; // no use
    float max_th = Q3 + 1.5 * IQR;

    vector<MapPoint *>::iterator pMP;
    for (pMP = mvMapPonits.begin();
         pMP != mvMapPonits.end();)
    {
        cv::Mat PointPosWorld = (*pMP)->GetWorldPos();
        cv::Mat PointPosCamera = Rcw * PointPosWorld + tcw;
        float z = PointPosCamera.at<float>(2);
        // notes: 排除过远处的3D点
        if (z > max_th){
            pMP = mvMapPonits.erase(pMP);
        }
        else
            ++pMP;
    }
    // 根据height，更新完mapPoint后，重新计算均值和偏差
    //this->ComputeMeanAndDeviation();
}

void Object_2D::MergeTwo_Obj2D(Object_2D *Old_Object2D)
{
    unique_lock<mutex> lock(mMutexObjMapPoints);
    for (size_t m = 0; m < Old_Object2D->mvMapPonits.size(); ++m)
    {
        bool bNewPoint = true;

        MapPoint *pMPLast = Old_Object2D->mvMapPonits[m];
        cv::Mat PosLast = pMPLast->GetWorldPos();

        // whether a new points.
        for (size_t n = 0; n < this->mvMapPonits.size(); ++n)
        {
            MapPoint *pMPCurr = this->mvMapPonits[n];
            cv::Mat PosCurr = pMPCurr->GetWorldPos();

            if (cv::countNonZero(PosLast - PosCurr) == 0)
            {
                bNewPoint = false;   //如果两个point坐标， 说明是同一个point， 则认为不是NewPoint
                break;
            }
        }

        if (bNewPoint)
        {
            this->mvMapPonits.push_back(pMPLast);  // 将NewPoint 加入当前Object2D的point集合中
        }
    }
    // 根据物体融合，更新完mapPoint后，重新计算均值和偏差
    //this->ComputeMeanAndDeviation();
}

int Object_2D::Object2D_DataAssociationWith_Object3D()  //cv::Mat &image
{
    //std::cout<<"查看是否关联到旧物体"<<std::endl;
    const cv::Mat image = mpCurrentFrame->mColorImage.clone();
    const cv::Mat Rcw = cv::Mat::zeros(3,3,CV_32F);
    const cv::Mat tcw = cv::Mat::eye(3,1,CV_32F);  //深拷贝
    mpCurrentFrame->mTcw.rowRange(0, 3).colRange(0, 3).copyTo(Rcw);
    mpCurrentFrame->mTcw.rowRange(0, 3).col(3).copyTo(tcw);

    cv::Rect RectCurrent = mBox_cvRect;     // object bounding box in current frame.
    cv::Rect RectPredict;                   // predicted bounding box according to last frame and next to last frame.
    cv::Rect RectProject;                   // bounding box constructed by projecting points.

    const vector<Object_Map*> ObjectMaps  = mpMap->GetObjects();
    // ****************************************************
    //         STEP 1. Motion  IoU  association.              *
    // ****************************************************
    float IouMax = 0;                           //
    bool bAssoByMotionIou = false;              // 默认禁用. 通过后续IOU匹配的结果,
    int AssoObjId_byIou = -1;                   // the associated map object ID.
    int ObjID_IouMax = -1;                      // temporary variable. 代表map中与this->object_2d, 最IOU匹配的object_map
    float IouThreshold = 0.5;                   // IoU threshold.
    if(MotionIou_flag)//if((flag != "NA") && (flag != "NP"))
    {

        for (int i = 0; i < (int)ObjectMaps.size(); i++)
        {
            Object_Map* obj3d = ObjectMaps[i];
            if (mclass_id != obj3d->mnClass)
                continue;
            if (obj3d->bad_3d||obj3d->backgroud_object)
                continue;
            if ((mpCurrentFrame->mnId-1) == obj3d->mnLastAddID )   //如果是临近的两帧， 则可以用IOU关联。
            {
                // step 1.1 predict object bounding box according to last frame and next to last frame.
                if ((mpCurrentFrame->mnId-2) == obj3d->mnLastLastAddID)
                {   //以下基于的假设: 临近三帧,相同物体的检测框,在图像上的移动是相同的
                    // 0____ll___l____c
                    // c = l - ll + l = 2 * l - ll
                    // left-top.
                    float left_top_x = obj3d->mLastRect.x * 2 - obj3d->mLastLastRect.x;     // cv::Rect的x代表 方形的左上角的x坐标
                    if (left_top_x < 0)
                        left_top_x = 0;
                    float left_top_y = obj3d->mLastRect.y * 2 - obj3d->mLastLastRect.y;     // cv::Rect的y代表 方形的左上角的y坐标
                    if (left_top_y < 0)
                        left_top_y = 0;
                    // right-bottom.
                    float right_down_x = (obj3d->mLastRect.x + obj3d->mLastRect.width) * 2 - (obj3d->mLastLastRect.x + obj3d->mLastLastRect.width);
                    if (left_top_x > image.cols)
                        right_down_x = image.cols;
                    float right_down_y = (obj3d->mLastRect.y + obj3d->mLastRect.height) * 2 - (obj3d->mLastLastRect.y + obj3d->mLastLastRect.height);
                    if (left_top_y > image.rows)
                        right_down_y = image.rows;

                    float width = right_down_x - left_top_x;
                    float height = right_down_y - left_top_y;

                    // predicted bounding box.
                    RectPredict = cv::Rect(left_top_x, left_top_y, width, height);

                    // If two consecutive frames are observed, increase the threshold.
                    IouThreshold = 0.6;
                }
                else
                    RectPredict = obj3d->mLastRect;    //如果不是临近三帧, 则认为物体检测框和上一帧相同

                // step 1.2 compute IoU, record the max IoU and the map object ID.
                float Iou = Converter::bboxOverlapratio(RectCurrent, RectPredict);

                //可视化 for debug
                if(debug_iou_view){
                    cv::Mat mat_test = mpCurrentFrame->mColorImage.clone();
                    cv::Scalar color = (200, 0, 0);
                    cv::rectangle(mat_test, RectCurrent, (0, 0, 255), 2);
                    cv::rectangle(mat_test, RectPredict, (255, 0, 0), 2);
                    cv::putText(mat_test, std::to_string(Iou), cv::Point(0, 500), cv::FONT_HERSHEY_DUPLEX, 1.0,
                                (0, 255, 0), 2);
                    cv::resize(mat_test, mat_test, cv::Size(640 * 0.5, 480 * 0.5), 0, 0, cv::INTER_CUBIC);
                    //cv::imshow("[MotionIou]", mat_test);
                }
                //std::cout<<"[MotionIou] iou:"<<Iou <<std::endl;
                if ((Iou > IouThreshold) && (Iou > IouMax))
                {
                    IouMax = Iou;
                    ObjID_IouMax = i;
                    //std::cout<<"[MotionIou] yes "<<std::endl;
                }

            }
        }// 对map中的object遍历完毕,找到IOU匹配最佳的物体"IouMaxObjID"

        // step 1.3 if the association is successful, update the map object.
        Object_Map* obj3d_IouMax = ObjectMaps[ObjID_IouMax];
        if ((IouMax > 0) && (ObjID_IouMax >= 0))
        {
            // 更新当前object2d中的特征点，到匹配上的object3d中
            bool bFlag = obj3d_IouMax->UpdateToObject3D(this, *mpCurrentFrame, MotionIou);
            if (bFlag)
            {
                bAssoByMotionIou = true;              // associated by IoU.
                AssoObjId_byIou = ObjID_IouMax;     // associated map object id.
            }
        }
    }
    // Iou data association END ----------------------------------------------------------------------------

    // *************************************************
    //      STEP 2. Nonparametric data association     *
    // *************************************************
    bool bAssoByNp = false;
    int  AssoObjId_byNP  = -1; //;nAssoByNPId
    vector<int> vAssoObjIds_byNP;     // potential associated objects.
    if(NoPara_flag)//if((flag != "NA") && (flag != "IoU"))
    {
        // 遍历vObjectMap中每一个物体实例，与自身的2D目标框做NP
        for (int i = (int)ObjectMaps.size() - 1; (i >= 0) ; i--)
        {
            Object_Map* obj3d = ObjectMaps[i];
            if (mclass_id != obj3d->mnClass)
                continue;
            if (obj3d->bad_3d||obj3d->backgroud_object)
                continue;

            // step 2.1 nonparametric test.
            int NoParaFlag = this->NoParaDataAssociation(obj3d);  //TODO: NoParaDataAssociation是怎么计算和obj3d的关联关系的??

            if (NoParaFlag == 0) // 0: skip the nonparametric test and continue with the subsequent t-test.
                break;
            if (NoParaFlag == 2) // 2: association failed, compare next object.
                continue;
            else if (NoParaFlag == 1) // 1: association succeeded, but there may be more than one.
                vAssoObjIds_byNP.push_back(i);
        }// 对map中的object遍历完毕,找到NoPara匹配合适的所有物体"vObjByNPId"

        // step 2.2 update data association and record potential associated objects.
        if (vAssoObjIds_byNP.size() >= 1)
        {
            // case 1: if associated by IoU, the objects judged by nonparametric-test are marked as potential association objects.
            if (bAssoByMotionIou)
            {
                Object_Map* obj3d_IOU = ObjectMaps[AssoObjId_byIou];
                // 遍历NP的匹配，是否与IOU判断一致
                for (int i = 0; i < vAssoObjIds_byNP.size(); i++)
                {
                    AssoObjId_byNP = vAssoObjIds_byNP[i];

                    //如果NP和IOU找到的物体相同, 则不作处理. 因为已经在前面的iou中处理过一次了
                    if (AssoObjId_byNP == AssoObjId_byIou)
                        continue;

                    //如果不同, 则在mReObj中, 记录NP潜在的链接关系. 之后可用于 TODO:其他部分。                                   Record potential association objects and potential association times.
                    AddPotentialAssociatedObjects(ObjectMaps, AssoObjId_byIou  ,AssoObjId_byNP);
                }
            }

            // case 2: if association failed by IoU,
            // notes: 如果没有IOU重合，则把NP的结果代入DataAssociateUpdate进行检查
            // 疑问：多对多怎么处理？即：vObjByNPId  vAssoObjIds_byNP 中有多个返回bFlag==true。不过好像没有影响
            // 答: 似乎只会处理第一个成功的, 就break出去了.
            else
            {
                for (int i = 0; i < vAssoObjIds_byNP.size(); i++)
                {
                    //  更新当前object2d中的特征点，到匹配上的object3d中
                    AssoObjId_byNP = vAssoObjIds_byNP[i];
                    bool bFlag = ObjectMaps[AssoObjId_byNP]->UpdateToObject3D(this, *mpCurrentFrame, NoPara);

                    // if association successful, other objects are marked as potential association objects.
                    if (bFlag)
                    {
                        bAssoByNp = true;               // associated by NP.
                        AssoObjId_byNP = AssoObjId_byNP;    // associated map object id.

                        if (vAssoObjIds_byNP.size() > i + 1)  //TODO: 这个判断没有意义. 因此如果不是最后一个, 那么下面必然要执行. 如果是最后一个,那么也不用循环了,之后的break也没有意义
                        {
                            for (int j = i + 1; j < vAssoObjIds_byNP.size(); j++)
                            {
                                int AssoObjId_byNP_2 = vAssoObjIds_byNP[j];
                                // Record potential association objects and potential association times.
                                AddPotentialAssociatedObjects(ObjectMaps, AssoObjId_byNP, AssoObjId_byNP_2);
                            }
                            break;
                        }
                    }
                }
            }
        }
    }
    // Nonparametric data association END --------------------------------------------------------------------------------------------------------



    // ****************************************************
    //         STEP 3. Projected box data association     *
    // ****************************************************
    bool bAssoByProjectIou = false;
    int MaxAssoObjId_byProIou = -1;  //int nAssoByProId = -1;
    vector<int> vAssoObjIds_byProIou;
    if(ProIou_flag)//if((flag != "NA") && (flag != "IoU") && (flag != "NP"))
    {
        // 获得最大IOU的编号和值，记录部分bIoU大于0.25的
        float fIouMax = 0.0;

        for (int i = (int)ObjectMaps.size() - 1; i >= 0; i--)
        {
            Object_Map* obj3D = ObjectMaps[i];
            if (mclass_id != obj3D->mnClass){
                //std::cout<<"[ProIou]物体class不同"<<std::endl;
                continue;
            }


            if (obj3D->bad_3d||obj3D->backgroud_object){
                //std::cout<<"[ProIou]物体bad"<<std::endl;
                continue;
            }


            // notes： 小样本和多物体下使用
            if(little_mass_flag)
            {
                int df = (int) obj3D->mvObject_2ds.size();
                if ((mvMapPonits.size() >= 10) && (df > 8)) {
                    //std::cout << "[ProIou]小样本和多物体下使用" << std::endl;
                    continue;
                }
            }


            // step 3.1 compute IoU with bounding box constructed by projecting points.
            // notes: mRectFeaturePoints 为物体确定的最大包围框，RectCurrent为YOLOX确定的包围框
            float fIou = Converter::bboxOverlapratio(RectCurrent, obj3D->mRect_byProjectPoints);
            float fIou2 = Converter::bboxOverlapratio(mBox_cvRect_FeaturePoints, obj3D->mRect_byProjectPoints);
            fIou = max(fIou, fIou2);

            //可视化 for debug
            if(debug_iou_view)
            {
                cv::Mat mat_test = mpCurrentFrame->mColorImage.clone();
                cv::Scalar color = (200, 0, 0);
                cv::rectangle(mat_test, RectCurrent, cv::Scalar(255, 0, 0), 2);  //红
                cv::rectangle(mat_test, mBox_cvRect_FeaturePoints, cv::Scalar(0, 255, 0), 2);  //绿
                cv::rectangle(mat_test, obj3D->mRect_byProjectPoints, cv::Scalar(0, 0, 255), 2);//蓝
                cv::putText(mat_test, std::to_string(fIou), cv::Point(0, 500), cv::FONT_HERSHEY_DUPLEX, 1.0,
                            (0, 255, 0), 2);
                cv::resize(mat_test, mat_test, cv::Size(640 * 0.5, 480 * 0.5), 0, 0, cv::INTER_CUBIC);
                //cv::imshow("[ProIou]", mat_test);
                //std::cout << "[ProIou] " << fIou << " iou1:"
                //          << Converter::bboxOverlapratio(RectCurrent, obj3D->mRect_byProjectPoints) << ", iou2:"
                //          << Converter::bboxOverlapratio(mBox_cvRect_FeaturePoints, obj3D->mRect_byProjectPoints)
                //          << std::endl;
            }
            // record the max IoU and map object id.
            // notes: 找到最大重叠目标框
            // TODO: 只要ProIou大于0.25就认为是，潜在的关联对象？？
            if ((fIou >= 0.25) && (fIou > fIouMax))
            {
                fIouMax = fIou;
                MaxAssoObjId_byProIou = i;   //AssoObjId_byProIou  ProIouMaxObjId = i;
                // 这种记录方式是不是有点不科学，毕竟这个不是排序过的
                vAssoObjIds_byProIou.push_back(i);
                //std::cout<<"[ProIou] yes "<<std::endl;
            }

        }
        // step 3.2 update data association and record potential associated objects.
        if (fIouMax >= 0.25)
        {
            sort(vAssoObjIds_byProIou.begin(), vAssoObjIds_byProIou.end());

            if (bAssoByMotionIou || bAssoByNp)
            { //如果已经成功了MotionIou或NP的数据关联， 则检查一下结果是否一致。不一致，则加入潜在关联对象
                for (int j = vAssoObjIds_byProIou.size() - 1; j >= 0; j--)
                {
                    int AssoObjId_byMotionIouOrNP;
                    if (bAssoByMotionIou)
                        AssoObjId_byMotionIouOrNP = AssoObjId_byIou;
                    if (bAssoByNp)
                        AssoObjId_byMotionIouOrNP = AssoObjId_byNP;

                    if (vAssoObjIds_byProIou[j] == AssoObjId_byMotionIouOrNP)
                        continue;

                    // 加入潜在关联物体
                    int AssoObjId_byProIou = vAssoObjIds_byProIou[j];
                    AddPotentialAssociatedObjects(ObjectMaps, AssoObjId_byMotionIouOrNP, AssoObjId_byProIou);
                }
            }
            else
            {
                // update. 如果没有经过MotionIou或NP的数据关联，则更新到对应的Objct3d。
                bool bFlag = ObjectMaps[MaxAssoObjId_byProIou]->UpdateToObject3D(this, *mpCurrentFrame, ProIou); // 4: project iou.

                // association succeeded.
                if (bFlag)
                {
                    bAssoByProjectIou = true;          // associated by projecting box.
                }

                for (int j = vAssoObjIds_byProIou.size() - 1; j >= 0; j--)
                {
                    if (vAssoObjIds_byProIou[j] == MaxAssoObjId_byProIou)
                        continue;

                    //加入潜在关联物体  TODO:为什么要把maxProIou和其他ProIou关联。 答：本程序就是把同时匹配的物体，作为被updateObject3d的物体的关联物体。
                     AddPotentialAssociatedObjects(ObjectMaps, MaxAssoObjId_byProIou, ObjectMaps[vAssoObjIds_byProIou[j]]->mnId);
                }
            }
        }
    }
    // Projected box data association END ---------------------------------------------------------------------------------------

    // ************************************************
    //          STEP 4. t-test data association       *
    // ************************************************
    // step 4.1 Read t-distribution boundary value.
    float tTestData[122][9] = {0};
    ifstream infile;
    std::string filePath = WORK_SPACE_PATH + "/data/t_test.txt";
    infile.open(filePath);
    for (int i = 0; i < 122; i++)
    {
        for (int j = 0; j < 9; j++)
        {
            infile >> tTestData[i][j];
        }
    }
    infile.close();

    // step 4.2 t-test.
    bool bAssoByTtest = false;
    int nAssoByTId = -1;
    vector<int> vObjByTId;
    vector<int> vObjByTIdLower; // potential association.
    if(Ttest_flag)//if((flag != "NA") && (flag != "IoU") && (flag != "NP"))
    {
        for (int i = (int)ObjectMaps.size() - 1; i >= 0; i--)
        {
            Object_Map* obj3d = ObjectMaps[i];
            if (mclass_id != obj3d->mnClass)
                continue;

            if (obj3d->bad_3d||obj3d->backgroud_object)
                continue;

            // t-test results in 3 directions.
            float t_test;
            float t_test_x, t_test_y, t_test_z;

            // Degrees of freedom.
            int df = (int)obj3d->mvObject_2ds.size();
            // 场景复杂下才启用
            if (df <= 8)
                continue;

            // Iou.
            float fIou = Converter::bboxOverlapratio(RectCurrent, obj3d->mRect_byProjectPoints);
            float fIou2 = Converter::bboxOverlapratio(mBox_cvRect_FeaturePoints, obj3d->mRect_byProjectPoints);
            fIou = max(fIou, fIou2);

            // The distance from points to the object center.
            float dis_x, dis_y, dis_z;
            cv::Mat pos_points = mPos_world;
            dis_x = abs(obj3d->mAveCenter3D.at<float>(0, 0) - pos_points.at<float>(0, 0));
            dis_y = abs(obj3d->mAveCenter3D.at<float>(1, 0) - pos_points.at<float>(1, 0));
            dis_z = abs(obj3d->mAveCenter3D.at<float>(2, 0) - pos_points.at<float>(2, 0));

            // t-test.
            // notes: 对应论文中公式5
            t_test_x = dis_x / (obj3d->mCenterStandar_x / sqrt(df));
            t_test_y = dis_y / (obj3d->mCenterStandar_y / sqrt(df));
            t_test_z = dis_z / (obj3d->mCenterStandar_z / sqrt(df));

            // Satisfy t test.  // 5->0.05.
            // notes: curr_t_test < t_{n-1, \alpha /2} 详见t test单样本双侧临界表
            if ((t_test_x < tTestData[min((df - 1), 121)][5]) &&
                (t_test_y < tTestData[min((df - 1), 121)][5]) &&
                (t_test_z < tTestData[min((df - 1), 121)][5]))
            {
                vObjByTId.push_back(i);
                //std::cout<<"[Ttest] yes 1 "<<std::endl;
            }
            // If the T-test is not satisfied, but the IOU is large, reducing the significance.
            else if (fIou > 0.25)
            {
                // 显著性降低为0.001，容许值更大一些
                if ((t_test_x < tTestData[min((df - 1), 121)][8]) &&
                    (t_test_y < tTestData[min((df - 1), 121)][8]) &&
                    (t_test_z < tTestData[min((df - 1), 121)][8]))
                {
                    vObjByTId.push_back(i);
                    //std::cout<<"[Ttest] yes 2 "<<std::endl;
                }

                else if ((fIou > 0.25) && ((t_test_x + t_test_y + t_test_z) / 3 < 10))
                {
                    vObjByTId.push_back(i);
                    //std::cout<<"[Ttest] yes 3 "<<std::endl;
                }
                else
                {
                    vObjByTIdLower.push_back(i);
                    //std::cout<<"[Ttest] yes 4 "<<std::endl;
                }
            }
            else if ((t_test_x + t_test_y + t_test_z) / 3 < 4)
            {
                //计算obj3d在mpCurrentFrame中的投影边框mRect_byProjectPoints
                obj3d->ComputeProjectRectFrameToCurrentFrame(*mpCurrentFrame);
                //投影边框分别于RectCurrent和mBox_cvRect_FeaturePoint进行比较
                float fIou_force = Converter::bboxOverlapratio(RectCurrent, obj3d->mRect_byProjectPoints);
                float fIou2_force = Converter::bboxOverlapratio(mBox_cvRect_FeaturePoints, obj3d->mRect_byProjectPoints);
                fIou_force = max(fIou_force, fIou2_force);

                if (fIou_force > 0.25){
                    //std::cout<<"[Ttest] yes 5 "<<std::endl;
                    vObjByTIdLower.push_back(i);
                }
            }
        }

        // step 4.2 update data association and record potential associated objects.
        if (bAssoByMotionIou || bAssoByNp || bAssoByProjectIou)
        {
            int ReId;
            if (bAssoByMotionIou)
                ReId = AssoObjId_byIou;
            if (bAssoByNp)
                ReId = AssoObjId_byNP;
            if (bAssoByProjectIou)
                ReId = MaxAssoObjId_byProIou;
            // 疑问： vObjByTId和vObjByTIdLower有什么区别？
            if (vObjByTId.size() >= 1)
            {
                for (int j = 0; j < vObjByTId.size(); j++)
                {
                    if (vObjByTId[j] == ReId)
                        continue;

                    AddPotentialAssociatedObjects(ObjectMaps, ReId, ObjectMaps[vObjByTId[j]]->mnId);
                }
            }

            if (vObjByTIdLower.size() >= 0)
            {
                for (int j = 0; j < vObjByTIdLower.size(); j++)
                {
                    if (vObjByTIdLower[j] == ReId)
                        continue;

                    AddPotentialAssociatedObjects(ObjectMaps, ReId, ObjectMaps[vObjByTIdLower[j]]->mnId);
                }
            }
        }
        else
        {
            if (vObjByTId.size() >= 1)
            {
                for (int i = 0; i < vObjByTId.size(); i++)
                {
                    bool bFlag = ObjectMaps[vObjByTId[i]]->UpdateToObject3D(this, *mpCurrentFrame, t_test); // 3 是指 T 方法.

                    if (bFlag)
                    {
                        bAssoByTtest = true;
                        nAssoByTId = vObjByTId[i];

                        if (vObjByTId.size() > i)
                        {
                            for (int j = i + 1; j < vObjByTId.size(); j++)
                            {
                                AddPotentialAssociatedObjects(ObjectMaps, nAssoByTId, ObjectMaps[vObjByTId[j]]->mnId);
                            }
                        }

                        if (vObjByTIdLower.size() >= 0)
                        {
                            for (int j = 0; j < vObjByTIdLower.size(); j++)
                            {
                                if (vObjByTIdLower[j] == nAssoByTId)
                                    continue;

                                AddPotentialAssociatedObjects(ObjectMaps, nAssoByTId, ObjectMaps[vObjByTIdLower[j]]->mnId);
                            }
                        }

                        break;
                    }
                }
            }
        }
    }
    // t-test data association END ---------------------------------------------------------------------------------------

    // *************************************************
    //             STEP 4. create a new object         *
    // *************************************************
    //if (bAssoByMotionIou || bAssoByNp || bAssoByProjectIou || bAssoByTtest)
    //    return true;
     if (bAssoByMotionIou )
        return MotionIou;
     if ( bAssoByNp)
        return NoPara;
     if ( bAssoByProjectIou)
        return ProIou;
     if (bAssoByTtest)
        return t_test;
     return 0;
}

int Object_2D::creatObject()
{
    unique_lock<mutex> lock1(mMutexObjMapPoints);   //目的: 不让mBox_cvRect被修改, 以免造成程序错乱.
    unique_lock<mutex> lock2(mGlobalMutex);
    const cv::Mat ColorImage = mpCurrentFrame->mColorImage.clone();
    int associate = Object2D_DataAssociationWith_Object3D();    // data association with object3d in map. 如果关联失败， 则声称一个新的物体
    //switch (associate) {
    //            case MotionIou:   cout << "关联方式：MotionIou. " << endl; return 0;
    //            case NoPara:    cout << "关联方式：NoPara. " << endl;  return 0;
    //            case ProIou:    cout << "关联方式：ProIou. " << endl;  return 0;
    //            case t_test:    cout << "关联方式：t_test. " << endl;  return 0;
    //}
    if(associate)
        return 0;  //关联成功

    // If the object appears at the edge of the image, ignore.
    // 如果检测框位置不好，则不生成新的物体
    if ((this->mBox_cvRect.x < 10) || (this->mBox_cvRect.y < 10) ||
        (this->mBox_cvRect.x + this->mBox_cvRect.width > ColorImage.cols - 10) ||
        (this->mBox_cvRect.y + this->mBox_cvRect.height > ColorImage.rows - 10))
    {
        this->bad = true;
        return -1;  //检测框 位置不好
    }

    // create a 3d object in the map.

    Object_Map *Object3D = new Object_Map;  //zhang 生成新物体
    Object3D->mvObject_2ds.push_back(this);
    const vector<Object_Map*> ObjectMaps  = mpMap->GetObjects();
    Object3D->mnId = ObjectMaps.size();
    Object3D->mnClass = mclass_id;
    //std::cout<<"【debug】开始创建新物体,>>>>>>>>,id:"<< ObjectMaps.size()<<",>>>>>>>>,";
    Object3D->mnConfidence_foractive = 1;
    Object3D->mnAddedID_nouse = mpCurrentFrame->mnId;
    Object3D->mnLastAddID = mpCurrentFrame->mnId;
    Object3D->mnLastLastAddID = mpCurrentFrame->mnId;
    Object3D->mLastRect = mBox_cvRect;
    //Object3D->mPredictRect = obj->mBoxRect;       // for iou.
    // add properties of the point and save it to the object.

    for (size_t i = 0; i < mvMapPonits.size(); i++)
    {
        MapPoint *pMP = mvMapPonits[i];
        pMP->object_mnId = Object3D->mnId;
        pMP->object_class = Object3D->mnClass;
        pMP->viewdCount_forObjectId.insert(make_pair(Object3D->mnId, 1));  //记录point被某个object3d看见的次数

        // save to the object.
        Object3D->mvpMapObjectMappoints.push_back(pMP);
        Object3D->mvpMapObjectMappoints_NewForActive.push_back(pMP);
    }

    mnId = Object3D->mnId;

    // save this 2d object to current frame (associates with a 3d object in the map).
    mpCurrentFrame->mvObject_2ds.push_back(this);
    mpCurrentFrame->AppearNewObject = true;                   // 如果有新的物体,则当前帧生成为关键帧
    //mpCurrentFrame->AppearNewObject = true;

    // update object map.
    Object3D->ComputeMeanAndDeviation_3D();
    Object3D->IsolationForestDeleteOutliers();
    Object3D->ComputeMeanAndDeviation_3D();
    //当创建了一个新的物体，则计算它的IE
    //Object3D->ComputeIE();
    if( Object3D->mCuboid3D.lenth/Object3D->mCuboid3D.width>3 || Object3D->mCuboid3D.width/Object3D->mCuboid3D.lenth>3  )
        return 2;   //物体的长宽比，太畸形
    else{
        //mpMap->mvObjectMap.push_back(ObjectMapSingle);
        mpMap->AddObject(Object3D);
        //std::cout<<"存入map"<<std::endl;
        return 1;  //创建物体成功
    }
}

//  nonparametric test.
int Object_2D::NoParaDataAssociation(Object_Map *Object3D)
{
    // step 1. sample size.
    // 2d object ponits in the frame -- m.
    // 2d物体中的点的数量
    int m = (int)mvMapPonits.size();
    int OutPointNum1 = 0;
    for (int i = 0; i < (int)mvMapPonits.size(); i++)
    {
        MapPoint *p1 = mvMapPonits[i];
        if (p1->isBad())
        {
            OutPointNum1++;
            continue;
        }
    }
    m = m - OutPointNum1;

    // 3d object points in the object map -- n.
    // 3d物体中的点的数量
    int n = (int)Object3D->mvpMapObjectMappoints.size();
    int OutPointNum2 = 0;
    for (int i = 0; i < (int)Object3D->mvpMapObjectMappoints.size(); i++) // 帧中物体.
    {
        MapPoint *p2 = Object3D->mvpMapObjectMappoints[i];
        if (p2->isBad())
        {
            OutPointNum2++;
            continue;
        }
    }
    n = n - OutPointNum2;

    // 0: skip the nonparametric test and continue with the subsequent t-test.
    // 0： 如果2d point很少，则不进行np假设验证
    if (m < 20)
        return 0;

    // 2: association failed, compare next object.
    // 2： 如果3d point很少，则无法进行np假设检验（可能是因为3d point样本太少，导致无法计算均值和方差），认为关联失败
    if (n < 20)
        return 2;

    // Homogenization to avoid too many points of map object; n = 3 * m.
    // 对3dpoint，均匀化，防止地图点过多
    bool bSampleMapPoints = true;
    // 三个维度上，均匀化后的3d point坐标的集合
    vector<float> x_pt_map_sample;
    vector<float> y_pt_map_sample;
    vector<float> z_pt_map_sample;
    // 均匀化的操作
    if (bSampleMapPoints)
    {
        int step = 1;
        if (n > 3 * m)   // 如果3d point是 2d point的三倍多。则进行均匀化，
        {
            n = 3 * m;
            step = (int)Object3D->mvpMapObjectMappoints.size() / n;
            // 三个维度上，未均匀化的3d point坐标的集合
            vector<float> x_pt;
            vector<float> y_pt;
            vector<float> z_pt;
            for (int i = 0; i < (int)Object3D->mvpMapObjectMappoints.size(); i++)
            {
                MapPoint *p2 = Object3D->mvpMapObjectMappoints[i];
                if (p2->isBad())
                {
                    continue;
                }

                cv::Mat x3D2 = p2->GetWorldPos();
                x_pt.push_back(x3D2.at<float>(0, 0));
                y_pt.push_back(x3D2.at<float>(1, 0));
                z_pt.push_back(x3D2.at<float>(2, 0));
            }
            // 注意：这里是分开维度进行排序，相当于把点拆解了
            sort(x_pt.begin(), x_pt.end());
            sort(y_pt.begin(), y_pt.end());
            sort(z_pt.begin(), z_pt.end());
            for (int i = 0; i < x_pt.size(); i += step)
            {
                x_pt_map_sample.push_back(x_pt[i]);
                y_pt_map_sample.push_back(y_pt[i]);
                z_pt_map_sample.push_back(z_pt[i]);
            }
            n = x_pt_map_sample.size();
        }
        else    // 如果3d point 不到 2d point的三倍多。则不进行均匀化，
        {
            for (int jj = 0; jj < (int)Object3D->mvpMapObjectMappoints.size(); jj++) // 地图中物体.
            {
                MapPoint *p2 = Object3D->mvpMapObjectMappoints[jj];
                if (p2->isBad())
                {
                    continue;
                }

                cv::Mat x3D2 = p2->GetWorldPos();
                x_pt_map_sample.push_back(x3D2.at<float>(0, 0));
                y_pt_map_sample.push_back(x3D2.at<float>(1, 0));
                z_pt_map_sample.push_back(x3D2.at<float>(2, 0));
            }

            n = x_pt_map_sample.size();
        }
    }

    float w_x_2d_bigger_3d = 0.0;
    float w_y_2d_bigger_3d = 0.0;
    float w_z_2d_bigger_3d = 0.0;
    float w_x_3d_bigger_2d = 0.0;
    float w_y_3d_bigger_2d = 0.0;
    float w_z_3d_bigger_2d = 0.0;
    float w_x_3d_equal_2d = 0.0;
    float w_y_3d_equal_2d = 0.0;
    float w_z_3d_equal_2d = 0.0;
    float w_x = 0.0;
    float w_y = 0.0;
    float w_z = 0.0;

    // 三个维度上，2d point坐标的集合
    for (int ii = 0; ii < (int)mvMapPonits.size(); ii++)
    {
        MapPoint *p1 = mvMapPonits[ii];
        if (p1->isBad() )
            continue;

        cv::Mat x3D1 = p1->GetWorldPos();
        double x_2d = x3D1.at<float>(0, 0);
        double y_2d = x3D1.at<float>(1, 0);
        double z_2d = x3D1.at<float>(2, 0);

        // TODO: 感觉这块有点问题，没有进行排序
        if (!bSampleMapPoints)
        {
            //从Object3D->mvpMapObjectMappoints重新提取point,进行
        }

        // 将3d point
        if (bSampleMapPoints)
        {
            for (int jj = 0; jj < (int)x_pt_map_sample.size(); jj++)
            {
                double x_3d = x_pt_map_sample[jj];
                double y_3d = y_pt_map_sample[jj];
                double z_3d = z_pt_map_sample[jj];

                // 这里x_pt_map_sample实际上是排序过的，下面相当于对物体帧上的点进行排序
                // 记录2dpoint和3dpoint, 大小比较的次数. 例如所有的 2dpoint都大于3dpoint, 则w_x_2d_bigger_3d等于 n*m
                if (x_2d > x_3d)
                    w_x_2d_bigger_3d++;
                else if (x_2d < x_3d)
                    w_x_3d_bigger_2d++;
                else if (x_2d == x_3d)
                    w_x_3d_equal_2d++;

                if (y_2d > y_3d)
                    w_y_2d_bigger_3d++;
                else if (y_2d < y_3d)
                    w_y_3d_bigger_2d++;
                else if (y_2d == y_3d)
                    w_y_3d_equal_2d++;

                if (z_2d > z_3d)
                    w_z_2d_bigger_3d++;
                else if (z_2d < z_3d)
                    w_z_3d_bigger_2d++;
                else if (z_2d == z_3d)
                    w_z_3d_equal_2d++;
            }
        }
    }

    // step 2. `compute the rank sum.`
    // notes: 注意，当我们计算若干等值元素的排名时，会用这些元素排名的平均值作为它们在整个序列中的排名。
    // 这就是为什么要加 w_x_00 / 2 的原因
    // w_x_12 + w_x_00 / 2 为 m对应的排序； w_x_21 + w_x_00 / 2 为 n对应的排序. 感觉不对
    // W = min(W_p, W_q)
    // zhang: w = (n*n - R)   + n(n+1)/2
    w_x = min(w_x_2d_bigger_3d + m * (m + 1) / 2, w_x_3d_bigger_2d + n * (n + 1) / 2) + w_x_3d_equal_2d / 2;
    w_y = min(w_y_2d_bigger_3d + m * (m + 1) / 2, w_y_3d_bigger_2d + n * (n + 1) / 2) + w_y_3d_equal_2d / 2;
    w_z = min(w_z_2d_bigger_3d + m * (m + 1) / 2, w_z_3d_bigger_2d + n * (n + 1) / 2) + w_z_3d_equal_2d / 2;

    // step 3. compute the critical value.
    // TODO: 修改为wiki上的标准形式
    // notes: 这里的公式其实不太对，代码中的公式为r1 = r_l = m + s * sqrt(\sigma)
    // 其中\sigma = m * n * (m + n + 1) / 12
    // 但是这种情况下是有重复秩的，\sigma应该用论文上的公式才对
    // 不过论文上公式和wiki上也并不一致
    // 给的注释即s的值好像也不太对，标准正态分布表中对应80%是0.85, 对应1.28的是90%
    // 对应 1.96的是 97.5%
    float r1 = 0.5 * m * (m + n + 1) - 1.282 * sqrt(m * n * (m + n + 1) / 12); // 80%：1.282  85%:1.96
    float r2 = 0.5 * m * (m + n + 1) + 1.282 * sqrt(m * n * (m + n + 1) / 12); // 80%：1.282  85%:1.96

    // step 4. whether the 3 directions meet the nonparametric test.
    bool old_np = false;
    int add = 0;
    if (w_x > r1 && w_x < r2)
        add++;
    if (w_y > r1 && w_y < r2)
        add++;
    if (w_z > r1 && w_z < r2)
        add++;

    if (add == 3)
        old_np = true;  // Nonparametric Association succeeded.

    if (old_np == 1)
        return 1;       // success.
    else
        return 2;       // failure.
} // Object_2D::NoParaDataAssociation() END ------------------------------------------------------------

void Object_2D::AddObjectPoint(ORB_SLAM2::MapPoint *pMP) {
    unique_lock<mutex> lock2(mGlobalMutex);
    unique_lock<mutex> lock(mMutexPos);
    mvMapPonits.push_back(pMP);
    const cv::Mat PointPosWorld = pMP->GetWorldPos();                 // world frame.
    sum_pos_3d += PointPosWorld;  //深拷贝
}

void Object_2D::AddPotentialAssociatedObjects( vector<Object_Map*> obj3ds, int AssoId, int beAssoId){
    map<int, int>::iterator sit;
    sit = obj3ds[AssoId]->mReObj.find(obj3ds[beAssoId]->mnId);
    if (sit != obj3ds[AssoId]->mReObj.end())
    {
        int sit_sec = sit->second;
        obj3ds[AssoId]->mReObj.erase(obj3ds[beAssoId]->mnId);
        obj3ds[AssoId]->mReObj.insert(make_pair(obj3ds[beAssoId]->mnId, sit_sec + 1));
    }
    else
        obj3ds[AssoId]->mReObj.insert(make_pair(obj3ds[beAssoId]->mnId, 1));
}









// ************************************
// object3d 通用函数部分 *
// ************************************
void Object_Map::ComputeMeanAndDeviation_3D() {
    if(end_build)
        return;
    mSumPointsPos = cv::Mat::zeros(3,1,CV_32F);
    // remove bad points.
    {
        unique_lock<mutex> lock(mMutexMapPoints);
        vector<MapPoint *>::iterator pMP;
        int i = 0;
        for (pMP = mvpMapObjectMappoints.begin();
             pMP != mvpMapObjectMappoints.end();)
        {
            i++;

            cv::Mat pos = (*pMP)->GetWorldPos();

            if ((*pMP)->isBad()) {
                pMP = mvpMapObjectMappoints.erase(pMP);
            } else {
                mSumPointsPos += pos;
                ++pMP;
            }
        }
    }

    // step 0. mean(3d center).
    mAveCenter3D = mSumPointsPos / (mvpMapObjectMappoints.size());

    // step 1. 特征点簇中心在xyz上的偏差 standard deviation in 3 directions.
    float sum_x2 = 0, sum_y2 = 0, sum_z2 = 0;
    vector<float> x_pt, y_pt, z_pt;
    for (size_t i = 0; i < mvpMapObjectMappoints.size(); i++) {
        cv::Mat pos = mvpMapObjectMappoints[i]->GetWorldPos();
        cv::Mat pos_ave = mAveCenter3D;

        // （x-x^）^2
        sum_x2 += (pos.at<float>(0) - pos_ave.at<float>(0)) * (pos.at<float>(0) - pos_ave.at<float>(0));
        sum_y2 += (pos.at<float>(1) - pos_ave.at<float>(1)) * (pos.at<float>(1) - pos_ave.at<float>(1));
        sum_z2 += (pos.at<float>(2) - pos_ave.at<float>(2)) * (pos.at<float>(2) - pos_ave.at<float>(2));

        x_pt.push_back(pos.at<float>(0));
        y_pt.push_back(pos.at<float>(1));
        z_pt.push_back(pos.at<float>(2));
    }
    mStandar_x = sqrt(sum_x2 / (mvpMapObjectMappoints.size()));
    mStandar_y = sqrt(sum_y2 / (mvpMapObjectMappoints.size()));
    mStandar_z = sqrt(sum_z2 / (mvpMapObjectMappoints.size()));

    if (x_pt.size() == 0)
        return;

    // step 2. 特征点簇中心与各object2d中的点簇中心的偏差 standard deviation of centroids (observations from different frames).
    float sum_x2_c = 0, sum_y2_c = 0, sum_z2_c = 0;
    vector<float> x_c, y_c, z_c;
    for (size_t i = 0; i < this->mvObject_2ds.size(); i++)
    {
        cv::Mat pos = this->mvObject_2ds[i]->mPos_world.clone();
        cv::Mat pos_ave = this->mAveCenter3D.clone();

        // （x-x^）^2
        sum_x2_c += (pos.at<float>(0) - pos_ave.at<float>(0)) * (pos.at<float>(0) - pos_ave.at<float>(0));
        sum_y2_c += (pos.at<float>(1) - pos_ave.at<float>(1)) * (pos.at<float>(1) - pos_ave.at<float>(1));
        sum_z2_c += (pos.at<float>(2) - pos_ave.at<float>(2)) * (pos.at<float>(2) - pos_ave.at<float>(2));
    }
    mCenterStandar_x = sqrt(sum_x2_c / (this->mvObject_2ds.size()));
    mCenterStandar_y = sqrt(sum_y2_c / (this->mvObject_2ds.size()));
    mCenterStandar_z = sqrt(sum_z2_c / (this->mvObject_2ds.size()));

    // step 3. 生成Cuboid3D的中心和尺寸  update object center and scale.
    if (this->mvObject_2ds.size() < 5)
    {
        sort(x_pt.begin(), x_pt.end());
        sort(y_pt.begin(), y_pt.end());
        sort(z_pt.begin(), z_pt.end());

        if ((x_pt.size() == 0) || (y_pt.size() == 0) || (z_pt.size() == 0)) {
            this->bad_3d = true;
            //std::cout<<"object->bad 点数为0" <<std::endl;
            return;
        }

        float x_min = x_pt[0];
        float x_max = x_pt[x_pt.size() - 1];

        float y_min = y_pt[0];
        float y_max = y_pt[y_pt.size() - 1];

        float z_min = z_pt[0];
        float z_max = z_pt[z_pt.size() - 1];

        // centre. 这是点集外包框的中心
        mCuboid3D.cuboidCenter = Eigen::Vector3d((x_max + x_min) / 2, (y_max + y_min) / 2, (z_max + z_min) / 2);

        mCuboid3D.x_min = x_min;
        mCuboid3D.x_max = x_max;
        mCuboid3D.y_min = y_min;
        mCuboid3D.y_max = y_max;
        mCuboid3D.z_min = z_min;
        mCuboid3D.z_max = z_max;

        mCuboid3D.lenth = x_max - x_min;
        mCuboid3D.width = y_max - y_min;
        mCuboid3D.height = z_max - z_min;

        mCuboid3D.corner_1 = Eigen::Vector3d(x_min, y_min, z_min);
        mCuboid3D.corner_2 = Eigen::Vector3d(x_max, y_min, z_min);
        mCuboid3D.corner_3 = Eigen::Vector3d(x_max, y_max, z_min);
        mCuboid3D.corner_4 = Eigen::Vector3d(x_min, y_max, z_min);
        mCuboid3D.corner_5 = Eigen::Vector3d(x_min, y_min, z_max);
        mCuboid3D.corner_6 = Eigen::Vector3d(x_max, y_min, z_max);
        mCuboid3D.corner_7 = Eigen::Vector3d(x_max, y_max, z_max);
        mCuboid3D.corner_8 = Eigen::Vector3d(x_min, y_max, z_max);

        mCuboid3D.corner_1_w = Eigen::Vector3d(x_min, y_min, z_min);
        mCuboid3D.corner_2_w = Eigen::Vector3d(x_max, y_min, z_min);
        mCuboid3D.corner_3_w = Eigen::Vector3d(x_max, y_max, z_min);
        mCuboid3D.corner_4_w = Eigen::Vector3d(x_min, y_max, z_min);
        mCuboid3D.corner_5_w = Eigen::Vector3d(x_min, y_min, z_max);
        mCuboid3D.corner_6_w = Eigen::Vector3d(x_max, y_min, z_max);
        mCuboid3D.corner_7_w = Eigen::Vector3d(x_max, y_max, z_max);
        mCuboid3D.corner_8_w = Eigen::Vector3d(x_min, y_max, z_max);
    }

    // step 4. update object pose。  更新物体在世界下的坐标
    Update_Twobj();


    // step 5. 计算8个定点的世界坐标
    vector<float> x_pt_obj, y_pt_obj, z_pt_obj;
    g2o::SE3Quat pose =  Converter::toSE3Quat(this->mCuboid3D.pose_mat);
    for (size_t i = 0; i < mvpMapObjectMappoints.size(); i++) {
        // world frame.
        Eigen::Vector3d PointPos_world = Converter::toVector3d(mvpMapObjectMappoints[i]->GetWorldPos());

        // object frame.   Twobj.inv * point = Tobjw * point
        Eigen::Vector3d PointPos_object = pose.inverse() * PointPos_world;
        x_pt_obj.push_back(PointPos_object[0]);
        y_pt_obj.push_back(PointPos_object[1]);
        z_pt_obj.push_back(PointPos_object[2]);
    }

    if (x_pt_obj.size() == 0)
        return;

    // rank.
    int s = x_pt_obj.size();
    sort(x_pt_obj.begin(), x_pt_obj.end());
    sort(y_pt_obj.begin(), y_pt_obj.end());
    sort(z_pt_obj.begin(), z_pt_obj.end());

    float x_min_obj = x_pt_obj[0];
    float x_max_obj = x_pt_obj[s - 1];
    float y_min_obj = y_pt_obj[0];
    float y_max_obj = y_pt_obj[s - 1];
    float z_min_obj = z_pt_obj[0];
    float z_max_obj = z_pt_obj[s - 1];

    // update object vertices and translate it to world frame.
    // g2o::SE3Quat Converter::toSE3Quat(const cv::Mat &cvT)
    mCuboid3D.corner_1 = pose * Eigen::Vector3d(x_min_obj, y_min_obj, z_min_obj);
    mCuboid3D.corner_2 = pose * Eigen::Vector3d(x_max_obj, y_min_obj, z_min_obj);
    mCuboid3D.corner_3 = pose * Eigen::Vector3d(x_max_obj, y_max_obj, z_min_obj);
    mCuboid3D.corner_4 = pose * Eigen::Vector3d(x_min_obj, y_max_obj, z_min_obj);
    mCuboid3D.corner_5 = pose * Eigen::Vector3d(x_min_obj, y_min_obj, z_max_obj);
    mCuboid3D.corner_6 = pose * Eigen::Vector3d(x_max_obj, y_min_obj, z_max_obj);
    mCuboid3D.corner_7 = pose * Eigen::Vector3d(x_max_obj, y_max_obj, z_max_obj);
    mCuboid3D.corner_8 = pose * Eigen::Vector3d(x_min_obj, y_max_obj, z_max_obj);

    //// object frame -> world frame (without yaw, parallel to world frame).
    //g2o::SE3Quat pose_without_yaw =  Converter::toSE3Quat(this->mCuboid3D.pose_noyaw_mat);
    //mCuboid3D.corner_1_w = pose_without_yaw * Eigen::Vector3d(x_min_obj, y_min_obj, z_min_obj);
    //mCuboid3D.corner_2_w = pose_without_yaw * Eigen::Vector3d(x_max_obj, y_min_obj, z_min_obj);
    //mCuboid3D.corner_3_w = pose_without_yaw * Eigen::Vector3d(x_max_obj, y_max_obj, z_min_obj);
    //mCuboid3D.corner_4_w = pose_without_yaw * Eigen::Vector3d(x_min_obj, y_max_obj, z_min_obj);
    //mCuboid3D.corner_5_w = pose_without_yaw * Eigen::Vector3d(x_min_obj, y_min_obj, z_max_obj);
    //mCuboid3D.corner_6_w = pose_without_yaw * Eigen::Vector3d(x_max_obj, y_min_obj, z_max_obj);
    //mCuboid3D.corner_7_w = pose_without_yaw * Eigen::Vector3d(x_max_obj, y_max_obj, z_max_obj);
    //mCuboid3D.corner_8_w = pose_without_yaw * Eigen::Vector3d(x_min_obj, y_max_obj, z_max_obj);


    // step 6. 计算cubic的长宽高和半径
    this->mCuboid3D.lenth = x_max_obj - x_min_obj;
    this->mCuboid3D.width = y_max_obj - y_min_obj;
    this->mCuboid3D.height = z_max_obj - z_min_obj;
    this->mCuboid3D.cuboidCenter = (mCuboid3D.corner_2 + mCuboid3D.corner_8) / 2;
    Update_Twobj();

    // maximum radius.
    float fRMax = 0.0;
    vector<cv::Mat> vCornerMat;
    vCornerMat.resize(8);
    for (int i = 0; i < 8; i++) {
        cv::Mat mDis = cv::Mat::zeros(3, 1, CV_32F);
        if (i == 0)
            vCornerMat[i] = Converter::toCvMat(mCuboid3D.corner_1);
        if (i == 1)
            vCornerMat[i] = Converter::toCvMat(mCuboid3D.corner_2);
        if (i == 2)
            vCornerMat[i] = Converter::toCvMat(mCuboid3D.corner_3);
        if (i == 3)
            vCornerMat[i] = Converter::toCvMat(mCuboid3D.corner_4);
        if (i == 4)
            vCornerMat[i] = Converter::toCvMat(mCuboid3D.corner_5);
        if (i == 5)
            vCornerMat[i] = Converter::toCvMat(mCuboid3D.corner_6);
        if (i == 6)
            vCornerMat[i] = Converter::toCvMat(mCuboid3D.corner_7);
        if (i == 7)
            vCornerMat[i] = Converter::toCvMat(mCuboid3D.corner_8);

        mDis = mAveCenter3D - vCornerMat[i];
        float fTmp = sqrt(mDis.at<float>(0) * mDis.at<float>(0) + mDis.at<float>(1) * mDis.at<float>(1) +
                          mDis.at<float>(2) * mDis.at<float>(2));
        fRMax = max(fRMax, fTmp);
    }
    mCuboid3D.mfRMax = fRMax;


    // step 7. 计算cubic中心, 与各object2d点云簇中心, 在直线距离上的偏差
    // standard deviation of distance.
    float dis = 0;
    for (size_t i = 0; i < mvObject_2ds.size(); i++) {
        float center_sum_x2 = 0, center_sum_y2 = 0, center_sum_z2 = 0;

        const cv::Mat pos = mvObject_2ds[i]->mPos_world.clone();
        const cv::Mat pos_ave = mAveCenter3D.clone();

        center_sum_x2 =
                (pos.at<float>(0) - pos_ave.at<float>(0)) * (pos.at<float>(0) - pos_ave.at<float>(0)); // dis_x^2
        center_sum_y2 =
                (pos.at<float>(1) - pos_ave.at<float>(1)) * (pos.at<float>(1) - pos_ave.at<float>(1)); // dis_y^2
        center_sum_z2 =
                (pos.at<float>(2) - pos_ave.at<float>(2)) * (pos.at<float>(2) - pos_ave.at<float>(2)); // dis_z^2

        dis += sqrt(center_sum_x2 + center_sum_y2 + center_sum_z2);
    }
    mCenterStandar = sqrt(dis / (mvObject_2ds.size()));

    // step 8. 计算ie
    this->ComputeIE();
    if(mIE < mIEThresholdEndMapping )
        end_build = true;
    if(mbPublishIEwheel)
        this->PublishIE();

    // step 9. 计算观测主方向
    this->ComputeMainDirection();
}

// 移除object3d中的outliers，重新优化物体的坐标和尺度
// 疑问： 这和ComputeMeanAndStandard有什么区别？
// 答：似乎是专属于biForest下的ComputeMeanAndStandard
// remove outliers and refine the object position and scale by IsolationForest.
void Object_Map::IsolationForestDeleteOutliers(){
    if(end_build)
        return;

    if(!iforest_flag)
        return;
    //if ((this->mnClass == 75) /*vase花瓶*/ || (this->mnClass == 64) /*mouse鼠标*/ || (this->mnClass == 65) /*remote遥控器*/ )
    //    return;

    //(1)通过if语句设置阈值，如果this->mnClass == 62，阈值为0.65，否则为0.6。
    float th = mIForest_thresh;
    if (this->mnClass == 62/*tv电视*/)
        th = 0.65;  // 阈值更高了,说明剔除的点变少了.说明这个物体的点相对分散

    //(2)创建一个vector data来存储MapPoint对象的3D坐标。
    std::vector<std::array<float, 3>> data; // uint32_t
    if (mvpMapObjectMappoints.size() < 30)
        return;
    if ((this->mnClass == 75) /*vase花瓶*/){
        for (size_t i = 0; i < mvpMapObjectMappoints.size(); i++)
        {
            // 将point的坐标，从cv::mat转为std::array
            MapPoint *pMP = mvpMapObjectMappoints[i];
            cv::Mat pos = pMP->GetWorldPos();

            std::array<float, 3> temp;
            temp[0] = pos.at<float>(0);
            temp[1] = pos.at<float>(1);
            temp[2] = 0.0;
            data.push_back(temp);
        }
    }
    else{
        for (size_t i = 0; i < mvpMapObjectMappoints.size(); i++)
        {
            // 将point的坐标，从cv::mat转为std::array
            MapPoint *pMP = mvpMapObjectMappoints[i];
            cv::Mat pos = pMP->GetWorldPos();

            std::array<float, 3> temp;
            temp[0] = pos.at<float>(0);
            temp[1] = pos.at<float>(1);
            temp[2] = pos.at<float>(2);
            data.push_back(temp);
        }
    }


    // (3)构建随机森林 筛选器: 创建一个Isolation Forest算法实例forest，并使用data来构建树，并指定采样数量为mvpMapObjectMappoints的一半。

    auto now = std::chrono::system_clock::now();
    auto seed = now.time_since_epoch().count();// 获取当前时间的时间戳作为种子
    std::mt19937 rng(seed);// 根据种子生成随机数生成器
    std::uniform_int_distribution<uint32_t> dist(0, std::numeric_limits<uint32_t>::max());
    uint32_t random_uint32 = dist(rng);
    iforest::IsolationForest<float, 3> forest; // uint32_t
    if (!forest.Build(50, random_uint32, data, ((int)mvpMapObjectMappoints.size() / 2)))
                    //数的数量, 随机数生成器的种子12345, 输入的数据,  采样的数量(此处是data的一半)
    {
        std::cerr << "Failed to build Isolation Forest.\n";
        return;
    }
    std::vector<double> anomaly_scores;

    // (4)计算Anomaly_score, 并将大于阈值的point, 标记为outlier:
    // 使用GetAnomalyScores函数来计算每个MapPoint对象的Anomaly_score
    if (!forest.GetAnomalyScores(data, anomaly_scores))
    {
        std::cerr << "Failed to calculate anomaly scores.\n";
        return;
    }

    //(5)将Anomaly_score大于阈值的点的索引存储在outlier_ids vector中。
    std::vector<int> outlier_ids;
    for (uint32_t i = 0; i < (int)mvpMapObjectMappoints.size(); i++)
    {
        // 如果Anomaly_score大于阈值, 则认为是outlier
        if (anomaly_scores[i] > th)
            outlier_ids.push_back(i);
    }

    if (outlier_ids.empty())
        return;

    // (6)将 outliers 从object3d的mvpMapObjectMappoints 中移除.
    // 遍历mvpMapObjectMappoints，对于outlier_ids中的每个索引，从mvpMapObjectMappoints中删除相应的MapPoint对象，并从mSumPointsPos中减去该MapPoint对象的位置
    int id_Mappoint = -1;
    int id_MOutpoint_out = 0;
    unique_lock<mutex> lock(mMutexMapPoints); // lock.
    vector<MapPoint *>::iterator pMP;
    for (pMP = mvpMapObjectMappoints.begin();
         pMP != mvpMapObjectMappoints.end();)
    {
        // pMP和numMappoint, 从头开始递加, 依次检阅
        id_Mappoint++;

        cv::Mat pos = (*pMP)->GetWorldPos();

        if (id_Mappoint == outlier_ids[id_MOutpoint_out])
        {
            id_MOutpoint_out++;
            pMP = mvpMapObjectMappoints.erase(pMP);
            //std::cout<<"[iforest debug] mSumPointsPos size:"<< mSumPointsPos.size() <<", pos size:"<< pos.size() <<std::endl;
            mSumPointsPos -= pos;
        }
        else
        {
            ++pMP;
        }
    }
}

void Object_Map::Update_Twobj()      //更新物体在世界下的坐标
{


    // Rotation matrix.
    float cp = cos(mCuboid3D.rotP);
    float sp = sin(mCuboid3D.rotP);
    float sr = sin(mCuboid3D.rotR);
    float cr = cos(mCuboid3D.rotR);
    float sy = sin(mCuboid3D.rotY);
    float cy = cos(mCuboid3D.rotY);
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
    Twobj.at<float>(0, 3) = mCuboid3D.cuboidCenter[0];

    Twobj.at<float>(1, 0) = R_result.at<float>(1, 0);
    Twobj.at<float>(1, 1) = R_result.at<float>(1, 1);
    Twobj.at<float>(1, 2) = R_result.at<float>(1, 2);
    Twobj.at<float>(1, 3) = mCuboid3D.cuboidCenter[1];

    Twobj.at<float>(2, 0) = R_result.at<float>(2, 0);
    Twobj.at<float>(2, 1) = R_result.at<float>(2, 1);
    Twobj.at<float>(2, 2) = R_result.at<float>(2, 2);
    Twobj.at<float>(2, 3) = mCuboid3D.cuboidCenter[2];

    Twobj.at<float>(3, 0) = 0;
    Twobj.at<float>(3, 1) = 0;
    Twobj.at<float>(3, 2) = 0;
    Twobj.at<float>(3, 3) = 1;

    // note no yaw.
    cv::Mat Twobj_without_yaw = cv::Mat::eye(4, 4, CV_32F);
    Twobj_without_yaw.at<float>(0, 3) = mCuboid3D.cuboidCenter[0];//mAveCenter3D.at<float>(0);
    Twobj_without_yaw.at<float>(1, 3) = mCuboid3D.cuboidCenter[1];
    Twobj_without_yaw.at<float>(2, 3) = mCuboid3D.cuboidCenter[2];//mAveCenter3D.at<float>(2);

    // SE3.[origin]
    //g2o::SE3Quat obj_pose = Converter::toSE3Quat(Twobj);
    //g2o::SE3Quat obj_pose_without_yaw = Converter::toSE3Quat(Twobj_without_yaw);
    //this->mCuboid3D.pose = obj_pose;
    //this->mCuboid3D.pose_without_yaw = obj_pose_without_yaw;
    this->mCuboid3D.pose_mat = Twobj;
    //this->mCuboid3D.pose_noyaw_mat = Twobj_without_yaw;
}

void Object_Map::ComputeProjectRectFrameToCurrentFrame(Frame &Frame)
{
    const cv::Mat Rcw = Frame.mTcw.rowRange(0, 3).colRange(0, 3);
    const cv::Mat tcw = Frame.mTcw.rowRange(0, 3).col(3);
    vector<float> x_pt;
    vector<float> y_pt;
    for (int j = 0; j < mvpMapObjectMappoints.size(); j++)
    {
        MapPoint *pMP = mvpMapObjectMappoints[j];
        cv::Mat PointPosWorld = pMP->GetWorldPos();

        cv::Mat PointPosCamera = Rcw * PointPosWorld + tcw;

        const float xc = PointPosCamera.at<float>(0);
        const float yc = PointPosCamera.at<float>(1);
        const float invzc = 1.0 / PointPosCamera.at<float>(2);

        float u = Frame.fx * xc * invzc + Frame.cx;
        float v = Frame.fy * yc * invzc + Frame.cy;

        x_pt.push_back(u);
        y_pt.push_back(v);

    }

    if (x_pt.size() == 0)
        return;

    sort(x_pt.begin(), x_pt.end());
    sort(y_pt.begin(), y_pt.end());
    float x_min = x_pt[0];
    float x_max = x_pt[x_pt.size() - 1];
    float y_min = y_pt[0];
    float y_max = y_pt[y_pt.size() - 1];

    if (x_min < 0)
        x_min = 0;
    if (y_min < 0)
        y_min = 0;
    if (x_max > Frame.mColorImage.cols)
        x_max = Frame.mColorImage.cols;
    if (y_max > Frame.mColorImage.rows)
        y_max = Frame.mColorImage.rows;

    mRect_byProjectPoints = cv::Rect(x_min, y_min, x_max - x_min, y_max - y_min);
}


void Object_Map::UpdateCoView(Object_Map *Obj_CoView)
{
    int nObjId = Obj_CoView->mnId;

    map<int, int>::iterator sit;
    sit = this->mmAppearSametime.find(nObjId);

    if (sit != this->mmAppearSametime.end())
    {
        int sit_sec = sit->second;
        this->mmAppearSametime.erase(nObjId);
        this->mmAppearSametime.insert(make_pair(nObjId, sit_sec + 1));
    }
    else
        this->mmAppearSametime.insert(make_pair(nObjId, 1));   // first co-view.
}

vector<MapPoint* > Object_Map::GetObjectMappoints(){
    //unique_lock<mutex> lock(mMutex); mvpMapObjectMappoints_NewForActive
    unique_lock<mutex> lock(mMutexMapPoints);
    return vector<MapPoint* >(mvpMapObjectMappoints.begin(), mvpMapObjectMappoints.end());
}

vector<MapPoint* > Object_Map::GetNewObjectMappoints(){
    //unique_lock<mutex> lock(mMutex); mvpMapObjectMappoints_NewForActive
    unique_lock<mutex> lock(mMutexMapPoints);
    return vector<MapPoint* >(mvpMapObjectMappoints_NewForActive.begin(), mvpMapObjectMappoints_NewForActive.end());
}



// ************************************
// object3d track部分 *
// ************************************

// MotionIou 1,  NoPara 2,  t_test 3,  ProIou 4
// return false的原因: class id不匹配; IOU太小；  Frame id没有递增;
bool Object_Map::UpdateToObject3D(Object_2D* Object_2d, Frame &mCurrentFrame, int Flag){
    //std::cout<<"UpdateToObject3D "<<Flag<<std::endl;

    if (Object_2d->mclass_id != mnClass)
        return false;

    const cv::Mat Rcw = mCurrentFrame.mTcw.rowRange(0, 3).colRange(0, 3);
    const cv::Mat tcw = mCurrentFrame.mTcw.rowRange(0, 3).col(3);

    // step 1. whether the box projected into the image changes greatly after the new point cloud is associated.
    if ((Flag != MotionIou) && (Flag != ProIou))
    // 此步只用于 NoPara 和 t_test
    // 因为以下,类似于IOU的匹配部分. 而NoPara和t_test之前认为IOU匹配度不好, 所以要再次验证IOU是否匹配.
    {
        // notes：ProjectRect1表示物体地图上的目标框投影; ProjectRect2表示融入当前帧的目标框投影
        cv::Rect ProjectRect_3D;
        cv::Rect ProjectRect_3Dand2D;

        // projected bounding box1.
        this->ComputeProjectRectFrameToCurrentFrame(mCurrentFrame);
        ProjectRect_3D = this->mRect_byProjectPoints;

        // mixed points of frame object and map object.
        vector<float> x_pt;
        vector<float> y_pt;
        // notes: Obj_c_MapPonits为物体上的3D点，mvpMapObjectMappoints为物体地图上的点
        for (int i = 0; i < Object_2d->mvMapPonits.size(); ++i)
        {
            MapPoint *pMP = Object_2d->mvMapPonits[i];
            cv::Mat PointPosWorld = pMP->GetWorldPos();
            cv::Mat PointPosCamera = Rcw * PointPosWorld + tcw;

            const float xc = PointPosCamera.at<float>(0);
            const float yc = PointPosCamera.at<float>(1);
            const float invzc = 1.0 / PointPosCamera.at<float>(2);

            float u = mCurrentFrame.fx * xc * invzc + mCurrentFrame.cx;
            float v = mCurrentFrame.fy * yc * invzc + mCurrentFrame.cy;

            x_pt.push_back(u);
            y_pt.push_back(v);
        }
        for (int j = 0; j < mvpMapObjectMappoints.size(); ++j)
        {
            MapPoint *pMP = mvpMapObjectMappoints[j];
            cv::Mat PointPosWorld = pMP->GetWorldPos();
            cv::Mat PointPosCamera = Rcw * PointPosWorld + tcw;

            const float xc = PointPosCamera.at<float>(0);
            const float yc = PointPosCamera.at<float>(1);
            const float invzc = 1.0 / PointPosCamera.at<float>(2);

            float u = mCurrentFrame.fx * xc * invzc + mCurrentFrame.cx;
            float v = mCurrentFrame.fy * yc * invzc + mCurrentFrame.cy;

            x_pt.push_back(u);
            y_pt.push_back(v);
        }

        // rank.
        sort(x_pt.begin(), x_pt.end());
        sort(y_pt.begin(), y_pt.end());
        float x_min = x_pt[0];
        float x_max = x_pt[x_pt.size() - 1];
        float y_min = y_pt[0];
        float y_max = y_pt[y_pt.size() - 1];

        if (x_min < 0)
            x_min = 0;
        if (y_min < 0)
            y_min = 0;
        if (x_max > mCurrentFrame.mColorImage.cols)
            x_max = mCurrentFrame.mColorImage.cols;
        if (y_max > mCurrentFrame.mColorImage.rows)
            y_max = mCurrentFrame.mColorImage.rows;

        // projected bounding box2.
        ProjectRect_3Dand2D = cv::Rect(x_min, y_min, x_max - x_min, y_max - y_min);

        // 4. 计算 Iou
        float fIou = Converter::bboxOverlapratio(ProjectRect_3D, ProjectRect_3Dand2D);
        float fIou2 = Converter::bboxOverlapratioFormer(ProjectRect_3Dand2D, Object_2d->mBox_cvRect);
        if ((fIou < 0.5) && (fIou2 < 0.8))  //fIou2不能小于0.8,是因为
            return false;
    }

    // step 2. update the ID of the last frame
    // 更新: last和lastlast的 frame id 和 物体检测框
    if (mnLastAddID != (int)mCurrentFrame.mnId)
    {
        mnLastLastAddID = mnLastAddID;
        mnLastAddID = mCurrentFrame.mnId;
        mLastLastRect = mLastRect;
        mLastRect = Object_2d->mBox_cvRect;
        mnConfidence_foractive++;
        AddObj2d(Object_2d);//this->mvObject_2ds.push_back(Object_2d);
    }
    else
        return false;

    Object_2d->mnId = mnId;


    {
        // step 3. Add the point cloud of the frame object to the map object
        // 将当前帧的点云添加到， map中的已有物体中。
        for (size_t j = 0; j < Object_2d->mvMapPonits.size(); ++j)
        {
            MapPoint *pMP = Object_2d->mvMapPonits[j];

            cv::Mat pointPos = pMP->GetWorldPos();
            cv::Mat mDis = mAveCenter3D - pointPos;
            float fDis = sqrt(mDis.at<float>(0) * mDis.at<float>(0) + mDis.at<float>(1) * mDis.at<float>(1) + mDis.at<float>(2) * mDis.at<float>(2));

            float th = 1.0;
            if (mvObject_2ds.size() > 5)
                th = 0.9;

            if (fDis > th * mCuboid3D.mfRMax)
                continue;

            pMP->object_mnId = mnId;
            pMP->object_class = mnClass;

            // 记录此point被此object3d看见的次数
            map<int, int>::iterator sit;
            sit = pMP->viewdCount_forObjectId.find(this->mnId);
            if (sit != pMP->viewdCount_forObjectId.end())
            {
                int sit_sec = sit->second;
                pMP->viewdCount_forObjectId.erase(this->mnId);   //zhang报错
                pMP->viewdCount_forObjectId.insert(make_pair(this->mnId, sit_sec + 1));
            }
            else
            {
                pMP->viewdCount_forObjectId.insert(make_pair(this->mnId, 1));
            }


            // notes: 检查有无重复地图点，并添加新的地图点
            {
                unique_lock<mutex> lock(mMutexMapPoints);
                bool new_point = true;
                // old points.
                for (size_t m = 0; m < mvpMapObjectMappoints.size(); ++m)
                {
                    cv::Mat obj_curr_pos = pMP->GetWorldPos();
                    cv::Mat obj_map_pos = mvpMapObjectMappoints[m]->GetWorldPos();

                    if (cv::countNonZero(obj_curr_pos - obj_map_pos) == 0)
                    {
                        mvpMapObjectMappoints[m]->feature_uvCoordinate = pMP->feature_uvCoordinate;
                        new_point = false;
                        break;
                    }
                }
                // new point.
                if (new_point)
                {
                    mvpMapObjectMappoints.push_back(pMP);

                    mvpMapObjectMappoints_NewForActive.push_back(pMP);

                    cv::Mat x3d = pMP->GetWorldPos();
                    mSumPointsPos += x3d;
                }
            }
        }

        // step 4. the historical point cloud is projected into the image, and the points not in the box(should not on the edge) are removed.
        // 将历史point投影到图像中，如果不在box中，则提出
        if ((Object_2d->mBox_cvRect.x > 25) && (Object_2d->mBox_cvRect.y > 25) &&
            (Object_2d->mBox_cvRect.x + Object_2d->mBox_cvRect.width < mCurrentFrame.mColorImage.cols - 25) &&
            (Object_2d->mBox_cvRect.y + Object_2d->mBox_cvRect.height < mCurrentFrame.mColorImage.rows - 25))
        {
            unique_lock<mutex> lock(mMutexMapPoints); // lock.
            vector<MapPoint *>::iterator pMP;
            for (pMP = mvpMapObjectMappoints.begin();
                 pMP != mvpMapObjectMappoints.end();)
            {
                // 在map中找到当前物体点上的物体类别
                int sit_sec = 0;
                map<int , int>::iterator sit;
                sit = (*pMP)->viewdCount_forObjectId.find(mnId);
                if (sit != (*pMP)->viewdCount_forObjectId.end())
                {
                    sit_sec = sit->second;
                }
                if (sit_sec > 8)
                {
                    ++pMP;
                    continue;
                }

                cv::Mat PointPosWorld = (*pMP)->GetWorldPos();
                cv::Mat PointPosCamera = Rcw * PointPosWorld + tcw;

                const float xc = PointPosCamera.at<float>(0);
                const float yc = PointPosCamera.at<float>(1);
                const float invzc = 1.0 / PointPosCamera.at<float>(2);

                float u = mCurrentFrame.fx * xc * invzc + mCurrentFrame.cx;
                float v = mCurrentFrame.fy * yc * invzc + mCurrentFrame.cy;

                if ((u > 0 && u < mCurrentFrame.mColorImage.cols) && (v > 0 && v < mCurrentFrame.mColorImage.rows))
                {
                    if (!Object_2d->mBox_cvRect.contains(cv::Point2f(u, v)))
                    {
                        pMP = mvpMapObjectMappoints.erase(pMP);
                        mSumPointsPos -= PointPosWorld;
                    }
                    else
                    {
                        ++pMP;
                    }
                }
                else
                {
                    ++pMP;
                }
            }
        }

        // step 5. update object mean.
        this->ComputeMeanAndDeviation_3D();

        // step 6. i-Forest.
        this->IsolationForestDeleteOutliers();
    }

    mCurrentFrame.mvObject_2ds.push_back(Object_2d);
    //std::cout   <<"与旧物体融合成功，cude h:" <<this->mCuboid3D.height
    //            <<", cude w:" <<this->mCuboid3D.width
    //            <<", cude l:" <<this->mCuboid3D.lenth
    //            <<std::endl;
    return true;
}



bool Object_Map::WhetherOverlap(Object_Map *CompareObj)
{
    // distance between two centers.
    float dis_x = abs(mCuboid3D.cuboidCenter(0) - CompareObj->mCuboid3D.cuboidCenter(0));
    float dis_y = abs(mCuboid3D.cuboidCenter(1) - CompareObj->mCuboid3D.cuboidCenter(1));
    float dis_z = abs(mCuboid3D.cuboidCenter(2) - CompareObj->mCuboid3D.cuboidCenter(2));

    float sum_lenth_half = mCuboid3D.lenth / 2 + CompareObj->mCuboid3D.lenth / 2;
    float sum_width_half = mCuboid3D.width / 2 + CompareObj->mCuboid3D.width / 2;
    float sum_height_half = mCuboid3D.height / 2 + CompareObj->mCuboid3D.height / 2;

    // whether overlap.
    if ((dis_x < sum_lenth_half) && (dis_y < sum_width_half) && (dis_z < sum_height_half))
        return true;
    else
        return false;
}


// ************************************
// object3d localmap部分 *
// ************************************
void Object_Map::SearchAndMergeMapObjs_fll(Map *mpMap)
{
    //首先检查mReObj是否为空
    if(mReObj.empty())
        return;

    //融合潜在的关联物体。
    map<int, int>::iterator sit;
    std::vector<Object_Map*> obj_3ds = mpMap->GetObjects();
    for (sit = mReObj.end(); sit != mReObj.begin(); sit--)
    {
        int nObjId = sit->first;
        //std::cout<<"debug SearchAndMergeMapObjs_fll: obj_3ds数量："<<obj_3ds.size()  << " ,nObjId: "<<nObjId <<std::endl;
        //ROS_INFO("SearchAndMergeMapObjs_fll: %d, %d", nObjId, sit->second );
        Object_Map* obj_ass = obj_3ds[nObjId];  // new bug: 可以先检查下是不是费控  查看nObjId是否大于obj_3ds.size(). 或者去掉这一行
        if (sit->second < 3)
            continue;

        if (obj_ass->bad_3d  || obj_ass->backgroud_object )
            continue;

        // 通过双样本Ttest测试，验证是否是同一个物体
        bool bDoubelTtest = this->DoubleSampleTtest_fll(obj_ass);
        bool bSametime = true;

        // make sure they don't appear at the same time.
        // 查询是否两个物体同时出现.
        map<int, int>::iterator sit2;
        sit2 = mmAppearSametime.find(nObjId);
        if (sit2 != mmAppearSametime.end())
        {
            continue;
        }
        else
            bSametime = false;

        // 如果满足, 不同时出现,且通过双样本Ttest,则融为一体. 保留被观测次数多的一方(obj2d多的一方)
        if((!bSametime || bDoubelTtest))
        {
            int nAppearTimes1 = mvObject_2ds.size();
            int nAppearTimes2 = obj_ass->mvObject_2ds.size();

            if (nAppearTimes1 > nAppearTimes2)
            {
                this->MergeTwoMapObjs_fll(obj_ass);
                this->ComputeMeanAndDeviation_3D();
                this->IsolationForestDeleteOutliers();
                obj_ass->bad_3d = true;
            }
            else
            {
                obj_ass->MergeTwoMapObjs_fll(this);
                obj_ass->ComputeMeanAndDeviation_3D();
                obj_ass->IsolationForestDeleteOutliers();
                this->bad_3d = true;
            }
        }
    }
}

bool Object_Map::DoubleSampleTtest_fll(ORB_SLAM2::Object_Map *RepeatObj) {
    // Read t-distribution boundary value.
    float tTestData[122][9] = {0};
    ifstream infile;
    std::string filePath = WORK_SPACE_PATH + "/data/t_test.txt";
    infile.open(filePath);
    for (int i = 0; i < 122; i++)
    {
        for (int j = 0; j < 9; j++)
        {
            infile >> tTestData[i][j];
        }
    }
    infile.close();

    int ndf1 = this->mvObject_2ds.size();
    float fMean1_x = this->mAveCenter3D.at<float>(0, 0);
    float fMean1_y = this->mAveCenter3D.at<float>(1, 0);
    float fMean1_z = this->mAveCenter3D.at<float>(2, 0);
    float fCenterStandar1_x = this->mCenterStandar_x;
    float fCenterStandar1_y = this->mCenterStandar_y;
    float fCenterStandar1_z = this->mCenterStandar_z;

    int ndf2 = RepeatObj->mvObject_2ds.size();
    float fMean2_x = RepeatObj->mAveCenter3D.at<float>(0, 0);
    float fMean2_y = RepeatObj->mAveCenter3D.at<float>(1, 0);
    float fMean2_z = RepeatObj->mAveCenter3D.at<float>(2, 0);
    float fCenterStandar2_x = RepeatObj->mCenterStandar_x;
    float fCenterStandar2_y = RepeatObj->mCenterStandar_y;
    float fCenterStandar2_z = RepeatObj->mCenterStandar_z;

    // Combined standard deviation.
    float d_x = sqrt( ( ( (ndf1-1)*fMean1_x*fMean1_x + (ndf2-1)*fMean2_x*fMean2_x ) / (ndf1 + ndf2 - 2) ) * (1/ndf1 + 1/ndf2) );
    float d_y = sqrt( ( ( (ndf1-1)*fMean1_y*fMean1_y + (ndf2-1)*fMean2_y*fMean2_y ) / (ndf1 + ndf2 - 2) ) * (1/ndf1 + 1/ndf2) );
    float d_z = sqrt( ( ( (ndf1-1)*fMean1_z*fMean1_z + (ndf2-1)*fMean2_z*fMean2_z ) / (ndf1 + ndf2 - 2) ) * (1/ndf1 + 1/ndf2) );

    // t-test
    float t_test_x = ( fMean1_x -fMean2_x ) / d_x;
    float t_test_y = ( fMean1_y -fMean2_y ) / d_y;
    float t_test_z = ( fMean1_z -fMean2_z ) / d_z;

    // Satisfy t test in 3 directions.
    if ((t_test_x < tTestData[min((ndf1 + ndf2 - 2), 121)][5]) &&
        (t_test_y < tTestData[min((ndf1 + ndf2 - 2), 121)][5]) &&
        (t_test_z < tTestData[min((ndf1 + ndf2 - 2), 121)][5]))
    {
        return true;
    }
    else
        return false;
}

void Object_Map::MergeTwoMapObjs_fll(Object_Map *RepeatObj)
{
    //std::cout<<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>MergeTwoMapObjs_fll 1>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"<<std::endl<<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>MergeTwoMapObjs_fll>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"<<std::endl;
 // step 1. 将RepeatObj添加到当前obj3d中
    // update points.
    for (int i = 0; i < RepeatObj->mvpMapObjectMappoints.size(); i++)
    {
        MapPoint *pMP = RepeatObj->mvpMapObjectMappoints[i];

        if(pMP->isBad())
            continue;

        cv::Mat pointPos = pMP->GetWorldPos();
        Eigen::Vector3d scale = Converter::toSE3Quat(this->mCuboid3D.pose_mat).inverse() * Converter::toVector3d(pointPos);
        if ((abs(scale[0]) > 1.1 * this->mCuboid3D.lenth / 2) ||
            (abs(scale[1]) > 1.1 * this->mCuboid3D.width / 2) ||
            (abs(scale[2]) > 1.1 * this->mCuboid3D.height / 2))
        {
            continue;
        }

        pMP->object_mnId = mnId;
        pMP->object_class = mnClass;

        map<int, int>::iterator sit;
        sit = pMP->viewdCount_forObjectId.find(pMP->object_mnId);
        // 该point被 此obj3d的次数 +1
        if (sit != pMP->viewdCount_forObjectId.end())
        {
            int sit_sec = sit->second;
            pMP->viewdCount_forObjectId.erase(pMP->object_mnId);
            pMP->viewdCount_forObjectId.insert(make_pair(pMP->object_mnId, sit_sec + 1));
        }
        else
        {
            pMP->viewdCount_forObjectId.insert(make_pair(pMP->object_mnId, 1));
        }
        {
            unique_lock<mutex> lock(mMutexMapPoints);
            bool new_point = true;
            // old points.
            for (size_t m = 0; m < mvpMapObjectMappoints.size(); ++m)
            {
                cv::Mat obj_curr_pos = pMP->GetWorldPos();
                cv::Mat obj_map_pos = mvpMapObjectMappoints[m]->GetWorldPos();

                if (cv::countNonZero(obj_curr_pos - obj_map_pos) == 0)
                {
                    mvpMapObjectMappoints[m]->feature_uvCoordinate = pMP->feature_uvCoordinate;
                    new_point = false;

                    break;
                }
            }
            // new points.
            if (new_point)
            {
                mvpMapObjectMappoints.push_back(pMP);
                mvpMapObjectMappoints_NewForActive.push_back(pMP);
                cv::Mat x3d = pMP->GetWorldPos();
                mSumPointsPos += x3d;
            }
        }
    }
    //std::cout<<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>MergeTwoMapObjs_fll 2>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"<<std::endl<<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>MergeTwoMapObjs_fll>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"<<std::endl;
    //std::cout<<"RepeatObj->mvObject_2ds.size(): "<<RepeatObj->mvObject_2ds.size()<<std::endl;

    // step 2. update frame objects.
    //int end;
    //if( RepeatObj->mvObject_2ds.size() > 10)
    //    end = RepeatObj->mvObject_2ds.size()-10;
    //else
    //    end = 0;
    for (int j=RepeatObj->mvObject_2ds.size()-1; j>=0; j--)
    {
        //Object_2D *ObjectFrame = RepeatObj->mvObject_2ds[j];
        //auto ObjectFrame = new Object_2D(RepeatObj->mvObject_2ds[j]);
        if(RepeatObj->mvObject_2ds[j]->bad)
            continue;
        //auto ObjectFrame = new Object_2D(RepeatObj->mvObject_2ds[j]);
        //Object_2D *ObjectFrame = RepeatObj->mvObject_2ds[j];
        //std::cout<<"2  ";
        //ObjectFrame = RepeatObj->mvObject_2ds[j];std::cout<<"3  ";
        //ObjectFrame->mnId = mnId;std::cout<<"4  ";
        Object_2D *ObjectFrame = RepeatObj->mvObject_2ds[j];
        ObjectFrame->mnId = mnId;
        mnConfidence_foractive++;

        AddObj2d(ObjectFrame);//this->mvObject_2ds.push_back(ObjectFrame);
    }
    //std::cout<<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>MergeTwoMapObjs_fll 3>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"<<std::endl<<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>MergeTwoMapObjs_fll>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"<<std::endl;

    // step 3. 将 RepeatObj共视关系,增加到当前obj3d中
    // the co-view relationship
    {
        map<int, int>::iterator sit;
        for (sit = RepeatObj->mmAppearSametime.begin(); sit != RepeatObj->mmAppearSametime.end(); sit++)
        {
            int nObjId = sit->first;
            int sit_sec = sit->second;

            map<int, int>::iterator sit2;
            sit2 = mmAppearSametime.find(nObjId);
            if (sit2 != mmAppearSametime.end())
            {
                int sit_sec2 = sit2->second;
                mmAppearSametime.erase(nObjId);
                mmAppearSametime.insert(make_pair(nObjId, sit_sec2 + sit_sec));
            }
            else
                mmAppearSametime.insert(make_pair(nObjId, 1));
        }
    }
    //std::cout<<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>MergeTwoMapObjs_fll 4>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"<<std::endl<<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>MergeTwoMapObjs_fll>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"<<std::endl;

    // step 4. 更新当前obj3d的最后观测帧的id
    // update the last observed frame.
    int nOriginLastAddID = mnLastAddID;
    int nOriginLastLatsAddID = mnLastLastAddID;
    cv::Rect OriginLastRect = mLastRect;
    // 如果此物体出现的更晚, 则不更新last.  查看lastlast是否需要更新
    // this object appeared recently.
    //TODO: 以下的数字都是不是多得多减1  答:不是. 因为mnLastAddID = mCurrentFrame.mnId  所以LastAddID就是最新的一个观测帧
    if (mnLastAddID > RepeatObj->mnLastAddID)
    {
        if (nOriginLastLatsAddID < RepeatObj->mnLastAddID)
        {
            mnLastLastAddID = RepeatObj->mnLastAddID;
            mLastLastRect = RepeatObj->mvObject_2ds[  RepeatObj->mvObject_2ds.size()-1  ]->mBox_cvRect;
        }
    }
    // 如果RepeatObj出现的更晚, 则更新last
    // RepeatObj appeared recently.
    else
    {
        mnLastAddID = RepeatObj->mnLastAddID;
        mLastRect = RepeatObj->mvObject_2ds[RepeatObj->mvObject_2ds.size() - 1]->mBox_cvRect;

        //查看lastlast是否要更新:
        //如果lastlast出现的更晚, 则不更新
        if (nOriginLastAddID > RepeatObj->mnLastLastAddID)
        {
            mnLastLastAddID = nOriginLastAddID;
            mLastLastRect = OriginLastRect;
        }
        //如果RepeatObj的lastlast出现的更晚, 则更新
        else if(RepeatObj->mvObject_2ds.size() >=  2 )
        {
            mnLastLastAddID = RepeatObj->mnLastLastAddID;
            //std::cout<<"[mergy debug]: size:" <<RepeatObj->mvObject_2ds.size() <<std::endl;
            //std::cout<<"[mergy debug]: width:" <<RepeatObj->mvObject_2ds[RepeatObj->mvObject_2ds.size() - 2]->mBox_cvRect.width <<std::endl;
            //std::cout<<"[mergy debug]: height:" <<RepeatObj->mvObject_2ds[RepeatObj->mvObject_2ds.size() - 2]->mBox_cvRect.height <<std::endl;
            mLastLastRect = RepeatObj->mvObject_2ds[RepeatObj->mvObject_2ds.size() - 2]->mBox_cvRect;
        }
    }
    //std::cout<<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>MergeTwoMapObjs_fll 5>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"<<std::endl<<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>MergeTwoMapObjs_fll>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"<<std::endl;

    // step 5. update direction.
    // TODO: 修改类型编号
    if (  (mnClass == 73) /*book*/ || (mnClass == 64) /*mouse*/ || (mnClass == 65)  /*remote*/
        || (mnClass == 66) /*keybord*/ || (mnClass == 56) /*chair*/      )
    {
        if(RepeatObj->mvAngleTimesAndScore.size() > 0)
        {
            for (auto &row_repeat : RepeatObj->mvAngleTimesAndScore)
            {
                bool new_measure = true;

                if(this->mvAngleTimesAndScore.size() > 0)
                {
                    for (auto &row_this : this->mvAngleTimesAndScore)
                    {
                        if(row_repeat[0] == row_this[0])
                        {
                            row_this[1] += row_repeat[1];

                            row_this[2] = row_this[2] * ((row_this[1] - row_repeat[1]) / row_this[1]) +
                                        row_repeat[2] * (row_repeat[1] / row_this[1]);

                            row_this[3] = row_this[3] * ((row_this[1] - row_repeat[1]) / row_this[1]) +
                                        row_repeat[3] * (row_repeat[1] / row_this[1]);

                            row_this[4] = row_this[4] * ((row_this[1] - row_repeat[1]) / row_this[1]) +
                                        row_repeat[4] * (row_repeat[1] / row_this[1]);

                            new_measure = false;
                            break;
                        }
                    }
                }

                if(new_measure == true)
                {
                    this->mvAngleTimesAndScore.push_back(row_repeat);
                }
            }
        }

        if(this->mvAngleTimesAndScore.size() > 0)
        {
            int best_num = 0;
            float best_score = 0.0;
            for(int i = 0; i < min(6, (int)this->mvAngleTimesAndScore.size()); i++)
            {
                float fScore = this->mvAngleTimesAndScore[i][2];
                if(fScore > best_score)
                {
                    best_score = fScore;
                    best_num = i;
                }
            }

            this->mCuboid3D.rotY = this->mvAngleTimesAndScore[best_num][0];
            this->mCuboid3D.mfErrorParallel = this->mvAngleTimesAndScore[best_num][3];
            this->mCuboid3D.mfErroeYaw = this->mvAngleTimesAndScore[best_num][4];
            this->Update_Twobj();
        }
    }
        //std::cout<<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>MergeTwoMapObjs_fll 6>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"<<std::endl<<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>MergeTwoMapObjs_fll>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"<<std::endl;

}


// 融合两个重叠的物体. 要经过iou、volume、AppearSametime、yolo class， 四关考验
void Object_Map::DealTwoOverlapObjs_fll(ORB_SLAM2::Object_Map *OverlapObj, float overlap_x, float overlap_y, float overlap_z) {
    bool bIou = false;       // false: Iou is large.
    bool bVolumeDiss = false;    // false: small volume difference.
    bool bSame_time = false; // false: doesn't simultaneous appearance.
    bool bClass = false;     // false: different classes.

    //两个物体的体积
    float fThis_obj_volume = (mCuboid3D.lenth * mCuboid3D.width) * mCuboid3D.height;
    float fOverlap_obj_volume = (OverlapObj->mCuboid3D.lenth * OverlapObj->mCuboid3D.width) * OverlapObj->mCuboid3D.height;

    // 体积Iou,是否大于0.3
    float overlap_volume = (overlap_x * overlap_y) * overlap_z;
    if ((overlap_volume / (fThis_obj_volume + fOverlap_obj_volume - overlap_volume)) >= 0.3)
        bIou = true;
    else
        bIou = false;

    //体积差异是否过大, 体积相差超过两倍,
    // compute the volume difference
    if ((fThis_obj_volume > 2 * fOverlap_obj_volume) || (fOverlap_obj_volume > 2 * fThis_obj_volume))
        bVolumeDiss = true;
    else
        bVolumeDiss = false;

    // 如果两个物体同时出现的次数大于3,
    // whether simultaneous appearance.
    map<int, int>::iterator sit;
    sit = mmAppearSametime.find(OverlapObj->mnId);
    if (sit != mmAppearSametime.end())
    {
        if (sit->second > 3)
            bSame_time = true;
        else
            bSame_time = false;
    }
    else
        bSame_time = false;

    // class 是否相同
    if (mnClass == OverlapObj->mnClass)
        bClass = true;
    else
        bClass = false;


    // case 1: IOU is large, the volume difference is small, doesn't simultaneous appearance, same class --> the same object, merge them.
    // iou大, 体积接近, 很少同时出现, class相同,  则认为是同一个物体
    // 此种情况下，将两个物体融合
    if ((bIou == true) && (bVolumeDiss == false) && (bSame_time == false) && (bClass == true))
    {
        if (this->mvObject_2ds.size() >= OverlapObj->mvObject_2ds.size())
        {
            this->MergeTwoMapObjs_fll(OverlapObj);
            OverlapObj->bad_3d = true;
        }
        else
        {
            OverlapObj->MergeTwoMapObjs_fll(this);
            this->bad_3d = true;
        }
    }

    // case 2: may be a false detection.
    // 此种情况下，将错误识别的物体删除。
    // zhangjiadong： 原程序将体积较小的删除？？？我认为不合理，改为将观测较少的物体删除
    else if ((bVolumeDiss == true) && (bSame_time == false) && (bClass == true))
    {
        if ((this->mvObject_2ds.size() >= OverlapObj->mvObject_2ds.size()) /*&& (fThis_obj_volume > fOverlap_obj_volume)*/ )
            OverlapObj->bad_3d = true;
        else if ((this->mvObject_2ds.size() < OverlapObj->mvObject_2ds.size()) /*&& (fThis_obj_volume < fOverlap_obj_volume)*/ )
            this->bad_3d = true;
    }
    else if ((bVolumeDiss == true) && (bClass == false))
    {
        if ((this->mvObject_2ds.size() >= OverlapObj->mvObject_2ds.size()) /*&& (fThis_obj_volume > fOverlap_obj_volume)*/ )
            OverlapObj->bad_3d = true;
        else if ((this->mvObject_2ds.size() < OverlapObj->mvObject_2ds.size()) /*&& (fThis_obj_volume < fOverlap_obj_volume)*/ )
            this->bad_3d = true;
    }

    // case 3: divide the overlap area of two objects equally.  (No significant effect.)
    // 此种情况下，将重叠部分均分
    else if ((bIou == true) && (bVolumeDiss == false) && (bSame_time == true) && (bClass == true))
    {
        this->DivideEquallyTwoObjs_fll(OverlapObj, overlap_x, overlap_y, overlap_z);
        OverlapObj->DivideEquallyTwoObjs_fll(OverlapObj, overlap_x, overlap_y, overlap_z);

        this->ComputeMeanAndDeviation_3D();
        OverlapObj->ComputeMeanAndDeviation_3D();
    }

    // case 4: big one gets smaller, the smaller one stays the same. (No significant effect.)
    // 重叠小, 体积相差大,多次同时出现,class不同.  说明是大物体,体积太大的了,挤占了小物体的空间
    // 此种情况下，缩小 大物体的体积
    else if ((bIou == false) && (bVolumeDiss == true) && (bSame_time == true) && (bClass == false))
    {
        if (fThis_obj_volume > fOverlap_obj_volume)
            this->BigToSmall_fll(OverlapObj, overlap_x, overlap_y, overlap_z);
        else if (fThis_obj_volume < fOverlap_obj_volume)
            OverlapObj->BigToSmall_fll(this, overlap_x, overlap_y, overlap_z);
    }

    // case 5:
    // 以上情况都处理完了, 剩下里面,如果还有, iou大, 多次同时出现, class相同的情况, 认为是同一个物体,进行融合
    else if((bIou == true) && (bSame_time == false) && (bClass == true))
    {
        if (this->mvObject_2ds.size()/2 >= OverlapObj->mvObject_2ds.size())
        {
            this->MergeTwoMapObjs_fll(OverlapObj);
            OverlapObj->bad_3d = true;
        }
        else if(OverlapObj->mvObject_2ds.size()/2 >= this->mvObject_2ds.size())
        {
            OverlapObj->MergeTwoMapObjs_fll(this);
            this->bad_3d = true;
        }
    }
    // TODO: case ...... and so on ......
}

// big one gets smaller, the smaller one stays the same. (No significant effect! Almost no use.)
void Object_Map::BigToSmall_fll(Object_Map *SmallObj, float overlap_x, float overlap_y, float overlap_z)
{
    // in which direction does the overlap occur.
    bool bxMin = false;
    bool bxMax = false;
    bool byMin = false;
    bool byMax = false;
    bool bzMin = false;
    bool bzMax = false;

    // x_min
    if ((SmallObj->mCuboid3D.x_min > this->mCuboid3D.x_min) && (SmallObj->mCuboid3D.x_min < this->mCuboid3D.x_max))
        bxMin = true;
    // x_max
    if ((SmallObj->mCuboid3D.x_max > this->mCuboid3D.x_min) && (SmallObj->mCuboid3D.x_max < this->mCuboid3D.x_max))
        bxMax = true;
    // y_min
    if ((SmallObj->mCuboid3D.y_min > this->mCuboid3D.y_min) && (SmallObj->mCuboid3D.y_min < this->mCuboid3D.y_max))
        byMin = true;
    // y_max
    if ((SmallObj->mCuboid3D.y_max > this->mCuboid3D.y_min) && (SmallObj->mCuboid3D.y_max < this->mCuboid3D.y_max))
        byMax = true;
    // z_min
    if ((SmallObj->mCuboid3D.z_min > this->mCuboid3D.z_min) && (SmallObj->mCuboid3D.z_min < this->mCuboid3D.z_max))
        bzMin = true;
    // z_max
    if ((SmallObj->mCuboid3D.z_max > this->mCuboid3D.z_min) && (SmallObj->mCuboid3D.z_max < this->mCuboid3D.z_max))
        bzMax = true;

    // false: one direction，ture: two directions.
    bool bx = false;
    bool by = false;
    bool bz = false;
    // x
    if ((bxMin = true) && (bxMax = true))
        bx = true;
    else
        bx = false;
    // y
    if ((byMin = true) && (byMax = true))
        by = true;
    else
        by = false;
    // z
    if ((bzMin = true) && (bzMax = true))
        bz = true;
    else
        bz = false;

    // Which direction to eliminate?
    int nFlag; // 0:x   1:y   2:z   3: surround

    // x
    if ((bx == false) && (by == true) && (bz == true))
        nFlag = 0;
    // y
    if ((bx == true) && (by == false) && (bz == true))
        nFlag = 1;
    // z
    if ((bx == true) && (by == true) && (bz == false))
        nFlag = 2;

    if ((bx == false) && (by == false) && (bz == true))
    {
        if (min(overlap_x, overlap_y) == overlap_x)
            nFlag = 0;
        else if (min(overlap_x, overlap_y) == overlap_y)
            nFlag = 1;
    }

    if ((bx == false) && (by == true) && (bz == false))
    {
        if (min(overlap_x, overlap_z) == overlap_x)
            nFlag = 0;
        else if (min(overlap_x, overlap_z) == overlap_z)
            nFlag = 2;
    }

    if ((bx == true) && (by == false) && (bz == false))
    {
        if (min(overlap_y, overlap_z) == overlap_y)
            nFlag = 1;
        else if (min(overlap_y, overlap_z) == overlap_z)
            nFlag = 2;
    }

    //     7------6
    //    /|     /|
    //   / |    / |
    //  4------5  |
    //  |  3---|--2
    //  | /    | /
    //  0------1
    {
        unique_lock<mutex> lock(mMutexMapPoints); // lock.

        // remove points in the overlap volume.
        vector<MapPoint *>::iterator pMP;
        for (pMP = mvpMapObjectMappoints.begin();
             pMP != mvpMapObjectMappoints.end();)
        {
            cv::Mat PointPosWorld = (*pMP)->GetWorldPos();

            // points in the smaller object.
            if ((PointPosWorld.at<float>(0) > SmallObj->mCuboid3D.x_min) && (PointPosWorld.at<float>(0) < SmallObj->mCuboid3D.x_max) &&
                (PointPosWorld.at<float>(1) > SmallObj->mCuboid3D.y_min) && (PointPosWorld.at<float>(1) < SmallObj->mCuboid3D.y_max) &&
                (PointPosWorld.at<float>(2) > SmallObj->mCuboid3D.z_min) && (PointPosWorld.at<float>(2) < SmallObj->mCuboid3D.z_max))
                pMP = mvpMapObjectMappoints.erase(pMP);
            else
            {
                ++pMP;
            }
        }
    }

    this->ComputeMeanAndDeviation_3D();
}


// BRIEF Divide the overlap area of two objects equally.  (No significant effect! Almost no use.)
void Object_Map::DivideEquallyTwoObjs_fll(Object_Map *AnotherObj, float overlap_x, float overlap_y, float overlap_z)
{
    {
        unique_lock<mutex> lock(mMutexMapPoints);

        vector<MapPoint *>::iterator pMP;
        for (pMP = mvpMapObjectMappoints.begin();
             pMP != mvpMapObjectMappoints.end();)
        {
            cv::Mat PointPosWorld = (*pMP)->GetWorldPos();
            float cuboidCenter0_ano = (AnotherObj->mCuboid3D.corner_2[0] + AnotherObj->mCuboid3D.corner_8[0])/2.0;
            float cuboidCenter1_ano = (AnotherObj->mCuboid3D.corner_2[1] + AnotherObj->mCuboid3D.corner_8[1]) / 2.0;
            float cuboidCenter2_ano = (AnotherObj->mCuboid3D.corner_2[2] + AnotherObj->mCuboid3D.corner_8[2])/2.0 ;
            if (((PointPosWorld.at<float>(0) > cuboidCenter0_ano - (AnotherObj->mCuboid3D.lenth / 2 - overlap_x / 2)) &&
                 (PointPosWorld.at<float>(0) < cuboidCenter0_ano + (AnotherObj->mCuboid3D.lenth / 2 - overlap_x / 2))) &&
                ((PointPosWorld.at<float>(1) > cuboidCenter1_ano - (AnotherObj->mCuboid3D.width / 2 - overlap_y / 2)) &&
                 (PointPosWorld.at<float>(1) < cuboidCenter1_ano + (AnotherObj->mCuboid3D.width / 2 - overlap_y / 2))) &&
                ((PointPosWorld.at<float>(2) > cuboidCenter2_ano - (AnotherObj->mCuboid3D.height / 2 - overlap_z / 2)) &&
                 (PointPosWorld.at<float>(2) < cuboidCenter2_ano + (AnotherObj->mCuboid3D.height / 2 - overlap_z / 2))))
            {
                pMP = mvpMapObjectMappoints.erase(pMP);
            }

            else
            {
                ++pMP;
            }
        }
    }
}




// ************************************
// object3d 信息熵计算部分 *
// ************************************

Object_Map::Object_Map() {
    cv::FileStorage fSettings( WORK_SPACE_PATH + "/config/" +yamlfile_object, cv::FileStorage::READ);
    if(!fSettings.isOpened())
	{
		cerr<<"Failed to open settings file at: "<<WORK_SPACE_PATH+ "/config/"+yamlfile_object<<endl;
	}
	//else cout<<"success to open file at: "<<WORK_SPACE_PATH+yamlfile_object<<endl;

    mIE_rows = fSettings["IE.rows"];
    mIE_cols = fSettings["IE.cols"];
    //std::cout<<"IE_RecoverInit:2  "
    //            <<", " << mIE_rows
    //            <<", " << mIE_cols
    //            <<std::endl;
    mP_occ = fSettings["IE.P_occ"];
    mP_free = fSettings["IE.P_free"];
    mP_prior = fSettings["IE.P_prior"];
    mIEThresholdPointNum = fSettings["IE.ThresholdPointNum"];
    IE_RecoverInit();

    mIForest_thresh = fSettings["IForest.Threshold"];
    publisher_IE = nh.advertise<visualization_msgs::Marker>("object_ie", 1000);
    mbPublishIEwheel = fSettings["IE.PublishIEwheel"];
    mIEThresholdEndMapping =  fSettings["IE.ThresholdEndMapping"];
    mnViewField =  fSettings["ViewField"];
}

double Object_Map::IE(const double &p){
    return -1*p*log2(p) - (1-p)*log2(1-p);
}

// 重置每个grid的: point数量,信息熵
void Object_Map::IE_RecoverInit() {
        //mCuboid3D = object->mCuboid3D;
        //mvpMapObjectMappoints = object->mvpMapObjectMappoints;

        //18*18个grid中 点的数量
        //vector<int> PointNum_OneColu(mIE_rows, 0.0) ;
        //vector<vector<int> >  PointNum(mIE_cols, PointNum_OneColu);
        //mvPointNum = PointNum;
        mvPointNum_mat = cv::Mat::zeros(mIE_rows,mIE_cols,CV_32F);

        //18*18个grid的 占据概率
        //vector<double> GridProb_OneColu(mIE_rows, mP_prior) ;
        //vector<vector<double> > GridProb(mIE_cols, GridProb_OneColu);
        //mvGridProb = GridProb;
        mvGridProb_mat = cv::Mat::ones(mIE_rows,mIE_cols,CV_32F);
        mvGridProb_mat *= mP_prior;

        //18*18个grid中 熵的大小(熵最大是0.693147)
        //vector<double> InforEntroy_OneColu(mIE_rows, IE(mP_prior) ) ;
        //vector<vector<double> > InforEntroy(mIE_cols, InforEntroy_OneColu);
        //mvInforEntroy = InforEntroy;
        mvInforEntroy_mat = cv::Mat::ones(mIE_rows,mIE_cols,CV_32F);
        mvInforEntroy_mat *= IE(mP_prior);
    }


//
void Object_Map::compute_grid_xy(const Eigen::Vector3d &zero_vec, const Eigen::Vector3d &point_vec, int& x, int& y){
    //假设所有"物体都位于水平面"

    //zero_vec是 起点轴.
    //point_vec 是中心指向point的连线
    Eigen::Vector3d v1(zero_vec(0),zero_vec(1),0.0), v2(point_vec(0),point_vec(1),0.0);
    double cosValNew = v1.dot(v2) / (v1.norm()*v2.norm()); //通过向量的点乘, 计算角度cos值
    double angleNew = acos(cosValNew) * 180 / M_PI;     //弧度角
    //acos()的正常范围是0~180度.
    // 因此如果point_vec的y值大于0, 则直接采用angleNew
    // 反之, 角度等于 180+(180-angleNew) = 360-angleNew

    //version1:
    //if(point_vec(1) >= 0)
    //    x = floor(angleNew/(360.0/mIE_cols));
    //else
    //    x = floor((360.0-angleNew)/(360.0/mIE_cols));

    //version2: 如果pointvec逆时针旋转到zerovec，则修正夹角
    if (v2.cross(v1)(2) > 0) {
        angleNew = 360 - angleNew;
    }
    x = floor(angleNew/(360.0/mIE_cols));

    y = floor(
                (  (point_vec(2)  + mCuboid3D.height/2 ) /mCuboid3D.height) * mIE_rows
            );
    //std::cout<<"[计算y]"  <<point_vec(2)   <<", 中心z " <<  mCuboid3D.cuboidCenter[2] <<",  cube高度 "<< mCuboid3D.height/2 <<std::endl;
}

// 计算每个grid的点数
cv::Mat Object_Map::compute_pointnum_eachgrid(){
    float cuboidCenter0 = (mCuboid3D.corner_2[0] + mCuboid3D.corner_8[0])/2.0;
    float cuboidCenter1 = (mCuboid3D.corner_2[1] + mCuboid3D.corner_8[1]) / 2.0;
    float cuboidCenter2 = (mCuboid3D.corner_2[2] + mCuboid3D.corner_8[2])/2.0 ;
    double center_x = cuboidCenter0;
    double center_y = cuboidCenter1;
    double center_z = cuboidCenter2;
    //object在世界的位姿
    cv::Mat T_w_o_mat = mCuboid3D.pose_mat;
    Eigen::Isometry3d T_w_o = ORB_SLAM2::Converter::toSE3Quat(T_w_o_mat);
    //物体坐标系下, 指向x轴的向量
    Eigen::Vector3d zero_vec( 1,0,0);
    zero_vec = T_w_o* zero_vec;

    for (int i = 0; i < mvpMapObjectMappoints.size(); ++i) {
        cv::Mat point_pose = mvpMapObjectMappoints[i]->GetWorldPos();
        Eigen::Vector3d point_vec( point_pose.at<float>(0)-center_x, point_pose.at<float>(1)-center_y, point_pose.at<float>(2)-center_z);
        int x = -1 , y = -1;
        compute_grid_xy(zero_vec, point_vec, x, y);
        if( x>=0 && x<=mIE_cols && y>=0 && y<=mIE_rows ) {
            int temp = mvPointNum_mat.at<float>(x,y);
            //std::cout<<"compute_pointnum_eachgrid1: " << temp <<std::endl;
            mvPointNum_mat.at<float>(x,y) = temp+1;
            //std::cout<<"compute_pointnum_eachgrid2: " << mvPointNum_mat.at<float>(x,y) <<std::endl;
        }
        else{
            std::cout<<"compute grid index: ERROR:i "<<i<<", x "<<x<<", y "<<y<<std::endl;
        }
    }

    return mvPointNum_mat;
}

//首先记录每个栅格被投影到的所有点，然后查看每个点被当前物体观察到的次数，记录下最大的次数。没有点的栅格，记录同一列的栅格的平均值。
void Object_Map::compute_perceptionNum_eachgrid() {
    float cuboidCenter0 = (mCuboid3D.corner_2[0] + mCuboid3D.corner_8[0])/2.0;
    float cuboidCenter1 = (mCuboid3D.corner_2[1] + mCuboid3D.corner_8[1]) / 2.0;
    float cuboidCenter2 = (mCuboid3D.corner_2[2] + mCuboid3D.corner_8[2])/2.0 ;
    double center_x = cuboidCenter0;
    double center_y = cuboidCenter1;
    double center_z = cuboidCenter2;

    //object在世界的位姿
    cv::Mat T_w_o_mat = mCuboid3D.pose_mat;
    Eigen::Isometry3d T_w_o = ORB_SLAM2::Converter::toSE3Quat(T_w_o_mat);
    //物体坐标系下, 指向x轴的向量
    Eigen::Vector3d zero_vec( 1,0,0);
    zero_vec = T_w_o* zero_vec;

    //更新有point的栅格的数值
    for (auto pMP : mvpMapObjectMappoints) {
        cv::Mat point_pose = pMP->GetWorldPos();
        Eigen::Vector3d point_vec( point_pose.at<float>(0)-center_x, point_pose.at<float>(1)-center_y, point_pose.at<float>(2)-center_z);
        int x = -1 , y = -1;
        compute_grid_xy(zero_vec, point_vec, x, y);
        if( x>=0 && x<=mIE_cols && y>=0 && y<=mIE_rows ) {
            map<int, int>::iterator sit;
            int viewdCount;
            sit = pMP->viewdCount_forObjectId.find(this->mnId);
            if (sit != pMP->viewdCount_forObjectId.end())
            {
                viewdCount = sit->second;
            }
            else
            {
                ROS_ERROR("IE Error: cant find point viewd Count for ObjectId");
            }

            //将此栅格上point的最大viewCount记录下来
            if(viewdCount>mvPointNum_mat.at<float>(x,y))
                mvPointNum_mat.at<float>(x,y) = viewdCount;
        }
        else{
            //ROS_ERROR("compute grid xy < 0,%d,%d",x,y);
        }
    }


}

//计算每个grid的占据概率
float Object_Map::log2(float x){
    float y = std::log(x) / std::log(2.0);
    return y;
}
void Object_Map::compute_occupied_prob_eachgrid(){
    //std::cout<<"debug 每个grid的point数量与占据概率：";
    for(int x=0; x<mIE_rows; x++){

        //计算这一行的点的总数
        int num_onecol = 0;
        int num_nozero = 0;
        for(int y=0; y<mIE_cols; y++){
            num_onecol +=  mvPointNum_mat.at<float>(x,y);
            if( mvPointNum_mat.at<float>(x,y) != 0 )
                num_nozero ++;
        }


        //总数大于阈值，才认为管材的有效
        if(num_onecol > mIEThresholdPointNum){
            //记录此列的point的平均值,用于没有point投影的栅格
            int ave = ceil(num_onecol/ num_nozero);

            //当前列的观测到认为有效，即当前列的grid，认为是free或occupied
            for(int y=0; y<mIE_rows; y++){
                double l_lnv ;
                double l_last = log2(   (1-log2(mvGridProb_mat.at<float>(x,y)))
                                  /  log2(mvGridProb_mat.at<float>(x,y))
                                 );
                double l_last_fake = 0.0;
                int PointNum = mvPointNum_mat.at<float>(x,y);
                int ObserveNum = mvObject_2ds.size();
                //if(ObserveNum==0) ObserveNum=40;//todo:  这是因为在融合的时候,有一步没把mvObject_2ds融合
                if(PointNum == 0){
                    PointNum = ave;

                    //free
                    //todo: 当前只更新一次，之后对物体内的point进行“是否为新添加的更新”，再进行增量更新

                    //version1: 通过观测的次数，来进行更新概率最接近雷达建图的方式，每次观测都更新栅格的概率。点云的作用在于构建了一个用于判断观测类型（free,占据，未知）的视觉物体建模模型，类似于雷达的传感器模型。
                    // 而且观测次数ObserveNum应该等于1，因为
                    //l_lnv = ObserveNum * log( mP_free/ (1.0 - mP_free));

                    //version2： 根据每个格子的点的数量更新栅格概率。 我觉得问题在于，点可能永远不会变动，因为点是地图中真实存在的，数量有限。
                    //l_lnv = l_last +  mNewPointNum * log( mP_free/ (1.0 - mP_free));

                    //version3: 根据格子里面的point“被当前物体观测到”的次数。
                    l_lnv = PointNum * log2( mP_free/ (1.0 - mP_free));
                }
                else{
                    //occupied
                    //todo: 当前只更新一次，之后对物体内的point进行“是否为新添加的更新”，再进行增量更新
                    //lnv_p = log(mP_prior) + ObserveNum * (log(mP_occ) - log(mP_prior));
                    l_lnv = PointNum * log2(mP_occ / (1.0 - mP_occ));
                }
                //mvGridProb_mat.at<float>(x,y) = exp(lnv_p);
                double bel = 1.0 - 1.0 / (1.0 + exp(l_lnv));
                if(bel>0.99)  bel=0.99;
                if(bel<0.01)  bel=0.01;
                mvGridProb_mat.at<float>(x,y) = (float) bel;
                //std::cout << mvPointNum_mat.at<float>(x,y) << "(" << mvGridProb_mat.at<float>(x,y) << "," << ObserveNum << "," << l_lnv << ")， ";
            }
        }
        else{
             //当前列的观测到认为无效，即当前列的grid，认为是unknown
             for(int y=0; y<mIE_rows; y++){
                 //unkonwn
                 mvGridProb_mat.at<float>(x,y) = mP_prior;
                 //std::cout<<mvPointNum_mat.at<float>(x,y)<<"("<<mvGridProb_mat.at<float>(x,y)<<")， ";
             }
        }
    }
    //std::cout<<"   "<<std::endl;
}


//计算每个grid的信息熵
void Object_Map::ComputeIE(){
    IE_RecoverInit();

    // 计算每个grid的点数
    //compute_pointnum_eachgrid();
    compute_perceptionNum_eachgrid();

    //计算每个grid的占据概率
    compute_occupied_prob_eachgrid();

    //计算各grid的信息熵
    //std::cout<<"debug 每个grid的占据概率：";
    for(int x=0; x<mIE_cols; x++)
        for(int y=0; y<mIE_rows; y++){
            //std::cout<<mvGridProb_mat.at<float>(x,y)<<"， ";
            mvInforEntroy_mat.at<float>(x,y) = IE(mvGridProb_mat.at<float>(x,y));
        }
    //std::cout<<""<<std::endl;

    //计算总的信息熵mIE
    double entroy = 0;
    for(int x=0; x<mIE_cols; x++)
        for(int y=0; y<mIE_rows; y++){
            entroy += mvInforEntroy_mat.at<float>(x,y);
        }

    mIE = entroy/(mIE_cols*mIE_rows);

    //记录栅格的状态??
}


void Object_Map::PublishIE(){
    // color.
    //std::vector<vector<float> > colors_bgr{ {135,0,248},  {255,0,253},  {4,254,119},  {255,126,1},  {0,112,255},  {0,250,250}   };
    //vector<float> color;

    //生成 rviz marker
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.ns = "InformationEntroy";
    marker.lifetime = ros::Duration(0.5);
    marker.id= mnId;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.scale.x=0.03;
    marker.scale.y=0.08;
    marker.pose.orientation.w=1.0;  //????
    marker.action=visualization_msgs::Marker::ADD;


    //for(size_t i=0; i< vObjs.size(); i++)
    {

        if((this->mvpMapObjectMappoints.size() < 10) || (this->bad_3d == true)  || this->backgroud_object)
        {
            return;
        }

        //color = colors_bgr[obj->mnClass % 6];
        double diameter = sqrt(mCuboid3D.width * mCuboid3D.width   +   mCuboid3D.lenth * mCuboid3D.lenth )/2.0;
        for(int x=0; x<mIE_rows; x++){
            double angle_divide = 2*M_PI/mIE_rows;
            double angle = angle_divide * ( x + 0.5 );
            double p_x = cos(angle) * diameter;
            double p_y = sin(angle) * diameter;

            double h_divide =  mCuboid3D.height/mIE_cols;
            for(int y=0; y<mIE_cols; y++){
                //计算纵坐标
                double p_z = h_divide * (y+0.5) - mCuboid3D.height/2.0;

                // 物体坐标系 -> 世界坐标系
                cv::Mat cvMat4 = mCuboid3D.pose_mat.clone();
                Eigen::Matrix4f eigenMat4f;
                cv::cv2eigen(cvMat4, eigenMat4f);
                //Eigen::Matrix4d T = ORB_SLAM2::Converter::cvMattoMatrix4d(obj->mCuboid3D.pose_mat);
                Eigen::Matrix4d T = eigenMat4f.cast<double>();
                Eigen::Matrix3d R = T.block<3, 3>(0, 0);
                Eigen::Vector3d p_world = R * Eigen::Vector3d(p_x, p_y, p_z);
                geometry_msgs::Point p;
                p.x= p_world[0] + T(0, 3);
                p.y= p_world[1] + T(1, 3);
                p.z= p_world[2] + T(2, 3);

                if(mvGridProb_mat.at<float>(x,y) > 0.5){
                    //marker.color.r =1.0; marker.color.g = 1.0; marker.color.b = 1.0; marker.color.a = 1.0;
                    //marker.color.r =color[2]/255.0; marker.color.g = color[1]/255.0; marker.color.b = color[0]/255.0; marker.color.a = 0.7;
                    //marker.color.r =255.0; marker.color.g = 255.0; marker.color.b = 255.0; marker.color.a = 0.7;
                    marker.color.r =0.0; marker.color.g = 0.0; marker.color.b = 0.0; marker.color.a = 0.7;
                }
                else if(mvGridProb_mat.at<float>(x,y) < 0.5){
                    //marker.color.r =0.0; marker.color.g = 0.0; marker.color.b = 0.0; marker.color.a = 1.0;
                    //marker.color.r =color[2]/255.0; marker.color.g = color[1]/255.0; marker.color.b = color[0]/255.0; marker.color.a = 0.15;
                    marker.color.r =255.0; marker.color.g = 255.0; marker.color.b = 255.0; marker.color.a = 0.7;
                }
                else {
                    marker.color.r =1.0; marker.color.g = 1.0; marker.color.b = 1.0; marker.color.a = 0.2;
                }

                marker.points.push_back(p);
                //usleep(100);
            }
        }
    }
    publisher_IE.publish(marker);
}

//计算物体的主向量, 注意:这是在world坐标系下描述的
void Object_Map::ComputeMainDirection(){

    double main_x, main_y, main_z;
    for (int i = 0; i < mvpMapObjectMappoints.size(); ++i) {
        cv::Mat point_pose = mvpMapObjectMappoints[i]->GetWorldPos();
        main_x +=  point_pose.at<float>(0) - this->mCuboid3D.cuboidCenter(0) ;
        main_y +=  point_pose.at<float>(1) - this->mCuboid3D.cuboidCenter(1) ;
        main_z +=  point_pose.at<float>(2) - this->mCuboid3D.cuboidCenter(2) ;
    }
    double normalize = sqrt( main_x*main_x + main_y*main_y + main_z*main_z );
    main_x = main_x/normalize;
    main_y = main_y/normalize;
    main_z = 0.0;//main_z/normalize;
    mMainDirection =  Eigen::Vector3d(main_x, main_y, main_z).normalized();
}

double Object_Map::get_information_entroy(){
    return mIE;
}



// ************************************
// object3d 筛选候选点 *
// ************************************
bool Object_Map::WheatherInRectFrameOf(const cv::Mat &Twc, const float &fx, const float &fy, const float &cx, const float &cy, const float &ImageWidth, const float &ImageHeight)
{
    //(1) 判断是否在物体检测框内
    //const cv::Mat Rcw = Tcw.rowRange(0, 3).colRange(0, 3);
    //const cv::Mat tcw = Tcw.rowRange(0, 3).col(3);
    //vector<float> x_pt;
    //vector<float> y_pt;
    //for (int j = 0; j < mvpMapObjectMappoints.size(); j++)
    //{
    //    MapPoint *pMP = mvpMapObjectMappoints[j];
    //    cv::Mat PointPosWorld = pMP->GetWorldPos();
    //
    //    cv::Mat PointPosCamera = Rcw * PointPosWorld + tcw;
    //
    //    const float xc = PointPosCamera.at<float>(0);
    //    const float yc = PointPosCamera.at<float>(1);
    //    const float invzc = 1.0 / PointPosCamera.at<float>(2);
    //
    //    float u = fx * xc * invzc + cx;
    //    float v = fy * yc * invzc + cy;
    //
    //    x_pt.push_back(u);
    //    y_pt.push_back(v);
    //
    //}
    //
    //if (x_pt.size() == 0)
    //    return false;
    //
    //sort(x_pt.begin(), x_pt.end());
    //sort(y_pt.begin(), y_pt.end());
    //float x_min = x_pt[0];
    //float x_max = x_pt[x_pt.size() - 1];
    //float y_min = y_pt[0];
    //float y_max = y_pt[y_pt.size() - 1];
    //

    ////Camera.width: 640
    ////Camera.height: 480
    //if (x_min < 0)
    //    return false;
    //if (y_min < 0)
    //    return false;
    //if (x_max > ImageWidth)
    //    return false;
    //if (y_max > ImageHeight)
    //    return false;


    //(2)相机的中心与物体的距离
    double dist =  sqrt(   (mCuboid3D.cuboidCenter(0)-Twc.at<float>(0,3)) * (mCuboid3D.cuboidCenter(0)-Twc.at<float>(0,3))
                        +  (mCuboid3D.cuboidCenter(1)-Twc.at<float>(1,3)) * (mCuboid3D.cuboidCenter(1)-Twc.at<float>(1,3))
                        );
    if(dist<mnViewField)
        return true;
}


BackgroudObject::BackgroudObject() {
    mPlane = PointCloud::Ptr(new PointCloud());
}
BackgroudObject::~BackgroudObject()  {
    mPlane.reset();
}

bool BackgroudObject::include(Object_Map* fo){
    ////前景物体的中心
    //Eigen::Vector3d  fo_centor;
    //fo_centor = Eigen::Vector3d(fo->mCuboid3D.cuboidCenter.x(), fo->mCuboid3D.cuboidCenter.y(), fo->mCuboid3D.cuboidCenter.z());

    //背景物体的中心
    cv::Mat T_w_bo = this->pose_mat;
    cv::Mat T_bo_w = T_w_bo.inv();

    // 将点从世界坐标系变换到椭球体坐标系下（即通过椭球体的位姿将点变换到与椭球体同一坐标系）
    Eigen::Vector4d centor_w = Eigen::Vector4d(fo->mCuboid3D.cuboidCenter.x(), fo->mCuboid3D.cuboidCenter.y(), fo->mCuboid3D.cuboidCenter.z(), 1.0);   //real_to_homo_coord<double>(fo_centor);
    Eigen::Vector4d centor_bo = Converter::cvMattoMatrix4d(T_bo_w) * centor_w;

    double x = std::abs(centor_bo[0]);
    double y = std::abs(centor_bo[1]);
    double z = std::abs(centor_bo[2]);
    //std::cout<<"[debug IOU,x]:" <<x<<std::endl;
    //std::cout<<"[debug IOU,y]:" <<y<<std::endl;
    //std::cout<<"[debug IOU,z]:" <<z<<std::endl;
    // 将点在各个坐标轴上的坐标与椭球体在各个坐标轴上的半径进行比较，若点在三个坐标轴上的坐标都小于椭球体在各个坐标轴上的半径，则返回true，否则返回false。
    if(x <this->length/2.0*1.5 && y < this->width/2.0*1.5 )
        return true;
    else{
        //std::cout<<"[include]: x:"<<x <<" length:"<<this->length <<", y:"<<y <<" width:"<<this->width << std::endl;
        return false;
    }
}

bool BackgroudObject::AllInclude(vector<Object_Map*> fos){
    //前景物体的中心
    for(auto fo: fos){
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
        if(x <this->length/2.0 && y < this->width/2.0 )
            mvFOs.push_back(fo);
    }
}

void BackgroudObject::IncludeFOs_and_WheatherEndActive(const std::vector<Object_Map*> &FOs) {
    //1. 统计属于此BO的FO
    mvFOs.clear();
    FO_num = 0;
    FO_num_not_end = 0;
    for(int i=0;i<FOs.size(); i++){
        Object_Map* fo = FOs[i];
        if(fo->bad_3d){
            continue;
        }

        //如果还有前景物体没有完成，则将 end_.. 改回false
        if( this->include(fo) )
        {
            mvFOs.push_back(fo);
            FO_num ++;
            if(!fo->end_build){
                //mState = UnEnd;
                //this->end_activemapping = false;
                FO_num_not_end ++;
            }
        }
    }
    //std::cout<<"[IncludeFOs_and_WheatherEndActive]:"<<FO_num <<"/"<<FOs.size()<<std::endl;

    //2.判断BO的状态
    //判断最大观测次数，防止陷入局部最优值
    if(mnObserveNum < mnObserveMaxNum){
        //如果背景物体内部一个物体也没有了，则认为是未探索状态。仍需要生成GNBV。
        if(FO_num==0){
            mState = UnExplored;
        }
        //如果背景物体内部存在多个物体了
        else{
            //如果所有物体都建立完了，则认为是 End
            if(FO_num_not_end==0)
                mState = End;
            //如果还有物体没有建立完了，则认为是 UnEnd
            else
                mState = UnEnd;
        }
    }
    //如果BO达到了最大观测次数，则不再进行观测。并将输入此BO的FO，都停止更新。
    else{

        mState = End;
        for(int i=0; i<mvFOs.size(); i++){
            mvFOs[i]->end_build = true;
        }
    }
}

bool BackgroudObject::return_end_ASLAM()
{
    if(mState==End)
        return true;
    else
        return false;
}

void BackgroudObject::computePose( double yaw ) {
    float cp = cos(0.0);
    float sp = sin(0.0);
    float sr = sin(0.0);
    float cr = cos(0.0);
    float sy = sin(yaw);
    float cy = cos(yaw);
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