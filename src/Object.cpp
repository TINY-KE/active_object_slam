//
// Created by zhjd on 11/8/22.
//

#include "Object.h"
#include "Converter.h"
namespace ORB_SLAM2
{
mutex Object_2D::mGlobalMutex;  //crash bug

Object_2D::Object_2D() {

}

Object_2D::Object_2D(Map* Map, Frame* CurrentFrame, const BoxSE &box) {
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
    // original box format.
    mBox_yoloSE = box;

    this->mpCurrentFrame = CurrentFrame;
    this->mpMap = Map;

    //初始化位姿
    this->sum_pos_3d = cv::Mat::zeros(3, 1, CV_32F);
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

bool Object_2D::Object2D_DataAssociationWith_everyObject3D()  //cv::Mat &image
{
    const cv::Mat image = mpCurrentFrame->mColorImage.clone();
    const cv::Mat Rcw = cv::Mat::zeros(3,3,CV_32F);
    const cv::Mat tcw = cv::Mat::eye(3,1,CV_32F);  //深拷贝
    mpCurrentFrame->mTcw.rowRange(0, 3).colRange(0, 3).copyTo(Rcw);
    mpCurrentFrame->mTcw.rowRange(0, 3).col(3).copyTo(tcw);

    cv::Rect RectCurrent = mBox_cvRect;    // object bounding box in current frame.
    cv::Rect RectPredict;               // predicted bounding box according to last frame and next to last frame.
    cv::Rect RectProject;               // bounding box constructed by projecting points.

    //const vector<Object_Map*> ObjectMaps  = mpMap->GetObjects();
    //// ****************************************************
    ////         STEP 1.  IoU  association.              *
    //// ****************************************************
    //float IouMax = 0;                   //
    //bool bAssoByIou = false;            // 默认禁用. 通过后续IOU匹配的结果, 再决定是否要启动IOU
    //int AssoObjId_byIou = -1;              // the associated map object ID.
    //int ObjID_IouMax = -1;               // temporary variable. 代表map中与this->object_2d, 最IOU匹配的object_map
    //float IouThreshold = 0.5;           // IoU threshold.
    ////if((flag != "NA") && (flag != "NP"))
    //{
    //
    //    for (int i = 0; i < (int)ObjectMaps.size(); i++)
    //    {
    //        Object_Map* obj3d = ObjectMaps[i];
    //        if (mclass_id != obj3d->mnClass)
    //            continue;
    //        if (obj3d->bBadErase)
    //            continue;
    //        if ((mpCurrentFrame->mnId-1) == obj3d->mnLastAddID )   //如果是临近的两帧， 则可以用IOU关联。
    //        {
    //            // step 1.1 predict object bounding box according to last frame and next to last frame.
    //            if ((mpCurrentFrame->mnId-2) == obj3d->mnLastLastAddID)
    //            {   //以下基于的假设: 临近三帧,相同物体的检测框,在图像上的移动是相同的
    //                // 0____ll___l____c
    //                // c = l - ll + l = 2 * l - ll
    //                // left-top.
    //                float left_top_x = obj3d->mLastRect.x * 2 - obj3d->mLastLastRect.x;     // cv::Rect的x代表 方形的左上角的x坐标
    //                if (left_top_x < 0)
    //                    left_top_x = 0;
    //                float left_top_y = obj3d->mLastRect.y * 2 - obj3d->mLastLastRect.y;     // cv::Rect的y代表 方形的左上角的y坐标
    //                if (left_top_y < 0)
    //                    left_top_y = 0;
    //                // right-bottom.
    //                float right_down_x = (obj3d->mLastRect.x + obj3d->mLastRect.width) * 2 - (obj3d->mLastLastRect.x + obj3d->mLastLastRect.width);
    //                if (left_top_x > image.cols)
    //                    right_down_x = image.cols;
    //                float right_down_y = (obj3d->mLastRect.y + obj3d->mLastRect.height) * 2 - (obj3d->mLastLastRect.y + obj3d->mLastLastRect.height);
    //                if (left_top_y > image.rows)
    //                    right_down_y = image.rows;
    //
    //                float width = right_down_x - left_top_x;
    //                float height = right_down_y - left_top_y;
    //
    //                // predicted bounding box.
    //                RectPredict = cv::Rect(left_top_x, left_top_y, width, height);
    //
    //                // If two consecutive frames are observed, increase the threshold.
    //                IouThreshold = 0.6;
    //            }
    //            else
    //                RectPredict = obj3d->mLastRect;    //如果不是临近三帧, 则认为物体检测框和上一帧相同
    //
    //            // step 1.2 compute IoU, record the max IoU and the map object ID.
    //            float Iou = Converter::bboxOverlapratio(RectCurrent, RectPredict);
    //            if ((Iou > IouThreshold) && (Iou > IouMax))
    //            {
    //                IouMax = Iou;
    //                ObjID_IouMax = i;
    //            }
    //        }
    //    }// 对map中的object遍历完毕,找到IOU匹配最佳的物体"IouMaxObjID"
    //
    //    // step 1.3 if the association is successful, update the map object.
    //    Object_Map* obj3d_IouMax = ObjectMaps[ObjID_IouMax];
    //    if ((IouMax > 0) && (ObjID_IouMax >= 0))
    //    {
    //        // 更新object3d 中的特征点
    //        bool bFlag = obj3d_IouMax->UpdateObject3D(this, image, Iou);
    //        if (bFlag)
    //        {
    //            bAssoByIou = true;              // associated by IoU.
    //            AssoObjId_byIou = ObjID_IouMax;     // associated map object id.
    //        }
    //    }
    //}
    //// Iou data association END ----------------------------------------------------------------------------
    //
    //// *************************************************
    ////      STEP 2. Nonparametric data association     *
    //// *************************************************
    //bool bAssoByNp = false;
    //int  AssoObjId_byNP  = -1; //;nAssoByNPId
    //vector<int> vAssoObjIds_byNP;     // potential associated objects.
    ////if((flag != "NA") && (flag != "IoU"))
    //{
    //    // 遍历vObjectMap中每一个物体实例，与自身的2D目标框做NP
    //    for (int i = (int)ObjectMaps.size() - 1; (i >= 0) ; i--)
    //    {
    //        Object_Map* obj3d = ObjectMaps[i];
    //        if (mclass_id != obj3d->mnClass)
    //            continue;
    //        if (obj3d->bBadErase)
    //            continue;
    //
    //        // step 2.1 nonparametric test.
    //        int NoParaFlag = this->NoParaDataAssociation(obj3d, image);  //TODO: NoParaDataAssociation是怎么计算和obj3d的关联关系的??
    //
    //        if (NoParaFlag == 0) // 0: skip the nonparametric test and continue with the subsequent t-test.
    //            break;
    //        if (NoParaFlag == 2) // 2: association failed, compare next object.
    //            continue;
    //        else if (NoParaFlag == 1) // 1: association succeeded, but there may be more than one.
    //            vAssoObjIds_byNP.push_back(i);
    //    }// 对map中的object遍历完毕,找到NoPara匹配合适的所有物体"vObjByNPId"
    //
    //    // step 2.2 update data association and record potential associated objects.
    //    if (vAssoObjIds_byNP.size() >= 1)
    //    {
    //        // case 1: if associated by IoU, the objects judged by nonparametric-test are marked as potential association objects.
    //        if (bAssoByIou)
    //        {
    //            Object_Map* obj3d_IOU = ObjectMaps[AssoObjId_byIou];
    //            // 遍历NP的匹配，是否与IOU判断一致
    //            for (int i = 0; i < vAssoObjIds_byNP.size(); i++)
    //            {
    //                int AssoObjId_byNP_temp = vAssoObjIds_byNP[i];
    //
    //                //如果NP和IOU找到的物体相同, 则不作处理
    //                if (AssoObjId_byNP_temp == AssoObjId_byIou)
    //                    continue;
    //
    //                //如果不同, 则在mReObj中, 记录NP潜在的链接关系. 之后可用于 TODO:其他部分                                   Record potential association objects and potential association times.
    //                Object_Map* obj3d_NP = ObjectMaps[AssoObjId_byNP_temp];
    //                map<int, int>::iterator sit;  sit = obj3d_IOU->mReObj.find(obj3d_NP->mnId);
    //                if (sit != obj3d_IOU->mReObj.end())
    //                {
    //                    int sit_sec = sit->second;
    //                    obj3d_IOU->mReObj.erase(obj3d_NP->mnId);
    //                    obj3d_IOU->mReObj.insert(make_pair(obj3d_NP->mnId, sit_sec + 1));
    //                }
    //                else
    //                    obj3d_IOU->mReObj.insert(make_pair(obj3d_NP->mnId, 1));
    //            }
    //        }
    //
    //        // case 2: if association failed by IoU,
    //        // notes: 如果没有IOU重合，则把NP的结果代入DataAssociateUpdate进行检查
    //        // 疑问：多对多怎么处理？即：vObjByNPId  vAssoObjIds_byNP 中有多个返回bFlag==true。不过好像没有影响
    //        // 答: 似乎只会处理第一个成功的, 就break出去了.
    //        else
    //        {
    //            for (int i = 0; i < vAssoObjIds_byNP.size(); i++)
    //            {
    //                // update.
    //                int AssoObjId_byNP_temp = vAssoObjIds_byNP[i];
    //                bool bFlag = ObjectMaps[AssoObjId_byNP_temp]->UpdateObject3D(this, image, NoPara);
    //
    //                // if association successful, other objects are marked as potential association objects.
    //                if (bFlag)
    //                {
    //                    bAssoByNp = true;               // associated by NP.
    //                    AssoObjId_byNP = AssoObjId_byNP_temp;    // associated map object id.
    //
    //                    if (vAssoObjIds_byNP.size() > i + 1)  //TODO: 这个判断没有意义. 因此如果不是最后一个, 那么下面必然要执行. 如果是最后一个,那么也不用循环了,之后的break也没有意义
    //                    {
    //                        for (int j = i + 1; j < vAssoObjIds_byNP.size(); j++)
    //                        {
    //                            // Record potential association objects and potential association times.
    //                            map<int, int>::iterator sit;
    //                            int AssoObjId_byNP_2 = vAssoObjIds_byNP[j];
    //                            sit = ObjectMaps[AssoObjId_byNP]->mReObj.find(  ObjectMaps[AssoObjId_byNP_2]->mnId  );
    //                            if (sit != ObjectMaps[AssoObjId_byNP]->mReObj.end())
    //                            {
    //                                int sit_sec = sit->second;
    //                                ObjectMaps[AssoObjId_byNP]->mReObj.erase(ObjectMaps[AssoObjId_byNP_2]->mnId);
    //                                ObjectMaps[AssoObjId_byNP]->mReObj.insert(make_pair(ObjectMaps[AssoObjId_byNP_2]->mnId, sit_sec + 1));
    //                            }
    //                            else
    //                                ObjectMaps[AssoObjId_byNP]->mReObj.insert(make_pair(ObjectMaps[AssoObjId_byNP_2]->mnId, 1));
    //                        }
    //                        break;
    //                    }
    //                }
    //            }
    //        }
    //    }
    //}
    //// Nonparametric data association END --------------------------------------------------------------------------------------------------------
    //
    //
    //
    //// ****************************************************
    ////         STEP 3. Projected box data association     *
    //// ****************************************************
    //bool bAssoByProject = false;
    //int nAssoByProId = -1;
    //vector<int> vObjByProIouId;
    //
    //if((flag != "NA") && (flag != "IoU") && (flag != "NP"))
    //{
    //    // 获得最大IOU的编号和值，记录部分bIoU大于0.25的
    //    float fIouMax = 0.0;
    //    int ProIouMaxObjId = -1;
    //    for (int i = (int)mpMap->mvObjectMap.size() - 1; i >= 0; i--)
    //    {
    //        if (_class_id != mpMap->mvObjectMap[i]->mnClass)
    //            continue;
    //
    //        if (mpMap->mvObjectMap[i]->bBadErase)
    //            continue;
    //        // notes： 小样本和多物体下使用
    //        int df = (int)mpMap->mvObjectMap[i]->mObject_2ds_current.size();
    //
    //        if ((Obj_c_MapPonits.size() >= 10) && (df > 8))
    //            continue;
    //
    //        // step 3.1 compute IoU with bounding box constructed by projecting points.
    //        // notes: mRectFeaturePoints 为物体确定的最大包围框，RectCurrent为YOLOX确定的包围框
    //        float fIou = Converter::bboxOverlapratio(RectCurrent, mpMap->mvObjectMap[i]->mRectProject);
    //        float fIou2 = Converter::bboxOverlapratio(mRectFeaturePoints, mpMap->mvObjectMap[i]->mRectProject);
    //        fIou = max(fIou, fIou2);
    //
    //        // record the max IoU and map object id.
    //        // notes: 找到最大重叠目标框
    //        if ((fIou >= 0.25) && (fIou > fIouMax))
    //        {
    //            fIouMax = fIou;
    //            ProIouMaxObjId = i;
    //            // 这种记录方式是不是有点不科学，毕竟这个不是排序过的
    //            vObjByProIouId.push_back(i);
    //        }
    //    }
    //    // step 3.2 update data association and record potential associated objects.
    //    if (fIouMax >= 0.25)
    //    {
    //        sort(vObjByProIouId.begin(), vObjByProIouId.end());
    //
    //        if (bAssoByIou || bAssoByNp)
    //        {
    //            for (int j = vObjByProIouId.size() - 1; j >= 0; j--)
    //            {
    //                int ReId;
    //                if (bAssoByIou)
    //                    ReId = AssoObjId_byIou;
    //                if (bAssoByNp)
    //                    ReId = nAssoByNPId;
    //
    //                if (vObjByProIouId[j] == ReId)
    //                    continue;
    //
    //                map<int, int>::iterator sit;
    //                sit = mpMap->mvObjectMap[ReId]->mReObj.find(mpMap->mvObjectMap[vObjByProIouId[j]]->mnId);
    //                if (sit != mpMap->mvObjectMap[ReId]->mReObj.end())
    //                {
    //                    int sit_sec = sit->second;
    //                    mpMap->mvObjectMap[ReId]->mReObj.erase(mpMap->mvObjectMap[vObjByProIouId[j]]->mnId);
    //                    mpMap->mvObjectMap[ReId]->mReObj.insert(make_pair(mpMap->mvObjectMap[vObjByProIouId[j]]->mnId, sit_sec + 1));
    //                }
    //                else
    //                    mpMap->mvObjectMap[ReId]->mReObj.insert(make_pair(mpMap->mvObjectMap[vObjByProIouId[j]]->mnId, 1));
    //            }
    //        }
    //        else
    //        {
    //            // update.
    //            bool bFlag = mpMap->mvObjectMap[ProIouMaxObjId]->DataAssociateUpdate_forobj2d(this, mCurrentFrame,
    //                                                                                          image, 4); // 4: project iou.
    //
    //            // association succeeded.
    //            if (bFlag)
    //            {
    //                bAssoByProject = true;          // associated by projecting box.
    //                nAssoByProId = ProIouMaxObjId;  // associated map object id.
    //            }
    //
    //            for (int j = vObjByProIouId.size() - 1; j >= 0; j--)
    //            {
    //                if (vObjByProIouId[j] == ProIouMaxObjId)
    //                    continue;
    //
    //                map<int, int>::iterator sit;
    //                sit = mpMap->mvObjectMap[ProIouMaxObjId]->mReObj.find(mpMap->mvObjectMap[vObjByProIouId[j]]->mnId);
    //                if (sit != mpMap->mvObjectMap[ProIouMaxObjId]->mReObj.end())
    //                {
    //                    int sit_sec = sit->second;
    //                    mpMap->mvObjectMap[ProIouMaxObjId]->mReObj.erase(mpMap->mvObjectMap[vObjByProIouId[j]]->mnId);
    //                    mpMap->mvObjectMap[ProIouMaxObjId]->mReObj.insert(make_pair(mpMap->mvObjectMap[vObjByProIouId[j]]->mnId, sit_sec + 1));
    //                }
    //                else
    //                    mpMap->mvObjectMap[ProIouMaxObjId]->mReObj.insert(make_pair(mpMap->mvObjectMap[vObjByProIouId[j]]->mnId, 1));
    //            }
    //        }
    //    }
    //}
    //// Projected box data association END ---------------------------------------------------------------------------------------
    //
    //// ************************************************
    ////          STEP 4. t-test data association       *
    //// ************************************************
    //// step 4.1 Read t-distribution boundary value.
    //float tTestData[122][9] = {0};
    //ifstream infile;
    //std::string filePath = WORK_SPACE_PATH + "/data/t_test.txt";
    //infile.open(filePath);
    //for (int i = 0; i < 122; i++)
    //{
    //    for (int j = 0; j < 9; j++)
    //    {
    //        infile >> tTestData[i][j];
    //    }
    //}
    //infile.close();
    //
    //// step 4.2 t-test.
    //bool bAssoByT = false;
    //int nAssoByTId = -1;
    //vector<int> vObjByTId;
    //vector<int> vObjByTIdLower; // potential association.
    //if((flag != "NA") && (flag != "IoU") && (flag != "NP"))
    //{
    //    for (int i = (int)mpMap->mvObjectMap.size() - 1; i >= 0; i--)
    //    {
    //        if (_class_id != mpMap->mvObjectMap[i]->mnClass)
    //            continue;
    //
    //        if (mpMap->mvObjectMap[i]->bBadErase)
    //            continue;
    //
    //        // t-test results in 3 directions.
    //        float t_test;
    //        float t_test_x, t_test_y, t_test_z;
    //
    //        // Degrees of freedom.
    //        int df = (int)mpMap->mvObjectMap[i]->mObject_2ds_current.size();
    //        // 场景复杂下才启用
    //        if (df <= 8)
    //            continue;
    //
    //        // Iou.
    //        float fIou = Converter::bboxOverlapratio(RectCurrent, mpMap->mvObjectMap[i]->mRectProject);
    //        float fIou2 = Converter::bboxOverlapratio(mRectFeaturePoints, mpMap->mvObjectMap[i]->mRectProject);
    //        fIou = max(fIou, fIou2);
    //
    //        // The distance from points to the object center.
    //        float dis_x, dis_y, dis_z;
    //        cv::Mat pos_points = GetWorldPos();
    //        dis_x = abs(mpMap->mvObjectMap[i]->mCenter3D.at<float>(0, 0) - pos_points.at<float>(0, 0));
    //        dis_y = abs(mpMap->mvObjectMap[i]->mCenter3D.at<float>(1, 0) - pos_points.at<float>(1, 0));
    //        dis_z = abs(mpMap->mvObjectMap[i]->mCenter3D.at<float>(2, 0) - pos_points.at<float>(2, 0));
    //
    //        // t-test.
    //        // notes: 对应论文中公式5
    //        t_test_x = dis_x / (mpMap->mvObjectMap[i]->mCenterStandar_x / sqrt(df));
    //        t_test_y = dis_y / (mpMap->mvObjectMap[i]->mCenterStandar_y / sqrt(df));
    //        t_test_z = dis_z / (mpMap->mvObjectMap[i]->mCenterStandar_z / sqrt(df));
    //
    //        // Satisfy t test.  // 5->0.05.
    //        // notes: curr_t_test < t_{n-1, \alpha /2} 详见t test单样本双侧临界表
    //        if ((t_test_x < tTestData[min((df - 1), 121)][5]) &&
    //            (t_test_y < tTestData[min((df - 1), 121)][5]) &&
    //            (t_test_z < tTestData[min((df - 1), 121)][5]))
    //        {
    //            vObjByTId.push_back(i);
    //        }
    //        // If the T-test is not satisfied, but the IOU is large, reducing the significance.
    //        else if (fIou > 0.25)
    //        {
    //            // 显著性降低为0.001，容许值更大一些
    //            if ((t_test_x < tTestData[min((df - 1), 121)][8]) &&
    //                (t_test_y < tTestData[min((df - 1), 121)][8]) &&
    //                (t_test_z < tTestData[min((df - 1), 121)][8]))
    //            {
    //                vObjByTId.push_back(i);
    //            }
    //
    //            else if ((fIou > 0.25) && ((t_test_x + t_test_y + t_test_z) / 3 < 10))
    //            {
    //                vObjByTId.push_back(i);
    //            }
    //            else
    //            {
    //                vObjByTIdLower.push_back(i);
    //            }
    //        }
    //        else if ((t_test_x + t_test_y + t_test_z) / 3 < 4)
    //        {
    //            mpMap->mvObjectMap[i]->ComputeProjectRectFrame(image, mCurrentFrame);
    //
    //            float fIou_force = Converter::bboxOverlapratio(RectCurrent, mpMap->mvObjectMap[i]->mRectProject);
    //            float fIou2_force = Converter::bboxOverlapratio(mRectFeaturePoints, mpMap->mvObjectMap[i]->mRectProject);
    //            fIou_force = max(fIou_force, fIou2_force);
    //
    //            if (fIou_force > 0.25)
    //                vObjByTIdLower.push_back(i);
    //        }
    //    }
    //
    //    // step 4.2 update data association and record potential associated objects.
    //    if (bAssoByIou || bAssoByNp || bAssoByProject)
    //    {
    //        int ReId;
    //        if (bAssoByIou)
    //            ReId = AssoObjId_byIou;
    //        if (bAssoByNp)
    //            ReId = nAssoByNPId;
    //        if (bAssoByProject)
    //            ReId = nAssoByProId;
    //        // 疑问： vObjByTId和vObjByTIdLower有什么区别？
    //        if (vObjByTId.size() >= 1)
    //        {
    //            for (int j = 0; j < vObjByTId.size(); j++)
    //            {
    //                if (vObjByTId[j] == ReId)
    //                    continue;
    //
    //                map<int, int>::iterator sit;
    //                sit = mpMap->mvObjectMap[ReId]->mReObj.find(mpMap->mvObjectMap[vObjByTId[j]]->mnId);
    //                if (sit != mpMap->mvObjectMap[ReId]->mReObj.end())
    //                {
    //                    int sit_sec = sit->second;
    //                    mpMap->mvObjectMap[ReId]->mReObj.erase(mpMap->mvObjectMap[vObjByTId[j]]->mnId);
    //                    mpMap->mvObjectMap[ReId]->mReObj.insert(make_pair(mpMap->mvObjectMap[vObjByTId[j]]->mnId, sit_sec + 1));
    //                }
    //                else
    //                    mpMap->mvObjectMap[ReId]->mReObj.insert(make_pair(mpMap->mvObjectMap[vObjByTId[j]]->mnId, 1));
    //            }
    //        }
    //
    //        if (vObjByTIdLower.size() >= 0)
    //        {
    //            for (int j = 0; j < vObjByTIdLower.size(); j++)
    //            {
    //                if (vObjByTIdLower[j] == ReId)
    //                    continue;
    //
    //                map<int, int>::iterator sit;
    //                sit = mpMap->mvObjectMap[ReId]->mReObj.find(mpMap->mvObjectMap[vObjByTIdLower[j]]->mnId);
    //                if (sit != mpMap->mvObjectMap[ReId]->mReObj.end())
    //                {
    //                    int sit_sec = sit->second;
    //                    mpMap->mvObjectMap[ReId]->mReObj.erase(mpMap->mvObjectMap[vObjByTIdLower[j]]->mnId);
    //                    mpMap->mvObjectMap[ReId]->mReObj.insert(make_pair(mpMap->mvObjectMap[vObjByTIdLower[j]]->mnId, sit_sec + 1));
    //                }
    //                else
    //                    mpMap->mvObjectMap[ReId]->mReObj.insert(make_pair(mpMap->mvObjectMap[vObjByTIdLower[j]]->mnId, 1));
    //            }
    //        }
    //    }
    //    else
    //    {
    //        if (vObjByTId.size() >= 1)
    //        {
    //            for (int i = 0; i < vObjByTId.size(); i++)
    //            {
    //                bool bFlag = mpMap->mvObjectMap[vObjByTId[i]]->DataAssociateUpdate_forobj2d(this, mCurrentFrame,
    //                                                                                            image, 3); // 3 是指 T 方法.
    //
    //                if (bFlag)
    //                {
    //                    bAssoByT = true;
    //                    nAssoByTId = vObjByTId[i];
    //
    //                    if (vObjByTId.size() > i)
    //                    {
    //                        for (int j = i + 1; j < vObjByTId.size(); j++)
    //                        {
    //                            map<int, int>::iterator sit;
    //                            sit = mpMap->mvObjectMap[nAssoByTId]->mReObj.find(mpMap->mvObjectMap[vObjByTId[j]]->mnId);
    //                            if (sit != mpMap->mvObjectMap[nAssoByTId]->mReObj.end())
    //                            {
    //                                int sit_sec = sit->second;
    //                                mpMap->mvObjectMap[nAssoByTId]->mReObj.erase(mpMap->mvObjectMap[vObjByTId[j]]->mnId);
    //                                mpMap->mvObjectMap[nAssoByTId]->mReObj.insert(make_pair(mpMap->mvObjectMap[vObjByTId[j]]->mnId, sit_sec + 1));
    //                            }
    //                            else
    //                                mpMap->mvObjectMap[nAssoByTId]->mReObj.insert(make_pair(mpMap->mvObjectMap[vObjByTId[j]]->mnId, 1));
    //                        }
    //                    }
    //
    //                    if (vObjByTIdLower.size() >= 0)
    //                    {
    //                        for (int j = 0; j < vObjByTIdLower.size(); j++)
    //                        {
    //                            if (vObjByTIdLower[j] == nAssoByTId)
    //                                continue;
    //
    //                            map<int, int>::iterator sit;
    //                            sit = mpMap->mvObjectMap[nAssoByTId]->mReObj.find(mpMap->mvObjectMap[vObjByTIdLower[j]]->mnId);
    //                            if (sit != mpMap->mvObjectMap[nAssoByTId]->mReObj.end())
    //                            {
    //                                int sit_sec = sit->second;
    //                                mpMap->mvObjectMap[nAssoByTId]->mReObj.erase(mpMap->mvObjectMap[vObjByTIdLower[j]]->mnId);
    //                                mpMap->mvObjectMap[nAssoByTId]->mReObj.insert(make_pair(mpMap->mvObjectMap[vObjByTIdLower[j]]->mnId, sit_sec + 1));
    //                            }
    //                            else
    //                                mpMap->mvObjectMap[nAssoByTId]->mReObj.insert(make_pair(mpMap->mvObjectMap[vObjByTIdLower[j]]->mnId, 1));
    //                        }
    //                    }
    //
    //                    break;
    //                }
    //            }
    //        }
    //    }
    //}
    //// t-test data association END ---------------------------------------------------------------------------------------

    // *************************************************
    //             STEP 4. create a new object         *
    // *************************************************
    //if (bAssoByIou || bAssoByNp || bAssoByProject || bAssoByT)
        return false;
}

int Object_2D::creatObject()
{
    unique_lock<mutex> lock(mMutexObjMapPoints);   //目的: 不让mBox_cvRect被修改, 以免造成程序错乱.
    const cv::Mat ColorImage = mpCurrentFrame->mColorImage.clone();
    bool associate = Object2D_DataAssociationWith_everyObject3D();    // data association.
    if(associate)
        return 0;  //关联成功

    // If the object appears at the edge of the image, ignore.
    if ((this->mBox_yoloSE.x < 10) || (this->mBox_yoloSE.y < 10) ||
        (this->mBox_yoloSE.x + this->mBox_yoloSE.width > ColorImage.cols - 10) ||
        (this->mBox_yoloSE.y + this->mBox_yoloSE.height > ColorImage.rows - 10))
    {
        this->bad = true;
        return -1;  //检测框 位置不好
    }

    // create a 3d object in the map.
    std::cout<<"【debug】开始创建新物体"<<std::endl;
    Object_Map *Object3D = new Object_Map;  //zhang 生成新物体
    Object3D->mvObject_2ds.push_back(this);
    const vector<Object_Map*> ObjectMaps  = mpMap->GetObjects();
    Object3D->mnId = ObjectMaps.size();
    Object3D->mnClass = mclass_id;
    Object3D->mnConfidence_foractive = 1;
    Object3D->mnAddedID = mpCurrentFrame->mnId;
    Object3D->mnLastAddID = mpCurrentFrame->mnId;
    Object3D->mnLastLastAddID = mpCurrentFrame->mnId;
    Object3D->mLastRect = mBox_cvRect;
    //Object3D->mPredictRect = obj->mBoxRect;       // for iou.
    std::cout<<"【debug】新物体 初始化"<<std::endl;
    // add properties of the point and save it to the object.
    for (size_t i = 0; i < mvMapPonits.size(); i++)
    {
        MapPoint *pMP = mvMapPonits[i];
        pMP->object_mnId = Object3D->mnId;
        pMP->object_class = Object3D->mnClass;
        pMP->viewdCount_forObjectId.insert(make_pair(Object3D->mnId, 1));  //记录point被某个object3d看见的次数

        // save to the object.
        Object3D->mvpMapObjectMappoints.push_back(pMP);
    }
    std::cout<<"【debug】新物体 复制特征点"<<std::endl;

    mnId = Object3D->mnId;

    // save this 2d object to current frame (associates with a 3d object in the map).
    mpCurrentFrame->mvObject_2ds.push_back(this);
    //mpCurrentFrame->AppearNewObject = true;
    std::cout<<"【debug】新物体 存入obj2d"<<std::endl;

    // update object map.
    Object3D->IsolationForestDeleteOutliers();
    std::cout<<"【debug】新物体 isolation"<<std::endl;
    Object3D->ComputeMeanAndDeviation_3D();
    std::cout<<"【debug】新物体 计算均值"<<std::endl;
    //mpMap->mvObjectMap.push_back(ObjectMapSingle);
    mpMap->AddObject(Object3D);
    std::cout<<"【debug】新物体 存入map"<<std::endl;

    return 1;  //创建物体成功
}

//  nonparametric test.
int Object_2D::NoParaDataAssociation(Object_Map *Object3D,  cv::Mat &image)
{
    //// step 1. sample size.
    //// 2d object ponits in the frame -- m.
    //int m = (int)Obj_c_MapPonits.size();
    //int OutPointNum1 = 0;
    //for (int ii = 0; ii < (int)Obj_c_MapPonits.size(); ii++)
    //{
    //    MapPoint *p1 = Obj_c_MapPonits[ii];
    //    if (p1->isBad() || p1->out_point)
    //    {
    //        OutPointNum1++;
    //        continue;
    //    }
    //}
    //m = m - OutPointNum1;
    //
    //// 3d object points in the object map -- n.
    //int n = (int)ObjectMapSingle->mvpMapObjectMappoints.size();
    //int OutPointNum2 = 0;
    //for (int ii = 0; ii < (int)ObjectMapSingle->mvpMapObjectMappoints.size(); ii++) // 帧中物体.
    //{
    //    MapPoint *p2 = ObjectMapSingle->mvpMapObjectMappoints[ii];
    //    if (p2->isBad() || p2->out_point)
    //    {
    //        OutPointNum2++;
    //        continue;
    //    }
    //}
    //n = n - OutPointNum2;
    //
    //// 0: skip the nonparametric test and continue with the subsequent t-test.
    //if (m < 20)
    //    return 0;
    //
    //// 2: association failed, compare next object.
    //if (n < 20)
    //    return 2;
    //
    //// Homogenization to avoid too many points of map object; n = 3 * m.
    //// 均匀化防止地图点过多
    //bool bSampleMapPoints = true;
    //vector<float> x_pt_map_sample;
    //vector<float> y_pt_map_sample;
    //vector<float> z_pt_map_sample;
    //if (bSampleMapPoints)
    //{
    //    int step = 1;
    //    if (n > 3 * m)
    //    {
    //        n = 3 * m;
    //        step = (int)ObjectMapSingle->mvpMapObjectMappoints.size() / n;
    //
    //        vector<float> x_pt;
    //        vector<float> y_pt;
    //        vector<float> z_pt;
    //        for (int jj = 0; jj < (int)ObjectMapSingle->mvpMapObjectMappoints.size(); jj++)
    //        {
    //            MapPoint *p2 = ObjectMapSingle->mvpMapObjectMappoints[jj];
    //            if (p2->isBad() || p2->out_point)
    //            {
    //                continue;
    //            }
    //
    //            cv::Mat x3D2 = p2->GetWorldPos();
    //            x_pt.push_back(x3D2.at<float>(0, 0));
    //            y_pt.push_back(x3D2.at<float>(1, 0));
    //            z_pt.push_back(x3D2.at<float>(2, 0));
    //        }
    //        // 注意：这里是分开维度进行排序，相当于把点拆解了
    //        sort(x_pt.begin(), x_pt.end());
    //        sort(y_pt.begin(), y_pt.end());
    //        sort(z_pt.begin(), z_pt.end());
    //
    //        for (int i = 0; i < x_pt.size(); i += step)
    //        {
    //            x_pt_map_sample.push_back(x_pt[i]);
    //            y_pt_map_sample.push_back(y_pt[i]);
    //            z_pt_map_sample.push_back(z_pt[i]);
    //        }
    //        n = x_pt_map_sample.size();
    //    }
    //    else
    //    {
    //        for (int jj = 0; jj < (int)ObjectMapSingle->mvpMapObjectMappoints.size(); jj++) // 地图中物体.
    //        {
    //            MapPoint *p2 = ObjectMapSingle->mvpMapObjectMappoints[jj];
    //            if (p2->isBad() || p2->out_point)
    //            {
    //                continue;
    //            }
    //
    //            cv::Mat x3D2 = p2->GetWorldPos();
    //            x_pt_map_sample.push_back(x3D2.at<float>(0, 0));
    //            y_pt_map_sample.push_back(x3D2.at<float>(1, 0));
    //            z_pt_map_sample.push_back(x3D2.at<float>(2, 0));
    //        }
    //
    //        n = x_pt_map_sample.size();
    //    }
    //}
    //
    //float w_x_12 = 0.0;
    //float w_y_12 = 0.0;
    //float w_z_12 = 0.0;
    //float w_x_21 = 0.0;
    //float w_y_21 = 0.0;
    //float w_z_21 = 0.0;
    //float w_x_00 = 0.0;
    //float w_y_00 = 0.0;
    //float w_z_00 = 0.0;
    //float w_x = 0.0;
    //float w_y = 0.0;
    //float w_z = 0.0;
    //
    //for (int ii = 0; ii < (int)Obj_c_MapPonits.size(); ii++)
    //{
    //    MapPoint *p1 = Obj_c_MapPonits[ii];
    //    if (p1->isBad() || p1->out_point)
    //        continue;
    //
    //    cv::Mat x3D1 = p1->GetWorldPos();
    //    double x1 = x3D1.at<float>(0, 0);
    //    double y1 = x3D1.at<float>(1, 0);
    //    double z1 = x3D1.at<float>(2, 0);
    //
    //    // TODO: 感觉这块有点问题，没有进行排序
    //    if (!bSampleMapPoints)
    //    {
    //        for (int jj = 0; jj < (int)ObjectMapSingle->mvpMapObjectMappoints.size(); jj++)
    //        {
    //            MapPoint *p2 = ObjectMapSingle->mvpMapObjectMappoints[jj];
    //            if (p2->isBad() || p2->out_point)
    //                continue;
    //
    //            cv::Mat x3D2 = p2->GetWorldPos();
    //            double x2 = x3D2.at<float>(0, 0);
    //            double y2 = x3D2.at<float>(1, 0);
    //            double z2 = x3D2.at<float>(2, 0);
    //
    //            // notes: p1为物体帧点，p2为地图上的点
    //            if (x1 > x2)
    //                w_x_12++;
    //            else if (x1 < x2)
    //                w_x_21++;
    //            else if (x1 == x2)
    //                w_x_00++;
    //
    //            if (y1 > y2)
    //                w_y_12++;
    //            else if (y1 < y2)
    //                w_y_21++;
    //            else if (y1 == y2)
    //                w_y_00++;
    //
    //            if (z1 > z2)
    //                w_z_12++;
    //            else if (z1 < z2)
    //                w_z_21++;
    //            else if (z1 == z2)
    //                w_z_00++;
    //        }
    //    }
    //
    //    if (bSampleMapPoints)
    //    {
    //        for (int jj = 0; jj < (int)x_pt_map_sample.size(); jj++)
    //        {
    //            double x2 = x_pt_map_sample[jj];
    //            double y2 = y_pt_map_sample[jj];
    //            double z2 = z_pt_map_sample[jj];
    //
    //            // 这里x_pt_map_sample实际上是排序过的，下面相当于对物体帧上的点进行排序
    //            if (x1 > x2)
    //                w_x_12++;
    //            else if (x1 < x2)
    //                w_x_21++;
    //            else if (x1 == x2)
    //                w_x_00++;
    //
    //            if (y1 > y2)
    //                w_y_12++;
    //            else if (y1 < y2)
    //                w_y_21++;
    //            else if (y1 == y2)
    //                w_y_00++;
    //
    //            if (z1 > z2)
    //                w_z_12++;
    //            else if (z1 < z2)
    //                w_z_21++;
    //            else if (z1 == z2)
    //                w_z_00++;
    //        }
    //    }
    //}
    //
    //// step 2. compute the rank sum.
    //// notes: 注意，当我们计算若干等值元素的排名时，会用这些元素排名的平均值作为它们在整个序列中的排名。
    //// 这就是为什么要加 w_x_00 / 2 的原因
    //// w_x_12 + w_x_00 / 2 为 m对应的排序； w_x_21 + w_x_00 / 2 为 n对应的排序
    //// W = min(W_p, W_q)
    //w_x = min(w_x_12 + m * (m + 1) / 2, w_x_21 + n * (n + 1) / 2) + w_x_00 / 2;
    //w_y = min(w_y_12 + m * (m + 1) / 2, w_y_21 + n * (n + 1) / 2) + w_y_00 / 2;
    //w_z = min(w_z_12 + m * (m + 1) / 2, w_z_21 + n * (n + 1) / 2) + w_z_00 / 2;
    //
    //// step 3. compute the critical value.
    //// TODO: 修改为wiki上的标准形式
    //// notes: 这里的公式其实不太对，代码中的公式为r1 = r_l = m + s * sqrt(\sigma)
    //// 其中\sigma = m * n * (m + n + 1) / 12
    //// 但是这种情况下是有重复秩的，\sigma应该用论文上的公式才对
    //// 不过论文上公式和wiki上也并不一致
    //// 给的注释即s的值好像也不太对，标准正态分布表中对应80%是0.85, 对应1.28的是90%
    //// 对应 1.96的是 97.5%
    //float r1 = 0.5 * m * (m + n + 1) - 1.282 * sqrt(m * n * (m + n + 1) / 12); // 80%：1.282  85%:1.96
    //float r2 = 0.5 * m * (m + n + 1) + 1.282 * sqrt(m * n * (m + n + 1) / 12); // 80%：1.282  85%:1.96
    //
    //// step 4. whether the 3 directions meet the nonparametric test.
    //bool old_np = false;
    //int add = 0;
    //if (w_x > r1 && w_x < r2)
    //    add++;
    //if (w_y > r1 && w_y < r2)
    //    add++;
    //if (w_z > r1 && w_z < r2)
    //    add++;
    //
    //if (add == 3)
    //    old_np = true;  // Nonparametric Association succeeded.
    //
    //if (old_np == 1)
    //    return 1;       // success.
    //else
    //    return 2;       // failure.
} // Object_2D::NoParaDataAssociation() END ------------------------------------------------------------

void Object_2D::AddObjectPoint(ORB_SLAM2::MapPoint *pMP) {
    unique_lock<mutex> lock2(mGlobalMutex);
    unique_lock<mutex> lock(mMutexPos);
    mvMapPonits.push_back(pMP);
    const cv::Mat PointPosWorld = pMP->GetWorldPos();                 // world frame.
    sum_pos_3d += PointPosWorld;  //深拷贝
}








void Object_Map::ComputeMeanAndDeviation_3D() {
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

    // object frame -> world frame (without yaw, parallel to world frame).
    g2o::SE3Quat pose_without_yaw =  Converter::toSE3Quat(this->mCuboid3D.pose_noyaw_mat);
    mCuboid3D.corner_1_w = pose_without_yaw * Eigen::Vector3d(x_min_obj, y_min_obj, z_min_obj);
    mCuboid3D.corner_2_w = pose_without_yaw * Eigen::Vector3d(x_max_obj, y_min_obj, z_min_obj);
    mCuboid3D.corner_3_w = pose_without_yaw * Eigen::Vector3d(x_max_obj, y_max_obj, z_min_obj);
    mCuboid3D.corner_4_w = pose_without_yaw * Eigen::Vector3d(x_min_obj, y_max_obj, z_min_obj);
    mCuboid3D.corner_5_w = pose_without_yaw * Eigen::Vector3d(x_min_obj, y_min_obj, z_max_obj);
    mCuboid3D.corner_6_w = pose_without_yaw * Eigen::Vector3d(x_max_obj, y_min_obj, z_max_obj);
    mCuboid3D.corner_7_w = pose_without_yaw * Eigen::Vector3d(x_max_obj, y_max_obj, z_max_obj);
    mCuboid3D.corner_8_w = pose_without_yaw * Eigen::Vector3d(x_min_obj, y_max_obj, z_max_obj);


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

}





void Object_Map::IsolationForestDeleteOutliers(){

}

bool Object_Map::UpdateObject3D(Object_2D* Object_2d, cv::Mat &image, int Flag){

}

void Object_Map::Update_Twobj()      //更新物体在世界下的坐标
{
    unique_lock<mutex> lock(mMutex);

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
    this->mCuboid3D.pose_noyaw_mat = Twobj_without_yaw;
}

void Object_Map::ComputeProjectRectFrame(Frame &Frame)
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

    mRectProject_forDataAssociate2D = cv::Rect(x_min, y_min, x_max - x_min, y_max - y_min);
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

}