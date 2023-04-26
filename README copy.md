# Active-EAO-Fusion
**目标：试图调整 EAO-SLAM ，建立物体级active slam**

### a todo
+ 构建camera在tf中的坐标系变换   <frameName>camera_depth_optical_frame</frameName>
  + 创建camera_depth_optical_frame和odom的tf变换
+ 为什么没有baselink和camera
+ 统一gazebo和slam的坐标系
  + 已经实现
+ 为什么movebase发送的速度不行？？
+ fabo的movebase在/home/zhjd/ws_active/src/robot/fabo_robot ？？
+ 

### 0 程序重构
+ if(mpCurrentFrame.N>500)  Initialization的时候,源程序设定的是50??  我要不要跟随
+ cv::Mat Ow = -Rinv * t;   为什么这是平移的逆矩阵
+ pKFini->GetPose() 中是,相机坐标系到init世界的变换关系,  
+ fill(mpCurrentFrame.mvpMapPoints.begin(),mpCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));  直接调用了frame中的point,  把vector清空
+ 直接获取frame中的point    MapPoint *pMP = mpCurrentFrame.mvpMapPoints[i];
+ mpCurrentFrame**.mvpMap**Points[i]=static_cast<MapPoint*>(NULL); 对frame中point清空的方法
+ point和frame的匹配和调用方法
   ` for(map<KeyFrame*,size_t>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {   
        KeyFrame* pKF = mit->first;
        pKF->EraseMapPointMatch(mit->second);
    }`
+ 【【重要】】  核验sum_pos_3d -= pos;
+ 【【重要】】  Q3 = z_c[(int)(z_c.size() * 3 / 4)];
  + zc是真实world坐标系中的高度吗？？  会不会还是深度？  应该是的
+ object2d中各种操作后，要不要自行 重新计算均值和偏差
  + RemoveOutlier_ByHeightorDepth()
  + 根据物体融合
+ objectMap中有三个 eigen vector
  + 等无eigen版本运行成功后，再添加上eigen试试
+ Object2D_DataAssociation_forCreateObject中
  + 要不要把map和optimize 锁住
  + 看看算法中会不会用到点的位姿和新的point？？
+ mnLastAddID 和 mnLastLastAddID  还没赋值
+ IouThreshold = 0.6;  IOU的阈值 要不要调节
+                 if ((MotionIou > IouThreshold) && (MotionIou > IouMax))
+ poseOptimizaiton 
  + g2o调用mappoint时, 启动了(MapPoint::mGlobalMutex)的线程锁
  + Recover optimized pose时, 没有启动frame的线程锁.  是否说明修改指针对应的值是不用修改的,只有调用size时禁止新的空间开辟
+ 为什么NoParaDataAssociation 会成功多个. 拿到不能像iou一样,有个最大值
+ 疑问:if (vAssoObjIds_byNP.size() > i + 1)  //TODO: 这个判断没有意义. 因此如果不是最后一个, 那么下面必然要执行. 如果是最后一个,那么也不用循环了,之后的break也没有意义
+ std::map<int, int> object_id_vector;   // pMP->object_id_vector.insert(make_pair(Object3D->mnId, 1));  //记录point被某个object3d看见的次数
+ AppearNewObject 用于 neednewkeyframe
+ cv::Mat image = mCurrentFrame.mColorImage.clone();  //todo: 这个还没有存入frame中
+ 剔除物体检测框时， 为什么不用处理过的检测框，而是用的原检测框
+ Merges objects with 5-10 points时， 不加语义标签 判断吗？ 而且只针对特征点少的。  不能把特征点多的也处理吗？
+ Object3D->mLastRect = obj2d->mBox_cvRect;             为什么不用处理过的检测框
+ [致命问题] track中的currentframe是临时的。但是被我输入到object2d中了。
+ [必改] 如果此物体被观测到的帧数很少, 而且过去30帧都没有被看到,则设置为bad_3d
  + 结果: 物体较长时间观测不到,就踢除
  + 分析: 源程序是作为桌面建图,  而我是室内建图, 很久看不到某个物体. 因为机器人去探索其他区域了,这些地图应该
  + 通过主动slam, 能否解决这个问题
+ WhetherOverlap(obj_3ds_new[j]) 用的是距离的方法,  能不能改为IOU
+ 共视关系: 两个物体在同一帧中被观测到, 则认为两个物体被共视
+ 物体位姿的存储形式 Converter::toSE3Quat(pFrame->mTcw)
+ Eigen::Vector3d PointPos_object = this->mCuboid3D.pose.inverse() * PointPos_world;
+ mLastFrame = Frame(mCurrentFrame);  没有将彩色图像和物体图像 传递过去  会有影响吗?
+ 关闭np test中的，均匀化试一试
+ wm统计量计算的对吗? W = min(W_p, W_q)
    w_x = min(w_x_2d_bigger_3d + m * (m + 1) / 2, w_x_3d_bigger_2d + n * (n + 1) / 2) + w_x_3d_equal_2d / 2;
    w_y = min(w_y_2d_bigger_3d + m * (m + 1) / 2, w_y_3d_bigger_2d + n * (n + 1) / 2) + w_y_3d_equal_2d / 2;
    w_z = min(w_z_2d_bigger_3d + m * (m + 1) / 2, w_z_3d_bigger_2d + n * (n + 1) / 2) + w_z_3d_equal_2d / 2;
  + 存储潜在关联对象的操作， 统一为一个函数. mReObj
+ AddPotentialAssociatedObjects(ObjectMaps, AssoObjId_byIou  ,AssoObjId_byNP);   为什么只将np加入iou，而不将iou加入np
  + 当前都是将** ，融入到被updateObjectPose的id中。
  + 查看mReObj后续的用途
+ 只要ProIou大于0.25就认为是，潜在的关联对象？？
+ ProIou=4 //Debug=4
+  Read t-distribution boundary value.  为什么t分布有二维值
+ obj3d->ComputeProjectRectFrameToCurrentFrame(*mpCurrentFrame);
+ object3d的ComputeMeanAndStandard和IsolationForestDeleteOutliers有什么区别？？
  + this->ComputeMeanAndStandard();
  + this->IsolationForestDeleteOutliers();
  + 这两者的顺序，怎么时而前，时而后
  + iforest不处理  75 64 65， 为什么？ 62的iforest阈值不同。
  + #pragma once
  + iforest 关闭后,试试效果pro iou debug: iou
+ MotionIou的速度估计有问题
+ if ((fIou < 0.5) && (fIou2 < 0.8))  这个指标 是提升了,还是下降了??
+ mSumPointsPos += x3d;  这些能不能去掉， 改为一个统一计算均值中心坐标的函数
+ 为什么要将object3d中的原point， 因为不符合最新帧的物体检测框， 就将原point剔除掉。 完全可能是因为当前帧的观测不到位
+ 如果此物体过去30帧都没有被看到, 而且被观测到的帧数少于10, 且与地图中的其他物体过于重合 ,则设置为bad_3d
+ mmAppearSametime 和 PotentialAssociatedObjects 的区别。 在active slam中的用途
+ 怎么匹配失败了。 用前几帧 测试一下。
+ 彩色纹理的正态分布
+ 为什么字ProIou中, 只将过去30帧的物体,投影过来
// object appeared in the last 30 frames.
            if (obj3d->mnLastAddID > mCurrentFrame.mnId - 30)
                obj3d->ComputeProjectRectFrameToCurrentFrame(mCurrentFrame);  //将obj3d中的point投影到当前帧中，计算投影边界框
            else
            {
                obj3d->mRect_byProjectPoints = cv::Rect(0, 0, 0, 0);
            }
  + 是不是为了在localmap中解决合格问题
  + ProIou 可以看到, 很多物体的Pro Rect很差
  + 暂时通过ProIou_only30_flag 关闭这个选项
+ MergeTwoMapObjs_fll中1.1的系数 要不要扩大到1.2
+ mvpMapObjectMappoints[m]->feature_uvCoordinate = pMP->feature_uvCoordinate; 中, 万一被融合物体中的pMP,更老旧怎么办???
+ 去掉mnConfidence_foractive, 直接用object2d的size代替?
+ 均分和bigToSmall,几乎没效果
+ GetNewObjectMappoints() 为什么点这么少啊??
+ object3d中的mRect_byProjectPoints 是投影到当前帧的投影框, 根据tTrackMotion中的obj3d->ComputeProjectRectFrameToCurrentFrame(mCurrentFrame),没获取一帧, 地图中物体的投影框就会重新计算
+ 计算最新一帧中 线段 和 cube边 的角度偏差      // 为什么只用最新一帧?? 万一不满足之前帧的中线段 怎么办?

### 0 新程序重构--要做的事情:
+ 后端剔除物体的原因. 是不是因为看到的点太少了?
+ 估计方向
+ 新的object点为什么 这么少?
+ 提取平面
+ 纹理的方差和均值
+ 今天一定要融合信息熵和方向估计
+ 为什么if(obj3d->mnLastAddID == mCurrentFrame.mnId)代表出现在当前帧
+ SampleObjYaw能不能整合到object3d中
+ MergeTwoMapObjs_forlocalmap中没有 关于object2d的筛选吗?
+ 后端融合时, 把obj2d添加过去有意义吗?
+ 丰富confidence的用法，用来解决后端带来的bug
  + 似乎j--已经解决了这个问题
+ Tracking::CreatObject_intrackmotion(),应该放在object的构造函数中
  + 数据关联 应该放在trackmotion trackReference trackLocal中
+ if (p == nullptr) 为什么会为空呢??
+ rviz的报错
+ 对于椅子，去掉z轴的iforest


### 0 未来可以微调的参数
+ 剔除物体检测框时的预设参数


### 0 算法学习
+ mlRelativeFramePoses.back(); 存的是上一帧到到参考关键帧的位姿变化
+ if(mpCurrentFrame.mvbOutlier[i])  mvbOutlier是怎么存入的？
+ 给48 × \times× 64个小网格mGrid[i][j]分配特征点。特征点的像素坐标在哪个网格中，则特征点属于哪个网格。
+ 内外点之分最简单的说法就是是否符合当前位姿的判断：如果根据当前位姿，之前帧二维特征点所恢复出的地图点重投影到当前帧与实际的二维特征点匹配不上了，那么认为这个是质量差的点是outlier，抛弃掉，如果能匹配上，那就是inlier，保留。还有一种情况，根据地图点3D位置，当前帧位姿理论上看不到这个地图点，在相机视野之外，也认为这个地图点是当前帧的outlier。
+ 成员变量std::map<KeyFrame*,size_t> mObservations保存了当前关键点对关键帧KeyFrame的观测关系,
  + std::map是一个key-value结构,其key为某个关键帧,value为当前地图点在该关键帧中的索引(是在该关键帧成员变量std::vector<MapPoint*> mvpMapPoints中的索引).
  + 和 nObs 的关系
+ optimization中inlier和outlier的不同：
  + e->setLevel(1);    // 设置为outlier , level 1 对应为外点,上面的过程中我们设置其为不优化
  + e->setLevel(0);   // 设置为inlier, level 0 对应为内点,上面的过程中我们就是要优化这些关系
+ mappoint的深入学习，对扭头算法很重要 https://guyuehome.com/37713
+ 雷达的point怎么生成的？
  + 

### 0 track的流程
+ 如果trackmotion  或者trackReference  成功了(bOK==TRUE). 那么我们就获得了相机的粗略位姿. 接下来通过TrackLocalMap ,获得更精确的位姿.  
+ 如果 TrackLocalMap 也成功了, 就是最好的结果. 那可以考虑生成关键帧
+ 如果 TrackLocalMap 失败了,并且刚初始化不久. 则重新开始建图
+ 最后,如果当前帧的参考关键帧 不存在, 则设置为当前的参考关键帧. 将当前帧成为 last帧.  开始处理下一帧


### 0 trackmotion和trackReferenc的区别
+ current帧的位姿,motion通过速度估计,reference等于上一帧
+ nmatches, motion比reference的要求高
+ 

### 1 过程记录
``
**TODO：**
+ 在实际场景中尝试:
  + 前景物体：鼠标,笔记本,椅子,
  + 背景物体：桌子,

+ 周日：
+ 将cartographer的相机位姿，作为物体建图的相机初始位姿
  + tf变换
  + 雷达的位姿, 作为rgbd相机的初始位姿.
    + 通过时间戳, 匹配相机的位姿
    + g2o中用 setToOriginImpl()初始化估计值.
    + virtual void setToOriginImpl() { _estimate = Plane3D() ;}   //?????
    + optimizer.initializeOptimization(0); ?
  + 根据tf和雷达位姿, 生成相机的位姿, 一同输入进system中.
  + 视觉里程计里面的:  mpCurrentFrame.SetPose(mLastFrame.mTcw);  为什么位姿设置的是上一帧的位姿.
    + 通过这个函数，输入每一帧的初始位姿
    + 

  

+ 关键1：NBV物体概率地图 
  + 编写gazebo的自主仿真模型. ￥￥￥￥
  + 通过夹角,判断物体是否在相机的视场内
  + 通过平面选择是否要,
  
~~+ 关键2：学习 特征点聚类的算法，并结合到nbv中~~
+ 关键2: 通过特征点, 调动机器人的偏头
+ 关键3：跑一遍雷达和相机的联合建图，用于周五的展示,并展示出

+ 怎么实现转头:
+ 用ros将角度发送出去
+ tracklocalmap是非关键帧，业参与吗？  如果是的话，  只针对关键帧做MAM，这样就不用发布那么多转头动作了。
+ system return mgreat_num 时，加一个锁


+ 转变成release，能不能解决molloc内存泄露的问题
+ 
+  专用于nbv的test/ 用于nbv test , 记得更改这些备注
+ //TODO:绝对数字     这也是专用于nbv的固定18的标志

+ 生成物体的方法是，2d object的dataAssociated，
+ 
+ 后端的线程，是直接调用的前端的
  + unique_lock<mutex> lock(MapPoint::mGlobalMutex);
  + unique_lock<mutex> lock(MapPlane::mGlobalMutex);
  + unique_lock<mutex> lock(pMap->mMutexMapUpdate);
  + 
+ 线程的待做:[锁线程应该用在其他线程,来调用本线程的时候]
  + Object_2d->mAssMapObjCenter 还没有改为get set的形式. 尽管这个变量在本程序中并没有起作用.
  + this->mAssMapObjCenter = _Pos.clone(); //this->GetWorldPos();  crash bug   这什么意思?/
  + 
+ [锁线程应该用在其他线程,来调用本线程的时候]
  +  检查一遍 类似vObjs[i]->mvpMapObjectMappoints; 的问题
+ obj->sum_pos_3d 和 obj->_Pos 不相等, 一个是物体的中心, 一个是特征点的坐标平均值.  但应注意到 特征点在物体内的分布不是均匀的,因此两者也不等
+ 多个线程嵌套可以吗? --> unique_lock<mutex> lock(mMutexPos);  //zhang:  对_Pose和sum_pos_3d计算时,禁止其他访问

+ ComputeMeanAndStandardFrame()的作用:
  + 重新计算object_2d中的点的均值和方差.主要是因为有些MapPoint被鉴定为bad, 需要从平均坐标值中剔除
  + 用在了Remove Outlier 和 MergeTwoFrameObj_2d中
+ 3D的 ComputeMeanAndStandard(); 用在了
  + 数据关联更新  DataAssociateUpdate_forobj2d  (有bug)
  + 是否融合两个mapObject  SearchAndMergeMapObjs_fll(包含了MergeTwoMapObj 有bug)
  + BigToSmall_fll
  + 处理两个物体的重叠   DealTwoOverlapObjs_fll(包含了MergeTwoMapObj 有bug)
  
+ localmap线程用到的
  + DealTwoOverlapObjs_fll  (包含了MergeTwoMapObj 有bug)
  + SearchAndMergeMapObjs_fll  (包含了MergeTwoMapObj 有bug)
  + ComputeMeanAndStandard()

+ 顺序
  + 禁掉了view
  + 禁掉了localmap中的eao部分的函数调用   关键词//报错debug
  + 禁掉了track中的pose //Optimizer::PoseOptimization(&mpCurrentFrame);  //报错debug
  + 禁掉了 track中的 NBV MAM: check the camera model
  + 禁掉了 
  + map.cc 中添加void AddObject(Object_Map *pObj);   //zhangjiadong    
  + todo  object在map中的vector改为 set
  + object.cc中 //zhang 去掉了this->
  + todo： 把cuboidCenter改为实时计算，而非存储



  
+ 把物体的共视关系也添加到 转头中
``
**已完成部分：**

+ 将yolo改为ros版本（darknet ros）,并通过ros message_filters::Synchronizer进行输入.
+ 编写了rosbag timestamp correct程序以修正,相机rosbag和darknet_net的时间戳
+ darknet_ros未识别到物体时，也输出/darknet_ros/bounding_boxes。在darknet_ros源码的YoloObjectDetector.cpp的publishInThread()函数中,修改if (num > 0 && num <= 100)未if (num >= 0 && num <= 100)
+ 在config文件中,ConstraintType为0时,设定初始帧的位姿为重力方向。
+ 在config文件中,ConstraintType为2时,根据kinect中的IMU生成真实的重力方向. 注意:IMU输出的位姿orientation,是imu坐标系在世界坐标系下的姿态, 即:世界坐标系到imu坐标系的旋转变换关系.
+ 时间测试: 可知yolo处理一帧图像是0.07秒（14hz），ros_eao处理一帧(提取特征点和特征面)是0.076秒。
+ 将物体信息和平面信息, 保存为txt
  + 平面信息的格式: 中心坐标三维, 法向量三维
  + 物体信息的格式:
+ 在rviz中可视化： 物体及内部的特征点、相机、关键帧、共视图、特征点、~~平面~~，topic为/objectmap
+ FABO移动平台的导航调试：
  + 全局路径规划voronoi_planner。 存在路径规划失败导致程序崩溃的bug， 可以试试Hybrd A*（https://blog.csdn.net/qq_42568675/article/details/116116994  https://github.com/dengpw/hybrid_astar_planner）试试
  + 局部路径规划teb_planner.  机器人倒车时，可能配到障碍物。
  + 通信频率10会崩溃， 尝试wifi能否解决问题
+ 

**未来想进行的修改 和 学习的内容：**
+ 运行tum数据集中的long_office，会报错。还没找到问题所在，是否与amd的cpu有关（在旧intel笔记本上没有这个bug）。可以尝试从gcc编译flag、c++版本、eigen版本等方面，试一试。
+ 有时候相机会卡死, 推测可能是message_filters::Synchronizer的问题. 重启相机后,可修复.
+ 怎么感觉frame.h中多生命了一个构造函数??  c文件里面还重复写了一个rgbd的frame的构造函数
+ KeyFrame *pKF = new KeyFrame(mpCurrentFrame, mpMap, mpKeyFrameDB);  关键帧里的词袋是怎么生成和使用的?
+ plane在loop close中是怎么起作用的??  发现和point是一样的,连变量名字都是一样的.  https://blog.csdn.net/YGG12022/article/details/124958961 . 
+ plane在回环检测和优化中,起到了重要作用, 因此我们把这个平面也作为一个重要的观测对象.
+ PEAC方法中,深度信息只用了0.2~4之间.是否妥当
+ mspMapPlanes中的平面 是什么时候添加进去的?
+ 特征点的UpdateNormalAndDepth 是什么用法??
+ g2o学习:
  + setToOriginImpl()初始化估计值.
+ 如何去除的真值约束? 对比加上和不加垂直方向的初始约束的效果；
+ 如何进行的平面的数据关联?  pMP->GetObservations()->second里存储的是什么？
+ 平面g2o的雅克比矩阵,怎么构建。答：linearizeOplus()函数里主要是配置雅克比矩阵。当然，G2O是支持自动求导的，该函数可以不实现。优化时由G2O自动处理。但准确的实现可加快优化计算的速度。下面介绍雅克比矩阵该如何计算。
+ plane edge的信息矩阵 ？ 答：信息矩阵中有三维，前两维度表示角度权重，最后一维表示距离权重

**失败的内容：**
+ evo比较有无平面优化时，轨迹精度的变化。 只能通过读取本地图片与物体检测的方法，进行比较
+ 

### 2 知识学习
+ PEAC平面提取方法介绍 --- ComputePlanesFromPEAC(const cv::Mat &imDepth)
  + 对深度图降采样: imDepth --> iCloud
  + ahc::PlaneFitter 从iCloud中提取平面. 平面在程序中的应用方法是:
    + plane_vertices_ :  plane_vertices_.at(i) 存储了i平面中各point, 在icloud(深度图像indepth间隔采样的结果)中的像素的索引.
    + extractedPlanes :  extractedPlanes.at(i) 存储了i平面的法向量,中心坐标.
  + 根据plane_vertices_中的像素索引, 从icloud中提取点云:   icloud --> inputCloud
  + 利用pcl::VoxelGrid对inputCloud,进行体素化的下采样:  inputCloud -->  dcoarseCloud
  + 将平面的点云信息dcoarseCloud, 添加进frame的mvPlanePoints中.   平面的法向量和坐标coef,添加进frame的mvPlaneCoefficients中
+ 平面参与优化 --- https://blog.csdn.net/supengufo/article/details/105846129
  + 读取地图或者临近帧的平面: vector<MapPlane *> vpMPl = pMap->GetAllMapPlanes();
  + 误差函数的构建:
    + 
  + 构建g2o的因子图:
    + 将vpMPl[i],添加为构建g2o中的点 VertexPlane.
    + 将vpMPl[i]->GetObservations()->first ,即观测到此平面的关键帧, 添加为g2o中的点. [这一步其实已经在BundleAdjustment的关键帧处理部分,已经完成了]
    + 添加边: optimizer.addEdge(edge);
    + 正式开始优化 optimizer.optimize(nIterations);
    + 将优化的结果, 赋值给plane. 
    
+ 平面的误差_error,是怎么构建的(computeError())
  + https://zhuanlan.zhihu.com/p/375820263
  + https://blog.csdn.net/zkk9527/article/details/89673896
  + https://blog.csdn.net/supengufo/article/details/105846129
  + 获取第i帧相机的位姿w2n, 类型为Plane3D
  + 获取平面位姿Pw(在world下的坐标), 格式为Isometry3D
    + 欧式变换Isometry3D入门: https://blog.csdn.net/LLABVIEW/article/details/124607433
  + 两者相乘w2n*plane, 将世界坐标系下的平面Pw  ,投影到第i帧坐标系下, 得到平面Pi ,即localPlane
  + 计算 localPlane 和 _measurement(关联成功的平面) 的误差
    + 角度
    + 距离
  
+ 物体融合
  + Converter::bboxOverlapratio(bbox1, bbox2)
  + 

+ UpdateAllMapObject()\MergePotentialAssObjs()\WhetherOverlapObject(); 为什么都是在localMapping中.
  + localMapping是对全局的map做处理, 例如: 全局的物体 mpMap->GetObjects();
  + 前端(tracker)负责生成物体, 后端负责优化物体(融合和位姿优化).
  
+ LocalBA是应用在localmapping中; 其他的poseBA是应用在tracker中; globalBA(在tracker的单目初始帧生成中用了一次),OptimizeEssentialGraph,OptimizeSim3应用在回环中.
  + 以上几种BA的区别?
+ 
+ 虚函数: 指向基类的指针在操作它的多态类对象时，会根据不同的类对象，调用其相应的函数，这个函数就是虚函数。
+ Object_Map::UpdateObjPose_forlocalmap() 将最新的mCuboid3D.rotP/rotY/rotR, 赋值给Twobj, 进而赋值给 mCuboid3D.pose(这就是物体在世界坐标系下的位姿).
  + Twobj_without_yaw 只保存了中心坐标, 即: 平行于世界坐标系
  + 

### 3 环境构建

```yaml
Ubuntu: 18.04
opencv: 3.4.10
Eigen: 3.2.1 ?? 
darknet_net: 
rapidjson: 1.1.0 
boost: 1.65.1
```
