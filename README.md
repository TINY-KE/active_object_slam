# Active-EAO-Fusion
**目标：试图调整 EAO-SLAM ，建立物体级active slam**
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
**TODO**
+ 改良信息熵的计算公式
+ int ObserveNum = mvObject_2ds.size();  
  + if(ObserveNum==0) ObserveNum=40;//todo:  这是因为在融合的时候,有一步没把mvObject_2ds融合
+ marker.id= IE_id ;  //TODO:绝对数字
  + 确定id是不是只针对同一个topic下的可视化单元. 本可视化程序中已经将他们拆分为多个topic了. 应该没什么问题吧
+ mNBV_Angle_correct 为什么需要纠正一下呢??
+ 把costmap在slam找那个生成,从而可以把把cost融入到nbv的评价函数中.
+ 平面的g2o优化 还没加进去
+ 有个物体始终不能生成正确的位姿 mCuboid3D.pose_mat.
  + 看看是哪个物体： 先剔除掉
  + 权益之计，注释掉了一些函数：
    + publishIE()
+ gazebo机器人的转速有点慢？？
+ float theta_robot_w=atan2(delta_x_w,delta_z_w);   //atan2(float __y, float __x)   这个角度是在z轴上的夹角，是不是求错了??
  + 改成了和x轴的。
+ 为什么global nbv到达之后，不移动
  + 是因为地图的信息没更新？
  + 是不是global nbv的算法 没起作用？
  + computeReward(candidate, ObjectMaps);还没有检查
  + 为什么global nbv的可视化结果和实际发送的导航目标  不一样
+ 为什么reward传不回来？？  
  + 是不是下面这个front有问题
    + mvGlobalCandidate[i].reward = globalCandidate.front().reward; 
    + mvGlobalCandidate[i].pose = globalCandidate.front().pose.clone();
  + 猜测是一个元素的vector的sort 造成的。
+ 两个物体融合之后是怎么重新估计yaw？
  + 如果是track的时候 融合的，直接利用当前帧的line重新估计
  + 如果是localmap的时候怎么办？？
+ 估计yaw时，怎么还有个if(obj3d->mnClass == 56)   // chair.
+ 为什么1111.avi视频里面有个  右上角的nbv没过去？
+ 与nbvs old之间的距离. 当一个 物体群,创建完毕之后,就清空nbvs old.
+ 怎么设计结束条件:?
  + 当物体的

+ 如果一个物体的长宽比大于5，就去除掉。后端也加上
+ zhangjiadong： 原程序将体积较小的删除？？？我认为不合理，改为将观测较少的物体删除
+ 怎么降低最后的旋转速度
+ 检查融合时，object2d的融合  好像有遗留问题
+ 事实证明，在移动过程中，物体会频繁更新。因为ie的更新  也应该放在localmap中
+ 研究MergeTwoMapObjs_fll(中的viewdCount_forObjectId
+ float mnViewField 移到nbvgenerator中



+ 暂时去掉global nbv的旋转： vector<Candidate> localCandidate = RotateCandidates(globalCandidate);
  + 或者减小 旋转角度
  + globalCandidate.push_back(globalCandidate_source);
+ std::sort(globalCandidate.begin(), globalCandidate.end(), [](Candidate a, Candidate b)->bool { return a.reward > b.reward; });
+ 



**以后的研究方向**
+ local nbv怎么影响路径规划？

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
+ 实现了基于gazebo的movebase导航
+ 局部nbv整合进nbv generator线程中，并梳理出伪代码
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
+ 在rviz中显示正确的信息熵
  + rviz不支持这个功能
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
