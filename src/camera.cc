#include "camera.h"



camera::camera(map_data mapData, int thres)
{
	std::cout << "Initializing camera data..." << std::endl;

    // 没用到
    //feature_threshold = thres;

    //将map中的特征点信息,存入到成员vector中
    upper_bound.resize(mapData.UB.size());      //观测角度(共视方向)的最大值
    lower_bound.resize(mapData.LB.size());          //观测角度(共视方向)的最小值
    max_range_dist.resize(mapData.maxDist.size());       //可视距离
    min_range_dist.resize(mapData.minDist.size());       //可视距离
    foundRatio.resize(mapData.foundRatio.size());   //查找率
    vector<float> v_float_ub(mapData.UB.begin(), mapData.UB.end());
    vector<float> v_float_lb(mapData.LB.begin(), mapData.LB.end());
    vector<float> v_float_max_dist(mapData.maxDist.begin(), mapData.maxDist.end());
    vector<float> v_float_min_dist(mapData.minDist.begin(), mapData.minDist.end());
    vector<float> v_float_found_ratio(mapData.foundRatio.begin(), mapData.foundRatio.end());
    upper_bound = v_float_ub;
    lower_bound = v_float_lb;
    max_range_dist = v_float_max_dist;
    min_range_dist = v_float_min_dist;
    foundRatio = v_float_found_ratio;

    //将map中的特征点坐标, 存入到成员vector中
    for (int i = 0; i < mapData.MapPointsCoordinate.size(); i++) {
        vector<float> v_float(mapData.MapPointsCoordinate[i].begin(), mapData.MapPointsCoordinate[i].end());
        map_points_vec.push_back(v_float);
    }

    //不知道用途
    //for(int i=0; i<GRID_COLS; i++){
    //    colCoeff[i] = i;
    //}
    //
    //for(int i=0; i<GRID_ROWS; i++){
    //    rowCoeff[i] = i;
    //}

    //举例: x*gridElementWidthInv 等于, 点在栅格中的第几格.
    //虽然 最后还是没什么用
    gridElementWidthInv=static_cast<float>(GRID_COLS)/(MaxX-MinX);
    gridElementHeightInv=static_cast<float>(GRID_ROWS)/(MaxY-MinY);
}

camera::camera()
{
	std::cout << "Initializing camera data..." << std::endl;
    //feature_threshold = 20;
}

void camera::setCamera(const float fx_, const float fy_, const float cx_, const float cy_,
                       const float width_, const float height_,
                       const float max_dist_, const float min_dist_ )
{
    fx = fx_;
    fy = fy_;
    cx = cx_;
    cy = cy_;
    MaxX = width_ - 10.0;
    MinX = 10.0;
    MaxY = height_ - 20.0;
    MinY = 20.0;
    max_dist = max_dist_;   //0.5 - 3.86 米
    mMin_dist = min_dist_;
}


int camera::countVisible(cv::Mat Twc) const {
    //point的数量
    int num_pt=map_points_vec.size();

    float num_visible=0;

    // I modified this section to enable the function to be const. (requirement of OMPL)
    Eigen::Matrix<float, 4,4> T_wc;
    cv2eigen(Twc, T_wc);
    Eigen::Matrix4f T_cw = T_wc.inverse();

    if (1) {        //setRobotPose(x_w, y_w, theta_rad_w)) {
        for (int i = 0; i < num_pt; ++i)
        {   
            proj_info mpProjection;
            // 计算map中每个点，是否在相机的视场内,并 TODO:计算特征点在哪个格子中
            mpProjection = isInFrustum(map_points_vec[i], upper_bound[i], lower_bound[i], T_wc, T_cw, max_range_dist[i], min_range_dist[i]);
            if(mpProjection.success)
                num_visible+=foundRatio[i];
        }
    }
    return int(round(num_visible));
}




std::vector<int> camera::posInGrid(float u, float v) const{

    int posX = round((u-MinX)*gridElementWidthInv);
    int posY = round((v-MinY)*gridElementHeightInv);

    return std::vector<int>{posX, posY};
    
}


proj_info camera::isInFrustum(std::vector<float> MapPoint_w, float upper_limit_theta, float lower_limit_theta, Eigen::Matrix4f T_wc, Eigen::Matrix4f T_cw, float max_range_dist, float min_range_dist) const {
     
        //将图片变换到相机坐标系中 convert map points into carmera frame
        Eigen::Matrix<float, 4, 1> point_w(MapPoint_w.data());
        point_w(3,0) = 1;
        Eigen::Matrix<float, 4, 1> point_c=T_cw * point_w;
         
        float PcX = point_c(0,0);
        float PcY= point_c(1,0);
        float PcZ = point_c(2,0);

        proj_info map_projection;
         
        // step 1:  如果点的深度太小，也就是离相机太近，则剔除
        // rule out the points with depth smaller than 0.05
        if(PcZ<0.05)
            return map_projection;

        // step 2:  如果点的uv坐标 不在相框内，，则剔除
        // rule out the points which are out of current view
        float inv_z = 1.0f/PcZ;
        float u=fx*PcX*inv_z+cx;
        float v=fy*PcY*inv_z+cy;
        if(u<MinX || u>MaxX)    
            return map_projection;
        if(v<MinY || v>MaxY)
            return map_projection;

        // step 3: 如果点离相机中心太远，或者太近，则剔除
        // rule out the points which are too close or too far away
        float dist=sqrt(PcZ*PcZ+PcY*PcY+PcX*PcX);
        if(dist < mMin_dist || dist>max_dist )
            return map_projection;

        if(dist<min_range_dist )  // || dist>max_range
            return map_projection;


        // step 4:  point的观测方向,是否符合t分布
        // rule out the points whose viewing direction is out of 95% t-distribution
        float delta_x_w=point_w(0,0) - T_wc(0,3);   //世界坐标系下,点的x坐标到到相机中心的x值
        float delta_y_w=point_w(1,0) - T_wc(1,3);   //世界坐标系下,点的y坐标到到相机中心的y值
        float delta_z_w=point_w(2,0) - T_wc(2,3);   //世界坐标系下,点的z坐标到到相机中心的z值
        float theta_robot_w=atan2(delta_x_w,delta_y_w);   //atan2(float __y, float __x)   与y轴的夹角
        if(theta_robot_w < lower_limit_theta || theta_robot_w > upper_limit_theta)
            return map_projection;

        // step 5:
        map_projection.success = true;
        std::vector<int>pos_xy = posInGrid(u,v);
        map_projection.x_grid = pos_xy[0];
        map_projection.y_grid = pos_xy[1];
         
        return map_projection;
    
}

//std::vector<std::vector<float> > camera::read_text(std::string filename)
//{
//    std::ifstream          file(filename.c_str());
//    if (!file.is_open()) {
//    	cout << "Error opening file " << filename.c_str() << endl;
//    	exit(1);
//    }
//
//    std::string  line;
//    std::cout<<"open file: "<<filename<<std::endl;
//    std::vector<std::vector<float>> out;
//
//    // Read one line at a time into the variable line:
//    while(std::getline(file, line))
//    {
//        std::vector<float>   lineData;
//
//        std::stringstream  lineStream(line);
//
//        float value;
//        // Read an integer at a time from the line
//        while(lineStream >> value)
//        {
//            // Add the integers from a line to a 1D array (vector)
//            lineData.push_back(value);
//            //std::cout<<value<<" ";
//        }
//        // When all the integers have been read, add the 1D array
//        // into a 2D array (as one line in the 2D array)
//        out.push_back(lineData);
//    }
//
//    return out;
//
//}


//std::vector<float>  camera::read_text_single_line(std::string filename)
//{
//    std::ifstream          file(filename.c_str());
//
//    std::string  line;
//    std::cout<<"open file: "<<filename<<std::endl;
//    std::vector<float> out;
//
//    // Read one line at a time into the variable line:
//    while(std::getline(file, line))
//    {
//
//        std::stringstream  lineStream(line);
//
//        float value;
//        // Read an integer at a time from the line
//        while(lineStream >> value)
//        {
//            // Add the integers from a line to a 1D array (vector)
//            out.push_back(value);
//        }
//        // When all the integers have been read, add the 1D array
//        // into a 2D array (as one line in the 2D array)
//
//    }
//    std::cout<<" Load Finished!"<<std::endl;
//    return out;
//
//}
//
//
//void camera::compute_std(std::vector<float> v, float & mean, float & stdev)
//{
//    float sum = std::accumulate(v.begin(), v.end(), 0.0);
//    mean = sum / v.size();
//
//    std::vector<float> diff(v.size());
//    std::transform(v.begin(), v.end(), diff.begin(),
//                std::bind2nd(std::minus<float>(), mean));
//    float sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
//    stdev = std::sqrt(sq_sum / v.size());
//}




/*
cv::Mat_<float> camera::vec2cvMat_2D(std::vector< std::vector<float> > &inVec){
  int rows = static_cast<int>(inVec.size());
    int cols = static_cast<int>(inVec[0].size());

    //std::cout<<rows<<" "<<cols<<std::endl;

    cv::Mat_<float> resmat(rows, cols);
    for (int i = 0; i < rows; i++)
    {
        resmat.row(i) = cv::Mat(inVec[i]).t();
    }
    std::cout<<" Load Finished!"<<std::endl;
    return resmat;
}*/


//camera::camera(std::vector<ORB_SLAM2::MapPoint*> &vpPts)
//{
//	std::cout << "Initializing camera data..." << std::endl;
//
//    feature_threshold = 20;
//
//    //load files Map
//    update_map(vpPts);
//
//}


//void camera::update_map(std::vector<ORB_SLAM2::MapPoint*> &vpPts)
//{
//    if(!map_vec.empty())
//    {
//        map_vec.clear();
//        upper_bound.clear();
//        lower_bound.clear();
//    }
//
//
//    if(!vpPts.empty())
//    {
//        for(size_t i=0; i<vpPts.size(); i++){
//            ORB_SLAM2::MapPoint* pPt = vpPts[i];
//
//            if(pPt->isBad())
//                continue;
//
//            cv::Mat Pos = pPt->GetWorldPos();
//            cv::Mat Normal = pPt->GetNormal();
//            int number_observed = pPt->Observations();
//
//            std::vector<float> one_pt;
//            one_pt.push_back(Pos.at<float>(0));
//            one_pt.push_back(Pos.at<float>(1));
//            one_pt.push_back(Pos.at<float>(2));
//            one_pt.push_back(1.0f);
//
//            map_vec.push_back(one_pt);
//            float theta_mean=pPt->theta_mean;
//            float theta_std=pPt->theta_std;
//            lower_bound.push_back(theta_mean-2.5*theta_std);
//            upper_bound.push_back(theta_mean+2.5*theta_std);
//        }
//    }
//}




//
//int camera::countVisible(float x_w, float y_w, float theta_rad_w) const {
//    int num_pt=map_vec.size();  //zhang  存入的是MD.Map中的
//    float num_visible=0;
//
//    //cout << x_w << " " << y_w << " " << theta_rad_w << endl;
//
//    // I modified this section to enable the function to be const. (requirement of OMPL)
//    Eigen::Matrix4f T_wb;
//    T_wb<<(float)(cos(theta_rad_w)),  (float)(-sin(theta_rad_w)), 0, x_w,
//            (float)(sin(theta_rad_w)), (float)cos(theta_rad_w), 0, y_w,
//            0,         0,         1,   0,
//            0,         0,        0,    1.0000;
//
//    //Eigen::Matrix4f T_sc = T_sw * T_wb * T_bc;
//    //Eigen::Matrix4f T_cs = T_sc.inverse();
//
//    //my
//    Eigen::Matrix4f T_sc = T_wb ;
//    Eigen::Matrix4f T_cs = T_sc.inverse();
//
//    if (1) {    //setRobotPose(x_w, y_w, theta_rad_w)) {
//
//        for (int i = 0; i < num_pt; ++i)
//        {
//            proj_info mpProjection;
//            mpProjection = isInFrustum(map_vec[i],  upper_bound[i],  lower_bound[i], T_sc, T_cs, max_range[i], min_range[i]);
//            if(mpProjection.success){
//                num_visible+=foundRatio[i];
//            }
//        }
//    }
//    else
//    {
//        std::cout<<"Cannot get current robot pose"<<std::endl;
//    }
//    return int(round(num_visible));
//}
//
//bool camera::IsStateVisiblilty(double x_w, double y_w, double theta_rad_w, int ths) {
//    	int cur_ths;
//    	if (ths == -1)
//    		cur_ths = feature_threshold;   //zhang 默认是这个模式
//    	else
//    		cur_ths = ths;
//
//    	return countVisible(x_w, y_w, theta_rad_w) > cur_ths;
//}