#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
// #include "detection/radar_tracker.h"
#include "detection/radar_cmkf_tracker.h"
#include "detection/camera_tracker.h"
#include "detection/lidar_tracker.h"
#include "detection/sensor_fusion.h"

extern ros::Publisher fusion_pub;
extern std::string FIXED_FRAME;
extern float X_OFFSET;
// extern RadarTracker radar_tracker;
extern RadarCMKFTracker radar_tracker;
extern CameraTracker camera_tracker;
extern LidarTracker lidar_tracker;

SensorFusion::SensorFusion(void)
{
    local_matched_pair.clear();
    radar_matched.clear();
    lidar_matched.clear();
    matched_pair.clear();
    prev_matched.clear();
    src_matched.clear();
    track_info.clear();
    X.clear();
    P.clear();
    radar_tracks.clear();
    lidar_tracks.clear();
    camera_tracks.clear();
    
    F = matrix6d::Zero(6,6);
    F(0,0) = 1;          F(1,1) = 1;
    F(2,2) = 1;          F(3,3) = 1;
    F(4,4) = 1;          F(5,5) = 1;
    Q = matrix6d::Zero(6,6);
    Q(0,0) = 0.001;     Q(1,1) = 0.001;
    Q(2,2) = 0.05;      Q(3,3) = 0.05;
    Q(4,4) = 0.1;       Q(5,5) = 0.1;
}

SensorFusion::~SensorFusion() { }

void SensorFusion::Run(void)
{
    // clock_t start = clock();
    GetLocalTracks();

    if (!X.size())
    {
      for (auto &pair : local_matched_pair)
      {
          InitTrack(pair);
      }
      prev_stamp = time_stamp;
    }
    else
    {
      ts = (time_stamp - prev_stamp).toSec();
      prev_stamp = time_stamp;
      Predict();
      MatchGNN();
      Update();
    }
    // clock_t end = clock();
    // float duration_ms = (float)(end-start)*1000/(float)CLOCKS_PER_SEC;  //程序用时 ms
    // std::cout << "duration(ms) = " << duration_ms << std::endl;
    PubFusionTracks();
}

void SensorFusion::GetLocalTracks(void){
    lidar_tracker.GetTimeStamp(time_stamp);
    ros::Time radar_stamp, camera_stamp;
    radar_tracker.GetTimeStamp(radar_stamp);
    camera_tracker.GetTimeStamp(camera_stamp);
    lidar_tracker.GetLidarTrack(lidar_tracks);
    radar_tracker.GetRadarTrack(radar_tracks);
    camera_tracker.GetCameraTrack(camera_tracks);

    //对齐radar信息
    double dt = (time_stamp - radar_stamp).toSec();
    F(0,2) = F(1,3) = F(2,4) = F(3,5) = dt;
    F(0,4) = F(1,5) = dt*dt/2;
    matrix6d Qr = matrix6d::Zero(6,6);
    Qr(0,0) = 0.05;      Qr(1,1) = 0.05;
    Qr(2,2) = 0.1;       Qr(3,3) = 0.1;
    Qr(4,4) = 0.3;       Qr(5,5) = 0.3;
    for(std::vector<LocalTrack>::iterator it= radar_tracks.begin(); it!= radar_tracks.end(); ++it){
        it->X = F * it->X;
        it->X(0) += X_OFFSET;
        it->P = F * it->P * F.transpose() + Qr;
    }
    // std::cout << "radar dt = " << dt*1000 <<std::endl;

    //对齐camera信息
    dt = (time_stamp - camera_stamp).toSec();
    F(0,2) = F(1,3) = F(2,4) = F(3,5) = dt;
    F(0,4) = F(1,5) = dt*dt/2;
    for(std::vector<LocalTrack>::iterator it= camera_tracks.begin(); it!= camera_tracks.end(); ++it){
        it->X = F * it->X;
    }

    // lidar & radar 局部航迹数据关联
    int lidar_track_num = lidar_tracks.size();
    int radar_track_num = radar_tracks.size();
    local_matched_pair.clear();
    if(radar_track_num == 0 && lidar_track_num > 0){
        for (int i=0; i<lidar_track_num; ++i)
        {
            std::pair<int, int> pair(-1, i);  // 表示单独的lidar航迹
            local_matched_pair.push_back(pair);
        }
        return;
    }else if(radar_track_num > 0 && lidar_track_num == 0){
        for (int i=0; i<radar_track_num; ++i)
        {
            std::pair<int, int> pair(i, -1);  // 表示单独的radar航迹
            local_matched_pair.push_back(pair);
        }
        return;
    }else if(radar_track_num == 0 && lidar_track_num == 0){
        return;
    }
    // std::cout << "radar_track_num = " << radar_track_num << "  lidar_track_num = " << lidar_track_num<< std::endl;
    
    radar_matched.clear();
    radar_matched.resize(radar_track_num, false);
    lidar_matched.clear();
    lidar_matched.resize(lidar_track_num, false);
    matrixXd w_ij = matrixXd::Zero(radar_track_num, lidar_track_num + radar_track_num);

    for ( int i = 0; i < radar_track_num; ++i )
    {
        vector6d Xr = radar_tracks[i].X;
        matrix6d Pr = radar_tracks[i].P;
        for ( int j = 0; j < lidar_track_num; ++j ){
            vector6d Xl = lidar_tracks[i].X;
            matrix6d Pl = lidar_tracks[i].P;
            if(fabs(Xr(0)- Xl(0)) < RX_TRACK_GATE && fabs(Xr(1)- Xl(1)) < RY_TRACK_GATE){
                matrix6d C = Pr + Pl;
                w_ij(i, j) = normalDistributionDensity< 6 >(C, Xr, Xl);
                // std::cout << "w_ij(i, j) = " << w_ij(i, j) <<std::endl;
            }else{
                w_ij(i, j) = 0;
                // std::cout << "fabs(Xr(0)- Xl(0)) = " << fabs(Xr(0)- Xl(0)) <<"  fabs(Xr(1)- Xl(1)) = " << fabs(Xr(1)- Xl(1))<< std::endl;
            }
        }
    }

    for ( int j = lidar_track_num; j < lidar_track_num + radar_track_num; ++j ){
        w_ij(j - lidar_track_num, j) = LOCAL_SINGLE_WEIGHT;
    }

    // solve the maximum-sum-of-weights problem (i.e. assignment problem)
    // in this case it is global nearest neighbour by minimizing the distances
    // over all measurement-filter-associations
    Auction<double>::Edges assignments = Auction<double>::solve(w_ij);

    // for all found assignments
    for ( const auto & e : assignments )
    {
        if ( e.y < lidar_track_num )  // radar & lidar 关联成功
        {
            std::pair<int, int> pair(e.x, e.y);  // (radar, lidar)
            local_matched_pair.push_back(pair);
            radar_matched[e.x] = true;
            lidar_matched[e.y] = true;
        }
        else // 未关联的局部航迹 radar
        {
            std::pair<int, int> pair(e.x, -1);  // 表示单独的radar航迹
            local_matched_pair.push_back(pair);
        }
    }

    for (int i=0; i<lidar_track_num; ++i)
    {
        if (!lidar_matched[i]) // 未关联的局部航迹 lidar
        {
            std::pair<int, int> pair(-1, i);  // 表示单独的lidar航迹
            local_matched_pair.push_back(pair);
        }
    }
}

ObjectType SensorFusion::GetCameraType(const std::pair<int, int>& pair){
    int radar_index = pair.first;
    int lidar_index = pair.second;
    float min_dist= CAMERA_TRACK_GATE;
    int camera_index;
    int size = camera_tracks.size();
    if(lidar_index == -1){
        float x = radar_tracks[radar_index].X(0);
        float y = radar_tracks[radar_index].X(1);
        for(int i=0; i<size; ++i){
            float dist = fabs(x- camera_tracks[i].X(0)) + fabs(y- camera_tracks[i].X(1));
            if(min_dist > dist){
                min_dist = dist;
                camera_index = i;
            }
        }
    }else{
        float x = lidar_tracks[lidar_index].X(0);
        float y = lidar_tracks[lidar_index].X(1);
        for(int i=0; i<size; ++i){
            float dist = fabs(x- camera_tracks[i].X(0)) + fabs(y- camera_tracks[i].X(1));
            if(min_dist > dist){
                min_dist = dist;
                camera_index = i;
            }
        }
    }
    if(min_dist < CAMERA_TRACK_GATE){
        return camera_tracks[camera_index].type;
    }
    return UNKNOWN;
}

void SensorFusion::InitTrack(const std::pair<int, int>& pair)
{
    int radar_index = pair.first;
    int lidar_index = pair.second;
    ObjectInfo init_info;
    init_info.type = GetCameraType(pair);
    vector6d init_X = vector6d::Zero(6);
    matrix6d init_P = matrix6d::Zero(6,6);
    if(radar_index == -1){
        init_X = lidar_tracks[lidar_index].X;
        init_P = lidar_tracks[lidar_index].P;
        init_info.width = lidar_tracks[lidar_index].width;
    }else if(lidar_index == -1){
        init_X = radar_tracks[radar_index].X;
        init_P = radar_tracks[radar_index].P;
    }else{
        init_P = (radar_tracks[radar_index].P.inverse() + lidar_tracks[lidar_index].P.inverse()).inverse();
        init_X = init_P * (radar_tracks[radar_index].P.inverse() * radar_tracks[radar_index].X
                           + lidar_tracks[lidar_index].P.inverse() * lidar_tracks[lidar_index].X);
        init_info.width = lidar_tracks[lidar_index].width;
    }
    X.push_back(init_X);
    P.push_back(init_P);
    track_info.push_back(init_info);
}

void SensorFusion::Predict(void)
{
    if(X.size() != P.size()){
      ROS_ERROR("fusion tracker error: Predict state size not equal");
    }
    F(0,2) = F(1,3) = F(2,4) = F(3,5) = ts;
    F(0,4) = F(1,5) = ts*ts/2;
    int prev_track_num = X.size();
    for (int i=0; i<prev_track_num; ++i)
    {
        X[i] = F * X[i];
        P[i] = F * P[i] * F.transpose() + Q;
    }
}

void SensorFusion::MatchGNN(void)
{
    int prev_track_num = X.size();
    int src_obj_num = local_matched_pair.size();

    matched_pair.clear();
    src_matched.clear();
    src_matched.resize(src_obj_num, false);
    prev_matched.clear();
    prev_matched.resize(prev_track_num, false);

    matrixXd w_ij = matrixXd::Zero(src_obj_num, prev_track_num + src_obj_num);

    for ( int i = 0; i < src_obj_num; ++i )
    {
        int radar_index = local_matched_pair[i].first;
        int lidar_index = local_matched_pair[i].second;
        vector6d Xm;
        matrix6d Pm;
        if(lidar_index == -1){
            Xm = radar_tracks[radar_index].X;
            Pm = radar_tracks[radar_index].P;
        }else{
            Xm = lidar_tracks[lidar_index].X;
            Pm = lidar_tracks[lidar_index].P;
        }
        for ( int j = 0; j < prev_track_num; ++j ){
            vector6d X_ = X[j];
            matrix6d P_ = P[j];
            if(fabs(Xm(0)- X_(0)) < RX_TRACK_GATE && fabs(Xm(1)- X_(1)) < RY_TRACK_GATE){
                matrix6d C = Pm + P_;
                w_ij(i, j) = normalDistributionDensity< 6 >(C, Xm, X_);
                // std::cout << "w_ij(i, j) = " << w_ij(i, j) <<std::endl;
            }else{
                w_ij(i, j) = 0;
            }
        }
    }

    for ( int j = prev_track_num; j < prev_track_num + src_obj_num; ++j ){
        w_ij(j - prev_track_num, j) = FUSION_NEWOBJ_WEIGHT;
    }

    // solve the maximum-sum-of-weights problem (i.e. assignment problem)
    // in this case it is global nearest neighbour by minimizing the distances
    // over all measurement-filter-associations
    Auction<double>::Edges assignments = Auction<double>::solve(w_ij);

    // for all found assignments
    for ( const auto & e : assignments )
    {
        if ( e.y < prev_track_num )  // 与系统航迹关联
        {
            std::pair<int, int> pair(e.x, e.y);  // (local, global)
            matched_pair.push_back(pair);
            src_matched[e.x] = true;
            prev_matched[e.y] = true;

            track_info[e.y].confi_dec = 0;    // target matched, confidence increase
            track_info[e.y].confi_inc++;
            track_info[e.y].confidence += log(track_info[e.y].confi_inc + 1) / log(1.5f);
            if (track_info[e.y].confidence > FUSION_MAX_CONFIDENCE) track_info[e.y].confidence = FUSION_MAX_CONFIDENCE;
        }
        else // 生成新航迹
        {
            InitTrack(local_matched_pair[e.x]);
        }
    }

    for (int i=prev_track_num-1; i>=0; --i)  // from back to front to avoid wrongly removing
    {
        if (!prev_matched[i])
        {
            track_info[i].confi_inc = 0;    // target not matched, confidence decrease
            track_info[i].confi_dec++;
            track_info[i].confidence -= pow(1.5f, track_info[i].confi_dec);

            if(track_info[i].confidence < 0 || !IsConverged(i))    // remove lost target
            {
                RemoveTrack(i);
            }
        }
    }
}

void SensorFusion::Update(void)
{
    if(X.size() != P.size()){
      ROS_ERROR("fusion tracker error: Update state size not equal");
    }
    
    for (int i=0; i<matched_pair.size(); ++i)    // upgrade matched
    {
        int src_index = matched_pair[i].first;
        int prev_index = matched_pair[i].second;
        ObjectType type_ = GetCameraType(local_matched_pair[src_index]);
        if(type_ != UNKNOWN){
            track_info[prev_index].type = type_;
        }
        int radar_index = local_matched_pair[src_index].first;
        int lidar_index = local_matched_pair[src_index].second;

        if(radar_index == -1){
            matrix6d temp_P = (lidar_tracks[lidar_index].P.inverse() + P[prev_index].inverse()).inverse();
            X[prev_index] = temp_P * (lidar_tracks[lidar_index].P.inverse()*lidar_tracks[lidar_index].X 
                                      + P[prev_index].inverse()*X[prev_index]);
            P[prev_index] = temp_P;
            track_info[prev_index].width = lidar_tracks[lidar_index].width;
        }else if(lidar_index == -1){
            matrix6d temp_P = (radar_tracks[radar_index].P.inverse() + P[prev_index].inverse()).inverse();
            X[prev_index] = temp_P * (radar_tracks[radar_index].P.inverse()*radar_tracks[radar_index].X 
                                      + P[prev_index].inverse()*X[prev_index]);
            P[prev_index] = temp_P;
        }else{
            matrix6d temp_P=(radar_tracks[radar_index].P.inverse()+ lidar_tracks[lidar_index].P.inverse()+ P[prev_index].inverse()).inverse();
            X[prev_index] = temp_P * (radar_tracks[radar_index].P.inverse()*radar_tracks[radar_index].X 
                                      + lidar_tracks[lidar_index].P.inverse()*lidar_tracks[lidar_index].X
                                      + P[prev_index].inverse()*X[prev_index]);
            P[prev_index] = temp_P;
            track_info[prev_index].width = lidar_tracks[lidar_index].width;
        }
    }
}

template <class T>
static void erase_from_vector(std::vector<T> &v, int index)
{
    if(index >= v.size()){
      ROS_ERROR("fusion tracker error: remove track index >= size");
      return;
    }
    v.erase(v.begin() + index);
}

void SensorFusion::RemoveTrack(int index)
{
    erase_from_vector(X, index);
    erase_from_vector(P, index);
    erase_from_vector(track_info, index);
}

bool SensorFusion::IsConverged(int track_index)
{
    bool converged = false;
    float rx_cov = P[track_index](0,0);
    float ry_cov = P[track_index](1,1);
    float vx_cov = P[track_index](2,2);
    float vy_cov = P[track_index](3,3);
    float ax_cov = P[track_index](4,4);
    float ay_cov = P[track_index](5,5);
    if (rx_cov < 5 && ry_cov < 5 
        && vx_cov < 25 && vy_cov < 25 && ax_cov < 25 && ay_cov < 25)
    {
        converged = true;
    }
    return converged;
}

void SensorFusion::PubFusionTracks(void)
{
    static int pre_marker_size_ = 0;
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker bbox_marker;
    bbox_marker.header.frame_id = FIXED_FRAME;
    bbox_marker.header.stamp = time_stamp;
    bbox_marker.color.r = 1.0f;    //fusion color yellow
    bbox_marker.color.g = 1.0f;
    bbox_marker.color.b = 0.0f;
    bbox_marker.color.a = 0.5f;
    bbox_marker.lifetime = ros::Duration();
    bbox_marker.frame_locked = true;
    bbox_marker.type = visualization_msgs::Marker::CUBE;
    bbox_marker.action = visualization_msgs::Marker::ADD;

    int marker_id = 0;
    int track_num = X.size();
    for (int i=0; i<track_num; ++i)
    {
        if(track_info[i].confidence < FUSION_MIN_CONFIDENCE) continue;
        if (!IsConverged(i))  continue;

        bbox_marker.id = marker_id++;
        // switch(track_info[i].type){
        //     case VEHICLE : bbox_marker.type = visualization_msgs::Marker::CUBE;     break;
        //     case PED     : bbox_marker.type = visualization_msgs::Marker::CYLINDER; break;
        //     default      : bbox_marker.type = visualization_msgs::Marker::CUBE;     break;
        // }
        bbox_marker.pose.position.x = X[i](0);
        bbox_marker.pose.position.y = X[i](1);
        bbox_marker.pose.position.z = -0.9;
        bbox_marker.scale.x = PED_WIDTH;
        bbox_marker.scale.y = track_info[i].width;
        bbox_marker.scale.z = PED_HEIGHT;
        marker_array.markers.push_back(bbox_marker);
    }

    if (marker_array.markers.size() > pre_marker_size_)
    {
        pre_marker_size_ = marker_array.markers.size();
    }

    for (int i = marker_id; i < pre_marker_size_; ++i)
    {
        bbox_marker.id = i;
        bbox_marker.action = visualization_msgs::Marker::DELETE;
        marker_array.markers.push_back(bbox_marker);
    }
    pre_marker_size_ = marker_id;
    fusion_pub.publish(marker_array);
}