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
    Q(0,0) = 0.0001;     Q(1,1) = 0.0001;
    Q(2,2) = 0.005;      Q(3,3) = 0.005;
    Q(4,4) = 0.01;       Q(5,5) = 0.01;
}

SensorFusion::~SensorFusion(void) { }

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
      time_stamp = input.header.stamp;
    }
    else
    {
      ts = (input.header.stamp - time_stamp).toSec();
      time_stamp = input.header.stamp;
      Predict();
      MatchGNN(src);
      Update(src);
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
            matrix6d C = Pr + Pl;
            w_ij(i, j) = normalDistributionDensity< 6 >(C, Xr, Xl);
            // std::cout << "w_ij(i, j) = " << w_ij(i, j) <<std::endl;
        }
    }

    for ( int j = lidar_track_num; j < lidar_track_num + radar_track_num; ++j ){
        w_ij(j - lidar_track_num, j) = FUSION_NEWOBJ_WEIGHT;
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

void SensorFusion::InitTrack(const std::pair<int, int>& pair)
{
    int radar_index = pair.first;
    int lidar_index = pair.second;
    vector6d init_X = vector6d::Zero(6);
    matrix6d init_P = matrix6d::Zero(6,6);
    if(radar_index == -1){
        init_X = lidar_tracks[lidar_index].X;
        init_P = lidar_tracks[lidar_index].P;
    }else if(lidar_index == -1){
        init_X = radar_tracks[radar_index].X;
        init_P = radar_tracks[radar_index].P;
    }else{
        init_P = (radar_tracks[radar_index].P.inverse() + lidar_tracks[lidar_index].P.inverse()).inverse();
        init_X = init_P * (radar_tracks[radar_index].P.inverse() * radar_tracks[radar_index].X
                           + lidar_tracks[lidar_index].P.inverse() * lidar_tracks[lidar_index].X);
    }
    X.push_back(init_X);
    P.push_back(init_P);
    ObjectInfo init_info(UNKNOWN, obj.width);
    track_info.push_back(init_info);
}

void SensorFusion::Predict()
{
    if(X.size() != P.size()){
      ROS_ERROR("lidar tracker error: Predict state size not equal");
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

void SensorFusion::MatchGNN(const std::vector<LidarObject>& src)
{
    int prev_track_num = X.size();
    int src_obj_num = src.size();

    matched_pair.clear();
    src_matched.clear();
    src_matched.resize(src_obj_num, false);
    prev_matched.clear();
    prev_matched.resize(prev_track_num, false);

    matrixXd w_ij(src_obj_num, prev_track_num + src_obj_num);
    w_ij = matrixXd::Zero(src_obj_num, prev_track_num + src_obj_num);

    // get likelihoods of measurements within track pdfs
    for ( int i = 0; i < src_obj_num; ++i )
    {
        float rx = src[i].rx;
        float ry = src[i].ry;
        vector2d z(rx, ry);
        for ( int j = 0; j < prev_track_num; ++j ){
            float rx_ = X[j](0);
            float ry_ = X[j](1);
            if (fabs(rx - rx_) < LIDAR_RX_GATE && fabs(ry - ry_) < LIDAR_RY_GATE) // track gate
            {
                vector2d z_(rx_, ry_);
                matrix2d S = H * P[j] * H.transpose() + R;
                w_ij(i, j) = normalDistributionDensity< 2 >(S, z_, z);
                // std::cout << "w_ij(i, j) = " << w_ij(i, j) <<std::endl;
            }else{
                w_ij(i, j) = 0;
            }
        }
    }

    // weights for initializing new filters
    for ( int j = prev_track_num; j < prev_track_num + src_obj_num; ++j ){
        w_ij(j - prev_track_num, j) = LIDAR_NEWOBJ_WEIGHT;
    }

    // solve the maximum-sum-of-weights problem (i.e. assignment problem)
    // in this case it is global nearest neighbour by minimizing the distances
    // over all measurement-filter-associations
    Auction<double>::Edges assignments = Auction<double>::solve(w_ij);

    // for all found assignments
    for ( const auto & e : assignments )
    {
        // is assignment an assignment from an already existing filter to a measurement?
        if ( e.y < prev_track_num )
        {
            std::pair<int, int> pair(e.x, e.y);  // (measurement, predict)
            matched_pair.push_back(pair);
            src_matched[e.x] = true;
            prev_matched[e.y] = true;

            track_info[e.y].confi_dec = 0;    // target matched, confidence increase
            track_info[e.y].confi_inc++;
            track_info[e.y].confidence += log(track_info[e.y].confi_inc + 1) / log(1.5f);
            if (track_info[e.y].confidence > LIDAR_MAX_CONFIDENCE) track_info[e.y].confidence = LIDAR_MAX_CONFIDENCE;
        }
        else // is this assignment a measurement that is considered new?
        {
            // create filter with measurment and keep it
            InitTrack(src[e.x]);
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

void SensorFusion::Update(const std::vector<LidarObject>& src)
{
    if(X.size() != P.size()){
      ROS_ERROR("lidar tracker error: Update state size not equal");
    }
    
    for (int i=0; i<matched_pair.size(); ++i)    // upgrade matched
    {
        int src_index = matched_pair[i].first;
        int prev_index = matched_pair[i].second;

        float rx_ = X[prev_index](0);
        float ry_ = X[prev_index](1);

        float rx = src[src_index].rx;
        float ry = src[src_index].ry;

        vector2d Y(rx-rx_, ry-ry_);
        matrix2d S = H * P[prev_index] * H.transpose() + R;
        matrix6_2d K = matrix6_2d::Zero(6,2);
        K = P[prev_index] * H.transpose() * S.inverse();

        X[prev_index] = X[prev_index] + K * Y;
        P[prev_index] = (matrix6d::Identity(6,6) - K * H) * P[prev_index];
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
    bbox_marker.color.r = 0.0f;
    bbox_marker.color.g = 1.0f;    //lidar color green
    bbox_marker.color.b = 0.0f;
    bbox_marker.color.a = 0.5;
    bbox_marker.lifetime = ros::Duration();
    bbox_marker.frame_locked = true;
    bbox_marker.type = visualization_msgs::Marker::CUBE;
    bbox_marker.action = visualization_msgs::Marker::ADD;

    int marker_id = 0;
    int track_num = X.size();
    for (int i=0; i<track_num; ++i)
    {
        if(track_info[i].confidence < LIDAR_MIN_CONFIDENCE) continue;
        if (!IsConverged(i))  continue;

        bbox_marker.id = marker_id++;
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
    lidar_kf_pub.publish(marker_array);
}