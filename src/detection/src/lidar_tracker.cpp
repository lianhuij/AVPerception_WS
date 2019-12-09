#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "detection/lidar_tracker.h"
#include "detection/sensor_fusion.h"

extern ros::Publisher lidar_kf_pub;
extern std::string FIXED_FRAME;
extern SensorFusion fusion_tracker;

LidarTracker::LidarTracker(void)
{
    matched_pair.clear();
    prev_matched.clear();
    src_matched.clear();
    track_info.clear();
    X.clear();
    P.clear();
    
    init_P = matrix6d::Zero(6,6);
    init_P(0,0) = 1;    init_P(1,1) = 1;
    init_P(2,2) = 16;   init_P(3,3) = 16;
    init_P(4,4) = 16;   init_P(5,5) = 16;
    F = matrix6d::Zero(6,6);
    F(0,0) = 1;         F(1,1) = 1;
    F(2,2) = 1;         F(3,3) = 1;
    Q = matrix6d::Zero(6,6);
    H = matrix2_6d::Zero(2,6);
    H(0,0) = 1;
    H(1,1) = 1;
    R = matrix2d::Zero(2,2);
    R(0,0) = 0.0225;       // 0.15^2
    R(1,1) = 0.0225;       // 0.15^2
}

LidarTracker::~LidarTracker() { }

void LidarTracker::KF(const detection::LidarRawArray& input)
{
    // clock_t start = clock();
    std::vector<LidarObject> src;
    LidarObject raw;
    for(int i=0; i<input.num; ++i){
        raw.rx = input.data[i].x;
        raw.ry = input.data[i].y;
        raw.width = input.data[i].width;
        src.push_back(raw);
    }

    if (!X.size())
    {
      for (auto &obj : src)
      {
          InitTrack(obj);
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
    PubLidarTracks();
    fusion_tracker.Run();    //fusion point at lidar received time
}

void LidarTracker::InitTrack(const LidarObject &obj)
{
    vector6d init_X = vector6d::Zero(6);
    init_X(0) = obj.rx;
    init_X(1) = obj.ry;
    X.push_back(init_X);
    P.push_back(init_P);

    ObjectInfo init_info(UNKNOWN, obj.width);
    track_info.push_back(init_info);
}

void LidarTracker::Predict(void)
{
    if(X.size() != P.size()){
      ROS_ERROR("lidar tracker error: Predict state size not equal");
    }
    float ax_ = ALPHA_X*ts;
    float ay_ = ALPHA_Y*ts;
    float bx_ = exp(-ALPHA_X*ts);
    float by_ = exp(-ALPHA_Y*ts);
    F(0,2) = F(1,3) = ts;
    F(2,4) = (1 - bx_)/ALPHA_X;  F(3,5) = (1 - by_)/ALPHA_Y;
    F(0,4) = (ax_ + bx_ -1)/(ALPHA_X*ALPHA_X);  F(1,5) = (ay_ + by_ -1)/(ALPHA_Y*ALPHA_Y);
    F(4,4) = bx_;  F(5,5) = by_;
    matrix6_2d U = matrix6_2d::Zero(6,2);
    U(0,0) = (0.5*ax_*ts + (1- bx_)/ALPHA_X -ts)/ALPHA_X;
    U(1,0) = ts - (1- bx_)/ALPHA_X;
    U(2,0) = 1- bx_;
    U(3,1) = (0.5*ay_*ts + (1- by_)/ALPHA_Y -ts)/ALPHA_Y;
    U(4,1) = ts - (1- by_)/ALPHA_Y;
    U(5,1) = 1- by_;
    vector2d A;
    float sigmax2 = (4-M_PI)/M_PI; 
    float sigmay2 = sigmax2;
    float qx11 = (1- bx_*bx_ +2*ax_ +2*pow(ax_,3)/3 -2*ax_*ax_ -4*ax_*bx_)/(2*pow(ALPHA_X,5));
    float qx12 = (1+ bx_*bx_ -2*bx_ +2*ax_*bx_ -2*ax_ +ax_*ax_)/(2*pow(ALPHA_X,4));
    float qx13 = (1- bx_*bx_ -2*ax_*bx_)/(2*pow(ALPHA_X,3));
    float qx22 = (4*bx_ -3 -bx_*bx_ +2*ax_)/(2*pow(ALPHA_X,3));
    float qx23 = (bx_*bx_ +1 -2*bx_)/(2*pow(ALPHA_X,2));
    float qx33 = (1- bx_*bx_)/(2*ALPHA_X);
    float qy11 = (1- by_*by_ +2*ay_ +2*pow(ay_,3)/3 -2*ay_*ay_ -4*ay_*by_)/(2*pow(ALPHA_Y,5));
    float qy12 = (1+ by_*by_ -2*by_ +2*ay_*by_ -2*ay_ +ay_*ay_)/(2*pow(ALPHA_Y,4));
    float qy13 = (1- by_*by_ -2*ay_*by_)/(2*pow(ALPHA_Y,3));
    float qy22 = (4*by_ -3 -by_*by_ +2*ay_)/(2*pow(ALPHA_Y,3));
    float qy23 = (by_*by_ +1 -2*by_)/(2*pow(ALPHA_Y,2));
    float qy33 = (1- by_*by_)/(2*ALPHA_Y);
    float qx_ = 2*ALPHA_X;
    float qy_ = 2*ALPHA_Y;
    int prev_track_num = X.size();
    for (int i=0; i<prev_track_num; ++i)
    {
        A(0) = X[i](4);  A(1) = X[i](5);
        sigmax2 *= pow(MAX_ACC - fabs(A(0)), 2);
        sigmay2 *= pow(MAX_ACC - fabs(A(1)), 2);
        qx_ *= sigmax2;
        qy_ *= sigmay2;
        qx11 *= qx_; qx12 *= qx_; qx13 *= qx_; qx22 *= qx_; qx23 *= qx_; qx33 *= qx_;
        qy11 *= qy_; qy12 *= qy_; qy13 *= qy_; qy22 *= qy_; qy23 *= qy_; qy33 *= qy_;
        Q << qx11, 0,    qx12, 0,    qx13, 0,
             0,    qy11, 0,    qy12, 0,    qy13,
             qx12, 0,    qx22, 0,    qx23, 0,
             0,    qy12, 0,    qy22, 0,    qy23,
             qx13, 0,    qx23, 0,    qx33, 0,
             0,    qy13, 0,    qy23, 0,    qy33;
        X[i] = F * X[i] + U * A;
        P[i] = F * P[i] * F.transpose() + Q;
    }
}

void LidarTracker::MatchGNN(const std::vector<LidarObject>& src)
{
    int prev_track_num = X.size();
    int src_obj_num = src.size();

    matched_pair.clear();
    src_matched.clear();
    src_matched.resize(src_obj_num, false);
    prev_matched.clear();
    prev_matched.resize(prev_track_num, false);

    matrixXd w_ij = matrixXd::Zero(src_obj_num, prev_track_num + src_obj_num);

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

void LidarTracker::Update(const std::vector<LidarObject>& src)
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
      ROS_ERROR("lidar tracker error: remove track index >= size");
      return;
    }
    v.erase(v.begin() + index);
}

void LidarTracker::RemoveTrack(int index)
{
    erase_from_vector(X, index);
    erase_from_vector(P, index);
    erase_from_vector(track_info, index);
}

bool LidarTracker::IsConverged(int track_index)
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

void LidarTracker::PubLidarTracks(void)
{
    static int pre_marker_size_ = 0;
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker bbox_marker;
    bbox_marker.header.frame_id = FIXED_FRAME;
    bbox_marker.header.stamp = time_stamp;
    bbox_marker.color.r = 0.0f;
    bbox_marker.color.g = 1.0f;    //lidar color green
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

void LidarTracker::GetTimeStamp(ros::Time& stamp){
    stamp = time_stamp;
}

void LidarTracker::GetLidarTrack(std::vector<LocalTrack>& tracks){
    tracks.clear();
    LocalTrack track;
    int size = X.size();
    for(int i=0; i<size; ++i){
        if(track_info[i].confidence < LIDAR_MIN_CONFIDENCE) continue;
        if (!IsConverged(i))  continue;
        track.X = X[i];
        track.P = P[i];
        track.width = track_info[i].width;
        tracks.push_back(track);
    }
}