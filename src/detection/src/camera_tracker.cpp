#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "detection/camera_tracker.h"

extern ros::Publisher cam_filtered_pub;
extern std::string fixed_frame;
extern float x_offset;

CameraTracker::CameraTracker()
{
    matched_pair.clear();
    prev_matched.clear();
    src_matched.clear();
    track_info.clear();
    X.clear();
    P.clear();
    
    ts = 0.08;
    time_stamp = ros::Time::now();
    init_P = matrix6d::Zero(6,6);
    init_P(0,0) = 1;    init_P(1,1) = 1;
    init_P(2,2) = 16;   init_P(3,3) = 16;
    init_P(4,4) = 16;   init_P(5,5) = 16;
    F = matrix6d::Zero(6,6);
    F(0,0) = 1;         F(1,1) = 1;
    F(2,2) = 1;         F(3,3) = 1;
    F(4,4) = 1;         F(5,5) = 1;
    Q = matrix6d::Zero(6,6);
    Q(0,0) = 0.2;       Q(1,1) = 0.2;
    Q(2,2) = 0.5;       Q(3,3) = 0.5;
    Q(4,4) = 0.9;       Q(5,5) = 0.9;
    H = matrix3_6d::Zero(3,6);
    H(0,0) = 1;
    H(1,1) = 1;
    H(2,2) = 1;
    R = matrix3d::Zero(3,3);
    R(0,0) = 4;       // 2^2
    R(1,1) = 1;       // 1^2
    R(2,2) = 1;       // 1^2
}

CameraTracker::~CameraTracker() { }

void CameraTracker::KF(const raw_data::CameraRawArray& input)
{
    // clock_t start = clock();
    std::vector<CameraObject> src;
    CameraObject raw;
    for(int i=0; i<input.num; ++i){
        raw.rx = input.data[i].x;
        raw.ry = input.data[i].y;
        raw.vx = input.data[i].vx;
        raw.target_type = input.data[i].target_type;
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
    PubCameraTracks();
}

void CameraTracker::InitTrack(const CameraObject &obj)
{
    vector6d init_X = vector6d::Zero(6);
    init_X(0) = obj.rx;
    init_X(1) = obj.ry;
    init_X(2) = obj.vx;
    X.push_back(init_X);
    P.push_back(init_P);

    TrackCount init_info({0,0,0});
    track_info.push_back(init_info);
}

void CameraTracker::Predict()
{
    if(X.size() != P.size()){
      ROS_ERROR("camera tracker error: Predict state size not equal");
    }
    int prev_track_num = X.size();
    for (int i=0; i<prev_track_num; ++i)
    {
        F(0,2) = F(1,3) = F(2,4) = F(3,5) = ts;
        F(0,4) = F(1,5) = ts*ts/2;
        X[i] = F * X[i];
        P[i] = F * P[i] * F.transpose() + Q;
    }
}

void CameraTracker::MatchGNN(std::vector<CameraObject> &src)
{
    int prev_track_num = X.size();
    int src_obj_num = src.size();

    matched_pair.clear();
    src_matched.clear();
    src_matched.resize(src_obj_num, false);
    prev_matched.clear();
    prev_matched.resize(prev_track_num, false);

    Eigen::MatrixXd w_ij(src_obj_num, prev_track_num + src_obj_num);
    w_ij = Eigen::MatrixXd::Zero(src_obj_num, prev_track_num + src_obj_num);

    // get likelihoods of measurements within track pdfs
    for ( int i = 0; i < src_obj_num; ++i )
    {
        float rx = src[i].rx;
        float ry = src[i].ry;
        float vx = src[i].vx;
        vector3d z(rx, ry, vx);
        for ( int j = 0; j < prev_track_num; ++j ){
            float rx_ = X[j](0);
            float ry_ = X[j](1);
            float vx_ = X[j](2);

            if (fabs(rx - rx_) < cam_rx_gate && fabs(ry - ry_) < cam_ry_gate && fabs(vx - vx_) < cam_vx_gate) // track gate
            {
                vector3d z_(rx_, ry_, vx_);
                matrix3d S = H * P[j] * H.transpose() + R;
                w_ij(i, j) = normalDistributionDensity< 3 >(S, z_, z);
                std::cout << "w_ij(i, j) = " << w_ij(i, j) <<std::endl;
            }else{
                w_ij(i, j) = 0;
            }
        }
    }

    // weights for initializing new filters
    for ( int j = prev_track_num; j < prev_track_num + src_obj_num; ++j ){
        w_ij(j - prev_track_num, j) = cam_newobj_weight;
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
            if (track_info[e.y].confidence > cam_max_confidence) track_info[e.y].confidence = cam_max_confidence;
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

void CameraTracker::Update(std::vector<CameraObject> &src)
{
    if(X.size() != P.size()){
      ROS_ERROR("camera tracker error: Update state size not equal");
    }
    
    for (int i=0; i<matched_pair.size(); ++i)    // upgrade matched
    {
        int src_index = matched_pair[i].first;
        int prev_index = matched_pair[i].second;

        float rx_ = X[prev_index](0);
        float ry_ = X[prev_index](1);
        float vx_ = X[prev_index](2);

        float rx = src[src_index].rx;
        float ry = src[src_index].ry;
        float vx = src[src_index].vx;

        vector3d Y(rx-rx_, ry-ry_, vx-vx_);
        matrix3d S = H * P[prev_index] * H.transpose() + R;
        matrix6_3d K = matrix6_3d::Zero(6,3);
        K = P[prev_index] * H.transpose() * S.inverse();

        X[prev_index] = X[prev_index] + K * Y;
        P[prev_index] = (matrix6d::Identity(6,6) - K * H) * P[prev_index];
    }
}

template <class T>
static void erase_from_vector(std::vector<T> &v, int index)
{
    if(index >= v.size()){
      ROS_ERROR("camera tracker error: remove track index >= size");
      return;
    }
    v.erase(v.begin() + index);
}

void CameraTracker::RemoveTrack(int index)
{
    erase_from_vector(X, index);
    erase_from_vector(P, index);
    erase_from_vector(track_info, index);
}

bool CameraTracker::IsConverged(int track_index)
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

void CameraTracker::PubCameraTracks()
{
    static int max_marker_size_ = 0;
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker bbox_marker;
    bbox_marker.header.frame_id = fixed_frame;
    bbox_marker.header.stamp = time_stamp;
    bbox_marker.color.r = 0.0f;
    bbox_marker.color.g = 0.0f;
    bbox_marker.color.b = 1.0f;    //camera color blue
    bbox_marker.color.a = 0.5;
    bbox_marker.lifetime = ros::Duration();
    bbox_marker.frame_locked = true;
    bbox_marker.type = visualization_msgs::Marker::CUBE;
    bbox_marker.action = visualization_msgs::Marker::ADD;

    int marker_id = 0;
    int track_num = X.size();
    for (int i=0; i<track_num; ++i)
    {
        if(track_info[i].confidence < cam_min_confidence) continue;
        if (!IsConverged(i))  continue;

        bbox_marker.id = marker_id++;
        bbox_marker.pose.position.x = X[i](0) + x_offset;   // add offset, convert to velodyne frame
        bbox_marker.pose.position.y = X[i](1);
        bbox_marker.pose.position.z = -0.9;
        bbox_marker.scale.x = ped_width;
        bbox_marker.scale.y = ped_width;
        bbox_marker.scale.z = ped_height;
        marker_array.markers.push_back(bbox_marker);
    }

    if (marker_array.markers.size() > max_marker_size_)
    {
        max_marker_size_ = marker_array.markers.size();
    }

    for (int i = marker_id; i < max_marker_size_; ++i)
    {
        bbox_marker.id = i;
        bbox_marker.color.a = 0;
        bbox_marker.pose.position.x = 0;
        bbox_marker.pose.position.y = 0;
        bbox_marker.pose.position.z = 0;
        bbox_marker.scale.x = 0.1;
        bbox_marker.scale.y = 0.1;
        bbox_marker.scale.z = 0.1;
        marker_array.markers.push_back(bbox_marker);
    }
    cam_filtered_pub.publish(marker_array);
}