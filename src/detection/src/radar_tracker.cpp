#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "detection/radar_tracker.h"

extern ros::Publisher radar_filtered_pub;
extern std::string fixed_frame;
extern float x_offset;

RadarTracker::RadarTracker()
{
    matched_pair.clear();
    prev_matched.clear();
    src_matched.clear();
    track_info.clear();
    X.clear();
    P.clear();
    
    ts = 0.05;
    time_stamp = ros::Time::now();
    init_P = Eigen::Matrix4f::Zero(4,4);
    init_P(0,0) = 1;
    init_P(1,1) = 1;
    init_P(2,2) = 16;
    init_P(3,3) = 16;
    F = Eigen::Matrix4f::Zero(4,4);
    F(0,0) = 1;
    F(1,1) = 1;
    F(2,2) = 1;
    F(3,3) = 1;
//  Q << dt_4/4*noise_ax,   0,                dt_3/2*noise_ax,  0,
//       0,                 dt_4/4*noise_ay,  0,                dt_3/2*noise_ay,
//       dt_3/2*noise_ax,   0,                dt_2*noise_ax,    0,
//       0,                 dt_3/2*noise_ay,  0,                dt_2*noise_ay;
    Q << 0.00001, 0,       0.0003, 0,
         0,       0.00001, 0,      0.0003,
         0.0003,  0,       0.01,   0,
         0,       0.0003,  0,      0.01;
    R = Eigen::Matrix3f::Zero(3,3);
    R(0,0) = 0.0625;    // 0.25^2
    R(1,1) = 0.0003;    // (1/180*pi)^2
    R(2,2) = 0.0144;    // 0.12^2
}

RadarTracker::~RadarTracker() { }

void RadarTracker::EKF(const raw_data::RadarRawArray& input)
{
    // clock_t start = clock();
    std::vector<Point> vec_pts;
    for (int i=0; i<input.num; ++i){
        vec_pts.push_back({input.data[i].x, input.data[i].y, 0, NOT_CLASSIFIED});
    }
    double eps = radar_cluster_eps;
    int min_pts = radar_cluster_minPts;
    DBSCAN dbScan(eps, min_pts, vec_pts);    //原始目标聚类
    dbScan.run();
    std::vector<std::vector<int> > idx = dbScan.getCluster();

    std::vector<RadarObject> src;
    RadarObject raw;
    for(int i=0; i<idx.size(); ++i){
        raw.r = raw.theta = raw.vt = 0;
        for(int j=0; j<idx[i].size(); ++j){
            raw.r     = raw.r + input.data[idx[i][j]].distance;
            raw.theta = raw.theta + input.data[idx[i][j]].angle *M_PI/180;
            raw.vt    = raw.vt + input.data[idx[i][j]].speed;
        }
        int size = idx[i].size();
        raw.r     /= size;
        raw.theta /= size;
        raw.vt    /= size;
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
      ts = (float)((input.header.stamp - time_stamp).toSec());
      time_stamp = input.header.stamp;
      Predict();
      MatchNN(src);
      Update(src);
    }
    // clock_t end = clock();
    // float duration_ms = (float)(end-start)*1000/(float)CLOCKS_PER_SEC;  //程序用时 ms
    // std::cout << "duration(ms) = " << duration_ms << std::endl;
    PubRadarTracks();
}

void RadarTracker::InitTrack(const RadarObject &obj)
{
    Eigen::Vector4f init_X = Eigen::Vector4f::Zero(4);
    init_X(0) = obj.r * cos(obj.theta);
    init_X(1) = obj.r * sin(obj.theta);
    init_X(2) = 0;
    init_X(3) = 0;
    X.push_back(init_X);
    P.push_back(init_P);

    TrackCount init_info({0,0,0});
    track_info.push_back(init_info);
}

void RadarTracker::Predict()
{
    if(X.size() != P.size()){
      ROS_ERROR("radar tracker error: Predict state size not equal");
    }
    int prev_track_num = X.size();
    for (int i=0; i<prev_track_num; ++i)
    {
        F(0,2) = ts;
        F(1,3) = ts;
        X[i] = F * X[i];
        P[i] = F * P[i] * F.transpose() + Q;
    }
}

void RadarTracker::MatchNN(std::vector<RadarObject> &src)
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
        float r     = src[i].r;
        float theta = src[i].theta;
        float vt    = src[i].vt;
        Eigen::Vector3f z(r, theta, vt);
        for ( int j = 0; j < prev_track_num; ++j ){
            float rx = X[j](0);
            float ry = X[j](1);
            float vx = X[j](2);
            float vy = X[j](3);
            float r_1 = rx * rx + ry * ry;
            float r_2 = sqrt(r_1);
            float r_3 = r_1 * r_2;
            float theta_ = atan2(ry, rx);
            float vt_ = (vx * rx + vy * ry) / r_2;

            if (fabs(r - r_2) < r_gate && fabs(theta - theta_) < theta_gate && fabs(vt - vt_) < vt_gate) // track gate
            {
                Eigen::Vector3f z_(r_2, theta_, vt_);
                Eigen::Matrix<float,3,4> H = Eigen::Matrix<float,3,4>::Zero(3,4);
                H(0,0) = rx / r_2;
                H(0,1) = ry / r_2;
                H(1,0) = -ry / r_1;
                H(1,1) = rx / r_1;
                H(2,0) = -ry * (rx * vy - ry * vx) / r_3;
                H(2,1) = rx * (rx * vy - ry * vx) / r_3;
                H(2,2) = rx / r_2;
                H(2,3) = ry / r_2;
                Eigen::Matrix3f S = H * P[j] * H.transpose() + R;
                w_ij(i, j) = normalDistributionDensity<3>(S, z_, z);
            }else{
                w_ij(i, j) = 0;
            }
        }
    }

    // weights for initializing new filters
    for ( int j = prev_track_num; j < prev_track_num + src_obj_num; ++j ){
        w_ij(j - prev_track_num, j) = 1e-6;
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
            if (track_info[e.y].confidence > 100) track_info[e.y].confidence = 100;
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

void RadarTracker::Update(std::vector<RadarObject> &src)
{
    if(X.size() != P.size()){
      ROS_ERROR("radar tracker error: Update state size not equal");
    }
    
    for (int i=0; i<matched_pair.size(); ++i)    // upgrade matched
    {
        int src_index = matched_pair[i].first;
        int prev_index = matched_pair[i].second;

        float rx = X[prev_index](0);
        float ry = X[prev_index](1);
        float vx = X[prev_index](2);
        float vy = X[prev_index](3);
        float r_1 = rx * rx + ry * ry;
        float r_2 = sqrt(r_1);
        float r_3 = r_1 * r_2;
        float theta_ = atan2(ry, rx);
        float vt_ = (vx * rx + vy * ry) / r_2;

        float r = src[src_index].r;
        float theta = src[src_index].theta;
        float vt = src[src_index].vt;

        Eigen::Vector3f Y(r-r_2, theta-theta_, vt-vt_);

        Eigen::Matrix<float,3,4> H = Eigen::Matrix<float,3,4>::Zero(3,4);
        H(0,0) = rx / r_2;
        H(0,1) = ry / r_2;
        H(1,0) = -ry / r_1;
        H(1,1) = rx / r_1;
        H(2,0) = -ry * (rx * vy - ry * vx) / r_3;
        H(2,1) = rx * (rx * vy - ry * vx) / r_3;
        H(2,2) = rx / r_2;
        H(2,3) = ry / r_2;
        Eigen::Matrix3f S = H * P[prev_index] * H.transpose() + R;
        Eigen::Matrix<float,4,3> K = P[prev_index] * H.transpose() * S.inverse();

        X[prev_index] = X[prev_index] + K * Y;
        P[prev_index] = (Eigen::Matrix4f::Identity(4,4) - K * H) * P[prev_index];
    }
}

template <class T>
static void erase_from_vector(std::vector<T> &v, int index)
{
    if(index >= v.size()){
      ROS_ERROR("radar tracker error: remove track index >= size");
      return;
    }
    v.erase(v.begin() + index);
}

void RadarTracker::RemoveTrack(int index)
{
    erase_from_vector(X, index);
    erase_from_vector(P, index);
    erase_from_vector(track_info, index);
}

bool RadarTracker::IsConverged(int track_index)
{
    bool converged = false;
    float rx_cov = P[track_index](0,0);
    float ry_cov = P[track_index](1,1);
    float vx_cov = P[track_index](2,2);
    float vy_cov = P[track_index](3,3);
    if (rx_cov < 5
        && ry_cov < 5
        && vx_cov < 25
        && vy_cov < 25)
    {
        converged = true;
    }
    return converged;
}

void RadarTracker::PubRadarTracks()
{
    static int max_marker_size_ = 0;
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker bbox_marker;
    bbox_marker.header.frame_id = fixed_frame;
    bbox_marker.header.stamp = time_stamp;
    bbox_marker.color.r = 1.0f;    //radar color red
    bbox_marker.color.g = 0.0f;
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
        if(track_info[i].confidence < radar_min_confidence) continue;
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
        bbox_marker.scale.x = 0;
        bbox_marker.scale.y = 0;
        bbox_marker.scale.z = 0;
        marker_array.markers.push_back(bbox_marker);
    }
    radar_filtered_pub.publish(marker_array);
}