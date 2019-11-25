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
    init_P(0,0) = 5;
    init_P(1,1) = 5;
    init_P(2,2) = 100;
    init_P(3,3) = 100;
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
    double eps = 0.85;
    int min_pts = 0;
    DBSCAN dbScan(eps, min_pts, vec_pts);    //原始目标聚类
    dbScan.run();
    std::vector<std::vector<int> > idx;
    idx = dbScan.getCluster();

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

    TrackCount init_info({0,1,0,0,0});
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
    src_matched.resize(src_obj_num, 0);
    prev_matched.clear();
    prev_matched.resize(prev_track_num, 0);

    for (int i=0; i<src_obj_num; ++i)
    {
        RadarObject robj = src[i];
        float r = robj.r;
        float theta = robj.theta;
        float vt = robj.vt;
        float rx_ = r*cos(theta);
        float ry_ = r*sin(theta);

        bool find_match = 0;
        int match_id = -1;
        float cost = FLT_MAX;

        for (int j=0; j<prev_track_num; ++j)
        {
            float rx = X[j](0);
            float ry = X[j](1);
            float vx = X[j](2);
            float vy = X[j](3);

            if (fabs(rx - rx_) < 1.5
                && fabs(ry - ry_) < 1)
            {
                float tmp_cost = fabs(rx-rx_) + fabs(ry-ry_);
                if (tmp_cost < cost)
                {
                    cost = tmp_cost;
                    find_match = 1;
                    match_id = j;
                }
            }
        }

        if (find_match)
        {
            if (!prev_matched[match_id])
            {
                std::pair<int, int> assignment(i, match_id);
                matched_pair.push_back(assignment);
                prev_matched[match_id] = 1;
                src_matched[i] = 1;

                // track_info[match_id].loss_cnt = 0;
                // if(track_info[match_id].exist_cnt <100) track_info[match_id].exist_cnt++;
                track_info[match_id].confi_dec = 0;
                track_info[match_id].confi_inc++;
                track_info[match_id].confidence += log(track_info[match_id].confi_inc + 1) / log(1.5f);
                if (track_info[i].confidence > 100) track_info[i].confidence = 100;
            }
            else
            {
                int new_match_id = X.size();
                Eigen::Vector4f X_copy(X[match_id]);
                Eigen::Matrix4f P_copy(P[match_id]);
                X.push_back(X_copy);
                P.push_back(P_copy);

                TrackCount track_info_copy({0,1,0,0,0});
                track_info.push_back(track_info_copy);

                std::pair<int, int> assignment(i, new_match_id);
                matched_pair.push_back(assignment);
                prev_matched.push_back(1);
                src_matched[i] = 1;
            }
        }
    }

    prev_track_num = X.size();
    src_obj_num = src.size();

    for (int i=0; i<prev_track_num; ++i)
    {
        if (!prev_matched[i])
        {
            // track_info[i].loss_cnt++;
            track_info[i].confi_inc = 0;
            track_info[i].confi_dec++;
            track_info[i].confidence -= pow(1.5f, track_info[i].confi_dec);
            if (track_info[i].confidence < -10) track_info[i].confidence = -10;
        }
    }

    // printf("[MatchNN][matched_track: %zu][unmatched_track: %d][unmatched_obj: %d]\n",
    //        matched_pair.size(), unmatched_track_num, unmatched_obj_num);
}

void RadarTracker::Update(std::vector<RadarObject> &src)
{
    if(X.size() != P.size()){
      ROS_ERROR("radar tracker error: Update state size not equal");
    }

    // a. upgrade matched
    for (int i=0; i<matched_pair.size(); ++i)
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
        double d2 = Y.transpose() * S.inverse() * Y;
        std::cout << "d2 = " << d2 << std::endl;
        double gamma = 2*log(0.95/(0.05*0.9*15.75*sqrt(S.determinant())));
        std::cout << "gamma = " << gamma << std::endl;

        X[prev_index] = X[prev_index] + K * Y;
        P[prev_index] = (Eigen::Matrix4f::Identity(4,4) - K * H) * P[prev_index];
    }

    // b. upgrade prev unmatched
    // TODO: according to loss count
    int prev_track_num = X.size();
    for (int i=prev_track_num-1; i>=0; --i)  // from back to front to avoid wrongly removing
    {
        if (!prev_matched[i])
        {
            // if (track_info[i].loss_cnt > max_loss_cnt
            //     || !IsConverged(i))
            if(track_info[i].confidence < 0 || !IsConverged(i))
            {
                RemoveTrack(i);
            }
        }
    }

    // c. upgrade src unmatched
    int src_obj_num = src.size();
    for (int i=0; i<src_obj_num; ++i)
    {
        if (!src_matched[i])
        {
            InitTrack(src[i]);
        }
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
        // if (track_info[i].exist_cnt < min_exist_cnt)  continue;
        if(track_info[i].confidence < 15) continue;
        if (!IsConverged(i))  continue;

        bbox_marker.id = marker_id++;
        bbox_marker.pose.position.x = X[i](0) + x_offset;;
        bbox_marker.pose.position.y = X[i](1);
        bbox_marker.pose.position.z = -0.9;
        bbox_marker.scale.x = 0.6;
        bbox_marker.scale.y = 0.6;
        bbox_marker.scale.z = 1.7;
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