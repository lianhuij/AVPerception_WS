#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "detection/radar_cmkf_tracker.h"

extern ros::Publisher radar_cmkf_pub;
extern std::string fixed_frame;
extern float x_offset;

RadarCMKFTracker::RadarCMKFTracker()
{
    matched_pair.clear();
    prev_matched.clear();
    src_matched.clear();
    track_info.clear();
    X.clear();
    P.clear();
    
    ts = 0.05;
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
    Q(0,0) = 0.05;      Q(1,1) = 0.05;
    Q(2,2) = 0.1;       Q(3,3) = 0.1;
    Q(4,4) = 0.3;       Q(5,5) = 0.3;
    Hp = matrix2_6d::Zero(2,6);
    Hp(0,0) = 1;  Hp(1,1) = 1;
    R = matrix3d::Zero(3,3);
    R(0,0) = 0.1225;    // 0.35^2
    R(1,1) = 0.0012;    // (2/180*pi)^2
    R(2,2) = 0.1225;    // 0.35^2
}

RadarCMKFTracker::~RadarCMKFTracker() { }

void RadarCMKFTracker::CMKF(const raw_data::RadarRawArray& input)
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
        int size = idx[i].size();
        for(int j=0; j<size; ++j){
            raw.r     = raw.r + input.data[idx[i][j]].distance;
            raw.theta = raw.theta + input.data[idx[i][j]].angle *M_PI/180;
            raw.vt    = raw.vt + input.data[idx[i][j]].speed;
        }
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
      ts = (input.header.stamp - time_stamp).toSec();
      time_stamp = input.header.stamp;
      Predict();
      MatchGNN(src);
      Update(src);
    }
    // clock_t end = clock();
    // float duration_ms = (float)(end-start)*1000/(float)CLOCKS_PER_SEC;  //程序用时 ms
    // std::cout << "duration(ms) = " << duration_ms << std::endl;
    PubRadarTracks();
}

void RadarCMKFTracker::InitTrack(const RadarObject &obj)
{
    vector6d init_X = vector6d::Zero(6);
    init_X(0) = obj.r * cos(obj.theta);
    init_X(1) = obj.r * sin(obj.theta);
    X.push_back(init_X);
    P.push_back(init_P);

    TrackCount init_info({0,0,0});
    track_info.push_back(init_info);
}

void RadarCMKFTracker::Predict()
{
    if(X.size() != P.size()){
      ROS_ERROR("radar tracker error: Predict state size not equal");
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

void RadarCMKFTracker::MatchGNN(std::vector<RadarObject> &src)
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
        vector3d z(r, theta, vt);
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
                vector3d z_(r_2, theta_, vt_);
                matrix3_6d H = matrix3_6d::Zero(3,6);
                H(0,0) = rx / r_2;
                H(0,1) = ry / r_2;
                H(1,0) = -ry / r_1;
                H(1,1) = rx / r_1;
                H(2,0) = -ry * (rx * vy - ry * vx) / r_3;
                H(2,1) = rx * (rx * vy - ry * vx) / r_3;
                H(2,2) = rx / r_2;
                H(2,3) = ry / r_2;
                matrix3d S = H * P[j] * H.transpose() + R;
                w_ij(i, j) = normalDistributionDensity< 3 >(S, z_, z);
            }else{
                w_ij(i, j) = 0;
            }
        }
    }

    // weights for initializing new filters
    for ( int j = prev_track_num; j < prev_track_num + src_obj_num; ++j ){
        w_ij(j - prev_track_num, j) = radar_newobj_weight;
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
            if (track_info[e.y].confidence > radar_max_confidence) track_info[e.y].confidence = radar_max_confidence;
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

void RadarCMKFTracker::Update(std::vector<RadarObject> &src)
{
    if(X.size() != P.size()){
      ROS_ERROR("radar tracker error: Update state size not equal");
    }
    
    double sigmar2 = R(0,0);
    double sigma2  = R(1,1);
    double sigmav2 = R(2,2);
    double e_2 = exp(-2*sigma2);
    double e_4 = pow(e_2,2);
    double a_1 = cosh(sigma2);
    double a_2 = cosh(2*sigma2);
    double b_1 = sinh(sigma2);
    double b_2 = sinh(2*sigma2);
    for (int i=0; i<matched_pair.size(); ++i)    // upgrade matched
    {
        int src_index  = matched_pair[i].first;
        int prev_index = matched_pair[i].second;
        float r     = src[src_index].r;
        float theta = src[src_index].theta;
        float vt    = src[src_index].vt;

        matrix2d Rp;
        Rp(0,0) = r*r*e_2*(pow(cos(theta),2)*(a_2 -a_1)+ pow(sin(theta),2)*(b_2 -b_1))+
                  sigmar2*e_2*(pow(cos(theta),2)*(2*a_2 -a_1)+ pow(sin(theta),2)*(2*b_2 -b_1));
        Rp(1,1) = r*r*e_2*(pow(sin(theta),2)*(a_2 -a_1)+ pow(cos(theta),2)*(b_2 -b_1))+
                  sigmar2*e_2*(pow(sin(theta),2)*(2*a_2 -a_1)+ pow(cos(theta),2)*(2*b_2 -b_1));
        Rp(0,1) = Rp(1,0) = sin(theta)*cos(theta)*e_4*(sigmar2+ (r*r+sigmar2)*(1-exp(sigma2)));
        matrix2d Sp   = Hp*P[prev_index]*Hp.transpose() + Rp;
        matrix6_2d Kp = P[prev_index]*Hp.transpose()*Sp.inverse();
        vector2d zp(r*cos(theta), r*sin(theta));
        vector2d up(r*cos(theta)*(exp(-sigma2)- exp(-0.5*sigma2)), r*sin(theta)*(exp(-sigma2)- exp(-0.5*sigma2)));
        X[prev_index] = X[prev_index] + Kp*(zp - up - Hp*X[prev_index]);
        P[prev_index] = (matrix6d::Identity(6,6) - Kp * Hp) * P[prev_index];

        double Rx_ = sigmar2*vt*cos(theta)*exp(-sigma2);
        double Ry_ = sigmar2*vt*sin(theta)*exp(-sigma2);
        double R_  = r*r*sigmav2*+ sigmar2*vt*vt+ 3*sigmar2*sigmav2;
        matrix1_2d Rp_;
        Rp_(0,0) = Rx_;  Rp_(0,1) = Ry_;
        matrix1_2d Lk = -Rp_*Rp.inverse();
        double Ra_ = R_ - Rp_*Rp.inverse()*Rp_.transpose();
        vector6d He = vector6d::Zero(6,1);
        He(0) = Lk(0,0)+X[prev_index](2);
        He(1) = Lk(0,1)+X[prev_index](3);
        He(2) = X[prev_index](0);
        He(3) = X[prev_index](1);
        double Ak = P[prev_index](0,0)*P[prev_index](2,2)+ P[prev_index](1,1)*P[prev_index](3,3)+ 
                    2*P[prev_index](0,1)*P[prev_index](2,3)+ 2*P[prev_index](0,3)*P[prev_index](1,2)+ 
                    P[prev_index](0,2)*P[prev_index](0,2)+ P[prev_index](1,3)*P[prev_index](1,3);
        double Se = He.transpose()*P[prev_index]*He + Ra_ + Ak;
        vector6d Ke = P[prev_index]*He/Se;
        X[prev_index] = X[prev_index] + Ke*(Lk(0,0)*r*cos(theta)+ Lk(0,1)*r*sin(theta)+ r*vt - Lk(0,0)*up(0)- Lk(0,1)*up(1)-
                                            Lk(0,0)*X[prev_index](0)- Lk(0,1)*X[prev_index](1)- X[prev_index](0)*X[prev_index](2)
                                            - X[prev_index](1)*X[prev_index](3)- (P[prev_index](0,2)+ P[prev_index](1,3)));
        P[prev_index] = (matrix6d::Identity(6,6) - Ke * He.transpose()) * P[prev_index];
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

void RadarCMKFTracker::RemoveTrack(int index)
{
    erase_from_vector(X, index);
    erase_from_vector(P, index);
    erase_from_vector(track_info, index);
}

bool RadarCMKFTracker::IsConverged(int track_index)
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

void RadarCMKFTracker::PubRadarTracks()
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
        bbox_marker.scale.x = 0.1;
        bbox_marker.scale.y = 0.1;
        bbox_marker.scale.z = 0.1;
        marker_array.markers.push_back(bbox_marker);
    }
    radar_cmkf_pub.publish(marker_array);
}