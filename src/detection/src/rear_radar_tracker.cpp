#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "detection/rear_radar_tracker.h"
#include "detection/rear_radar_fusion.h"

extern ros::Publisher left_radar_rviz_pub, right_radar_rviz_pub,
                      left_radar_pub, right_radar_pub;
extern RearRadarTracker left_radar_tracker;
extern RearRadarTracker right_radar_tracker;
extern RearRadarFusion  radar_fusion_tracker;
extern std::string FIXED_FRAME;
extern float Y_OFFSET;
extern int RADAR_MIN_CONFIDENCE;
extern int RADAR_MAX_CONFIDENCE;
extern float RADAR_CLUSTER_EPS;
extern float RADAR_CLUSTER_MINPTS;
extern float RADAR_NEWOBJ_WEIGHT;
extern float R_GATE;
extern float THETA_GATE;
extern float VT_GATE;

RearRadarTracker::RearRadarTracker(void)
{
    matched_pair.clear();
    prev_matched.clear();
    src_matched.clear();
    track_info.clear();
    X.clear();
    P.clear();
    
    pre_marker_size_ = 0;
    init_P = matrix6d::Zero(6,6);
    init_P(0,0) = 1;    init_P(1,1) = 1;
    init_P(2,2) = 16;   init_P(3,3) = 16;
    init_P(4,4) = 16;   init_P(5,5) = 16;
    F = matrix6d::Zero(6,6);
    F(0,0) = 1;         F(1,1) = 1;
    F(2,2) = 1;         F(3,3) = 1;
    F(4,4) = 1;         F(5,5) = 1;
    Q = matrix6d::Zero(6,6);
    Q(0,0) = 0.01;      Q(1,1) = 0.01;
    Q(2,2) = 0.05;      Q(3,3) = 0.05;
    Q(4,4) = 0.15;      Q(5,5) = 0.15;
    Hp = matrix2_6d::Zero(2,6);
    Hp(0,0) = 1;  Hp(1,1) = 1;
    R = matrix3d::Zero(3,3);
    R(0,0) = 0.1225;    // 0.35^2
    R(1,1) = 0.0012;    // (2/180*pi)^2
    R(2,2) = 0.1225;    // 0.35^2
}

RearRadarTracker::~RearRadarTracker() { }

void RearRadarTracker::CMKF(const raw_data::RadarRawArray& input)
{
    // clock_t start = clock();
    std::vector<Point> vec_pts;
    for (int i=0; i<input.num; ++i){
        vec_pts.push_back({input.data[i].x, input.data[i].y, 0, NOT_CLASSIFIED});
    }
    DBSCAN dbScan(RADAR_CLUSTER_EPS, RADAR_CLUSTER_MINPTS, vec_pts);    //原始目标聚类
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
    if(this == &left_radar_tracker){
        radar_fusion_tracker.Run();     //fusion at left radar time
    }
}

void RearRadarTracker::InitTrack(const RadarObject &obj)
{
    vector6d init_X = vector6d::Zero(6);
    init_X(0) = obj.r * cos(obj.theta);
    init_X(1) = obj.r * sin(obj.theta);
    X.push_back(init_X);
    P.push_back(init_P);

    ObjectInfo init_info;
    track_info.push_back(init_info);
}

void RearRadarTracker::Predict(void)
{
    if(X.size() != P.size()){
      ROS_ERROR("radar tracker error: Predict state size not equal");
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

void RearRadarTracker::MatchGNN(std::vector<RadarObject> &src)
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

            if (fabs(r - r_2) < R_GATE && fabs(theta - theta_) < THETA_GATE && fabs(vt - vt_) < VT_GATE) // track gate
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
                // std::cout<<"w_ij(i, j) = "<< w_ij(i, j) <<std::endl;
            }else{
                w_ij(i, j) = 0;
                // std::cout<<fabs(r - r_2)<<" "<< fabs(theta - theta_) <<" "<<fabs(vt - vt_) <<std::endl;
            }
        }
    }

    // weights for initializing new filters
    for ( int j = prev_track_num; j < prev_track_num + src_obj_num; ++j ){
        w_ij(j - prev_track_num, j) = RADAR_NEWOBJ_WEIGHT;
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
            if (track_info[e.y].confidence > RADAR_MAX_CONFIDENCE) track_info[e.y].confidence = RADAR_MAX_CONFIDENCE;
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
                // std::cout<<"confidence = "<<track_info[i].confidence<<" "<<X[i](0) <<" "<<X[i](1)<< std::endl;
                RemoveTrack(i);
            }
        }
    }
}

void RearRadarTracker::Update(std::vector<RadarObject> &src)
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
        int src_idx  = matched_pair[i].first;
        int prev_idx = matched_pair[i].second;
        float r     = src[src_idx].r;
        float theta = src[src_idx].theta;
        float vt    = src[src_idx].vt;

        matrix2d Rp;
        Rp(0,0) = r*r*e_2*(pow(cos(theta),2)*(a_2 -a_1)+ pow(sin(theta),2)*(b_2 -b_1))+
                  sigmar2*e_2*(pow(cos(theta),2)*(2*a_2 -a_1)+ pow(sin(theta),2)*(2*b_2 -b_1));
        Rp(1,1) = r*r*e_2*(pow(sin(theta),2)*(a_2 -a_1)+ pow(cos(theta),2)*(b_2 -b_1))+
                  sigmar2*e_2*(pow(sin(theta),2)*(2*a_2 -a_1)+ pow(cos(theta),2)*(2*b_2 -b_1));
        Rp(0,1) = Rp(1,0) = sin(theta)*cos(theta)*e_4*(sigmar2+ (r*r+sigmar2)*(1-exp(sigma2)));
        matrix2d Sp   = Hp*P[prev_idx]*Hp.transpose() + Rp;
        matrix6_2d Kp = P[prev_idx]*Hp.transpose()*Sp.inverse();
        vector2d zp(r*cos(theta), r*sin(theta));
        vector2d up(r*cos(theta)*(exp(-sigma2)- exp(-0.5*sigma2)), r*sin(theta)*(exp(-sigma2)- exp(-0.5*sigma2)));
        X[prev_idx] = X[prev_idx] + Kp*(zp - up - Hp*X[prev_idx]);
        P[prev_idx] = (matrix6d::Identity(6,6) - Kp * Hp) * P[prev_idx];

        double Rx_ = sigmar2*vt*cos(theta)*exp(-sigma2);
        double Ry_ = sigmar2*vt*sin(theta)*exp(-sigma2);
        double R_  = r*r*sigmav2*+ sigmar2*vt*vt+ 3*sigmar2*sigmav2;
        matrix1_2d Rp_;
        Rp_(0,0) = Rx_;  Rp_(0,1) = Ry_;
        matrix1_2d Lk = -Rp_*Rp.inverse();
        double Ra_ = R_ - Rp_*Rp.inverse()*Rp_.transpose();
        vector6d He = vector6d::Zero(6,1);
        He(0) = Lk(0,0)+X[prev_idx](2);
        He(1) = Lk(0,1)+X[prev_idx](3);
        He(2) = X[prev_idx](0);
        He(3) = X[prev_idx](1);
        double Ak = P[prev_idx](0,0)*P[prev_idx](2,2)+ P[prev_idx](1,1)*P[prev_idx](3,3)+ 
                    2*P[prev_idx](0,1)*P[prev_idx](2,3)+ 2*P[prev_idx](0,3)*P[prev_idx](1,2)+ 
                    P[prev_idx](0,2)*P[prev_idx](0,2)+ P[prev_idx](1,3)*P[prev_idx](1,3);
        double Se = He.transpose()*P[prev_idx]*He + Ra_ + Ak;
        vector6d Ke = P[prev_idx]*He/Se;
        X[prev_idx] = X[prev_idx] + Ke*(Lk(0,0)*r*cos(theta)+ Lk(0,1)*r*sin(theta)+ r*vt - Lk(0,0)*up(0)- Lk(0,1)*up(1)-
                                            Lk(0,0)*X[prev_idx](0)- Lk(0,1)*X[prev_idx](1)- X[prev_idx](0)*X[prev_idx](2)
                                            - X[prev_idx](1)*X[prev_idx](3)- (P[prev_idx](0,2)+ P[prev_idx](1,3)));
        P[prev_idx] = (matrix6d::Identity(6,6) - Ke * He.transpose()) * P[prev_idx];
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

void RearRadarTracker::RemoveTrack(int index)
{
    erase_from_vector(X, index);
    erase_from_vector(P, index);
    erase_from_vector(track_info, index);
}

bool RearRadarTracker::IsConverged(int track_index)
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
    }else{
        // std::cout<<"not converged "<<vx_cov<<" "<<vy_cov<<" "<<ax_cov<<" "<<ay_cov<<std::endl;
    }
    return converged;
}

void RearRadarTracker::PubRadarTracks(void)
{
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker bbox_marker;
    bbox_marker.header.frame_id = FIXED_FRAME;
    bbox_marker.header.stamp = time_stamp;
    if(this == &left_radar_tracker){
      bbox_marker.color.r = 0.0f;
      bbox_marker.color.g = 0.0f;
      bbox_marker.color.b = 1.0f;    //left radar color red
    }
    if(this == &right_radar_tracker){
      bbox_marker.color.r = 1.0f;    //right radar color red
      bbox_marker.color.g = 0.0f;
      bbox_marker.color.b = 0.0f;
    }
    bbox_marker.color.a = 0.5f;
    bbox_marker.lifetime = ros::Duration();
    bbox_marker.frame_locked = true;
    bbox_marker.type = visualization_msgs::Marker::CUBE;
    bbox_marker.action = visualization_msgs::Marker::ADD;
    detection::TargetArray target_array;
    detection::Target target;
    target_array.header.stamp = time_stamp;

    int marker_id = 0;
    int track_num = X.size();
    for (int i=0; i<track_num; ++i)
    {
        // if(track_info[i].confidence < RADAR_MIN_CONFIDENCE) continue;
        // if (!IsConverged(i))  continue;

        bbox_marker.id = marker_id;
        bbox_marker.pose.position.x = X[i](0);
        if(this == &left_radar_tracker){
          bbox_marker.pose.position.y = X[i](1) + Y_OFFSET;
        }
        if(this == &right_radar_tracker){
          bbox_marker.pose.position.y = X[i](1) - Y_OFFSET;
        }
        bbox_marker.pose.position.z = 0;
        bbox_marker.scale.x = PED_WIDTH;
        bbox_marker.scale.y = PED_WIDTH;
        bbox_marker.scale.z = PED_HEIGHT;
        marker_array.markers.push_back(bbox_marker);
        target.rx = X[i](0);
        if(this == &left_radar_tracker){
          target.ry = X[i](1) + Y_OFFSET;
        }
        if(this == &right_radar_tracker){
          target.ry = X[i](1) - Y_OFFSET;
        }
        target.vx = X[i](2);
        target.vy = X[i](3);
        target.ax = X[i](4);
        target.ay = X[i](5);
        target.rx_cov = P[i](0,0);
        target.ry_cov = P[i](1,1);
        target.vx_cov = P[i](2,2);
        target.vy_cov = P[i](3,3);
        target.ax_cov = P[i](4,4);
        target.ay_cov = P[i](5,5);
        target.width  = track_info[i].width;
        target.type   = track_info[i].type;
        target_array.data[marker_id] = target;
        marker_id++;
    }
    target_array.num = marker_id;

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
    if(this == &left_radar_tracker){
      left_radar_rviz_pub.publish(marker_array);
      left_radar_pub.publish(target_array);
    }
    if(this == &right_radar_tracker){
      right_radar_rviz_pub.publish(marker_array);
      right_radar_pub.publish(target_array);
    }
}

void RearRadarTracker::GetTimeStamp(ros::Time& stamp){
    stamp = time_stamp;
}

void RearRadarTracker::GetRadarTrack(std::vector<LocalTrack>& tracks){
    tracks.clear();
    LocalTrack track;
    int size = X.size();
    for(int i=0; i<size; ++i){
        // if(track_info[i].confidence < RADAR_MIN_CONFIDENCE) continue;
        // if (!IsConverged(i))  continue;
        track.X  = X[i];
        track.P  = P[i];
        tracks.push_back(track);
    }
}