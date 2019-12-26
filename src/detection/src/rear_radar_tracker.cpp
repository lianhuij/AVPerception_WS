#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "detection/rear_radar_tracker.h"

extern ros::Publisher left_radar_rviz_pub, right_radar_rviz_pub,
                      left_radar_pub, right_radar_pub;
extern RearRadarTracker* left_radar_tracker_ptr;
extern RearRadarTracker* right_radar_tracker_ptr;
extern std::string FIXED_FRAME;
extern float Y_OFFSET;

RearRadarTracker::RearRadarTracker(void)
{
    matched_pair.clear();
    prev_matched.clear();
    src_matched.clear();
    track_info.clear();
    X.clear();
    X_.clear();
    P.clear();
    P_.clear();
    
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
}

void RearRadarTracker::PubRadarTracks(void)
{
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker bbox_marker;
    bbox_marker.header.frame_id = FIXED_FRAME;
    bbox_marker.header.stamp = time_stamp;
    if(this == left_radar_tracker_ptr){
      bbox_marker.color.r = 0.0f;
      bbox_marker.color.g = 0.0f;
      bbox_marker.color.b = 1.0f;    //left radar color red
    }
    if(this == right_radar_tracker_ptr){
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
        if(track_info[i].confidence < RADAR_MIN_CONFIDENCE) continue;
        if (!IsConverged(i))  continue;

        bbox_marker.id = marker_id;
        bbox_marker.pose.position.x = X[i](0);
        if(this == left_radar_tracker_ptr){
          bbox_marker.pose.position.y = X[i](1) + Y_OFFSET;
        }
        if(this == right_radar_tracker_ptr){
          bbox_marker.pose.position.y = X[i](1) - Y_OFFSET;
        }
        bbox_marker.pose.position.z = 0;
        bbox_marker.scale.x = PED_WIDTH;
        bbox_marker.scale.y = PED_WIDTH;
        bbox_marker.scale.z = PED_HEIGHT;
        marker_array.markers.push_back(bbox_marker);
        target.rx = X[i](0);
        if(this == left_radar_tracker_ptr){
          target.ry = X[i](1) + Y_OFFSET;
        }
        if(this == right_radar_tracker_ptr){
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
    if(this == left_radar_tracker_ptr){
      left_radar_rviz_pub.publish(marker_array);
      left_radar_pub.publish(target_array);
    }
    if(this == right_radar_tracker_ptr){
      right_radar_rviz_pub.publish(marker_array);
      right_radar_pub.publish(target_array);
    }
}