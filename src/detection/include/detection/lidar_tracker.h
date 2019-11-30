#ifndef DETECTION_LIDAR_TRACKER_H
#define DETECTION_LIDAR_TRACKER_H

#include <ros/ros.h>
#include <detection/LidarRawArray.h>
#include "detection/object.h"
#include "detection/GNN.h"

const int lidar_min_confidence   = 7;
const int lidar_max_confidence   = 100;
const float lidar_newobj_weight  = 0.01;
const float lidar_rx_gate  = 0.8;   // rx 0.8m, ry 0.8m
const float lidar_ry_gate  = 0.8;

class LidarTracker
{
public:
    LidarTracker();
    ~LidarTracker();

    void KF(const detection::LidarRawArray& input);
    void InitTrack(const LidarObject &obj);
    void Predict();
    void MatchGNN(const std::vector<LidarObject>& src);
    void Update(const std::vector<LidarObject>& src);
    void RemoveTrack(int index);
    bool IsConverged(int track_index);
    void PubLidarTracks();

private:
    std::vector<vector6d> X;  // rx ry vx vy ax ay
    std::vector<matrix6d> P;
    std::vector<TrackCount> track_info;
    matrix6d init_P;
    matrix6d F;
    matrix6d Q;
    matrix2_6d H;
    matrix2d R;

    std::vector<std::pair<int, int> > matched_pair;
    std::vector<bool> prev_matched;
    std::vector<bool> src_matched;

    double ts;
    ros::Time time_stamp;
};

#endif // DETECTION_LIDAR_TRACKER_H