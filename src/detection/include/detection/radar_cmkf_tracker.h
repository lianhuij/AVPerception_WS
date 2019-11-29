#ifndef DETECTION_RADAR_CMKF_TRACKER_H
#define DETECTION_RADAR_CMKF_TRACKER_H

#include <ros/ros.h>
#include <raw_data/RadarRawArray.h>
#include "detection/object.h"
#include "detection/dbscan.h"
#include "detection/GNN.h"

const int radar_min_confidence   = 13;
const int radar_max_confidence   = 30;
const float radar_cluster_eps    = 0.85;
const float radar_cluster_minPts = 0;
const float radar_newobj_weight  = 0.1;
const float r_gate     = 1.0;   // r 1.0m, theta 5deg, vt 1.0m/s
const float theta_gate = 0.08;
const float vt_gate    = 1.0;

class RadarCMKFTracker
{
public:
    RadarCMKFTracker();
    ~RadarCMKFTracker();

    void CMKF(const raw_data::RadarRawArray& input);
    void InitTrack(const RadarObject &obj);
    void Predict();
    void MatchGNN(std::vector<RadarObject> &src);
    void Update(std::vector<RadarObject> &src);
    void RemoveTrack(int index);
    bool IsConverged(int track_index);
    void PubRadarTracks();

private:
    std::vector<vector6d> X;  // rx ry vx vy ax ay
    std::vector<matrix6d> P;
    std::vector<TrackCount> track_info;
    matrix6d init_P;
    matrix6d F;
    matrix6d Q;
    matrix2_6d Hp;
    matrix3d R;

    std::vector<std::pair<int, int> > matched_pair;
    std::vector<bool> prev_matched;
    std::vector<bool> src_matched;

    double ts;
    ros::Time time_stamp;
};

#endif // DETECTION_RADAR_CMKF_TRACKER_H