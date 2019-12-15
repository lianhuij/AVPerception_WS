#ifndef DETECTION_RADAR_CMKF_TRACKER_H
#define DETECTION_RADAR_CMKF_TRACKER_H

#include <ros/ros.h>
#include <raw_data/RadarRawArray.h>
#include <detection/Target.h>
#include <detection/TargetArray.h>
#include "detection/object.h"
#include "detection/dbscan.h"
#include "detection/GNN.h"

const int RADAR_MIN_CONFIDENCE   = 22;
const int RADAR_MAX_CONFIDENCE   = 60;
const float RADAR_CLUSTER_EPS    = 0.85;
const float RADAR_CLUSTER_MINPTS = 0;
const float RADAR_NEWOBJ_WEIGHT  = 0.1;
const float R_GATE     = 1.0;   // r 1.0m, theta 5deg, vt 1.0m/s
const float THETA_GATE = 0.08;
const float VT_GATE    = 1.0;

class RadarCMKFTracker
{
public:
    RadarCMKFTracker(void);
    ~RadarCMKFTracker();

    void CMKF(const raw_data::RadarRawArray& input);
    void InitTrack(const RadarObject &obj);
    void Predict(void);
    void MatchGNN(std::vector<RadarObject> &src);
    void Update(std::vector<RadarObject> &src);
    void RemoveTrack(int index);
    bool IsConverged(int track_index);
    void PubRadarTracks(void);
    void GetTimeStamp(ros::Time& stamp);
    void GetRadarTrack(std::vector<LocalTrack>& tracks);

private:
    std::vector<vector6d> X, X_;  // rx ry vx vy ax ay
    std::vector<matrix6d> P, P_;
    std::vector<ObjectInfo> track_info;
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