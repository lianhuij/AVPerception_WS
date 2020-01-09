#ifndef DETECTION_REAR_RADAR_TRACKER_H
#define DETECTION_REAR_RADAR_TRACKER_H

#include <ros/ros.h>
#include <raw_data/RadarRawArray.h>
#include <detection/Target.h>
#include <detection/TargetArray.h>
#include "detection/object.h"
#include "detection/dbscan.h"
#include "detection/GNN.h"

class RearRadarTracker
{
public:
    RearRadarTracker(void);
    ~RearRadarTracker();

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

    std::vector<vector6d> X;  // rx ry vx vy ax ay
    std::vector<matrix6d> P;
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
    int pre_marker_size_;
};

#endif // DETECTION_REAR_RADAR_TRACKER_H