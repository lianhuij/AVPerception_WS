#ifndef DETECTION_RADAR_TRACKER_H
#define DETECTION_RADAR_TRACKER_H

#include <ros/ros.h>
#include <raw_data/RadarRawArray.h>
#include "detection/object.h"
#include "detection/dbscan.h"

const int max_loss_cnt = 5;
const int min_exist_cnt = 5;

class RadarTracker
{
public:
    RadarTracker();
    ~RadarTracker();

    void EKF(const raw_data::RadarRawArray& input);
    void InitTrack(const RadarObject &obj);
    void Predict();
    void MatchNN(std::vector<RadarObject> &src);
    void Update(std::vector<RadarObject> &src);
    void RemoveTrack(int index);
    bool IsConverged(int track_index);
    void PubRadarTracks();

private:
    std::vector<Eigen::Vector4f> X;  // rx ry vx vy
    std::vector<Eigen::Matrix4f> P;
    std::vector<TrackCount> track_info;
    Eigen::Matrix4f init_P;
    Eigen::Matrix4f F;
    Eigen::Matrix4f Q;
    Eigen::Matrix3f R;

    std::vector<std::pair<int, int> > matched_pair;
    std::vector<bool> prev_matched;
    std::vector<bool> src_matched;

    float ts;
    ros::Time time_stamp;
};

#endif // DETECTION_RADAR_TRACKER_H