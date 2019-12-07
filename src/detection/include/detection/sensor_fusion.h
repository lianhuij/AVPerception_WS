#ifndef DETECTION_SENSOR_FUSION_H
#define DETECTION_SENSOR_FUSION_H

#include <ros/ros.h>
#include "detection/object.h"
#include "detection/GNN.h"

const int FUSION_MIN_CONFIDENCE   = 7;
const int FUSION_MAX_CONFIDENCE   = 100;
const float LOCAL_SINGLE_WEIGHT   = 0.001;
const float FUSION_NEWOBJ_WEIGHT  = 0.00001;
const float CAMERA_TRACK_GATE     = 2.5;
const float RX_TRACK_GATE         = 1.5;
const float RY_TRACK_GATE         = 1.0;

class SensorFusion    //fusion at lidar time
{
public:
    SensorFusion(void);
    ~SensorFusion();

    void Run(void);
    void GetLocalTracks(void);
    void InitTrack(const std::pair<int, int>& pair);
    void Predict(void);
    void MatchGNN(void);
    void Update(void);
    void RemoveTrack(int index);
    bool IsConverged(int track_index);
    void PubFusionTracks(void);
    ObjectType GetCameraType(const std::pair<int, int>& pair);

private:
    std::vector<vector6d> X;  // rx ry vx vy ax ay
    std::vector<matrix6d> P;
    std::vector<ObjectInfo> track_info;
    matrix6d F;
    matrix6d Q;
    std::vector<LocalTrack> radar_tracks;
    std::vector<LocalTrack> lidar_tracks;
    std::vector<LocalTrack> camera_tracks;

    std::vector<std::pair<int, int> > local_matched_pair;  // <radar, lidar>
    std::vector<bool> radar_matched;
    std::vector<bool> lidar_matched;
    std::vector<std::pair<int, int> > matched_pair;
    std::vector<bool> prev_matched;
    std::vector<bool> src_matched;

    double ts;
    ros::Time time_stamp;
    ros::Time prev_stamp;
};

#endif // DETECTION_SENSOR_FUSION_H