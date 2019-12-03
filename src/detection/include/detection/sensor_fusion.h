#ifndef DETECTION_SENSOR_FUSION_H
#define DETECTION_SENSOR_FUSION_H

#include <ros/ros.h>
#include "detection/object.h"
#include "detection/GNN.h"

const int FUSION_MIN_CONFIDENCE   = 7;
const int FUSION_MAX_CONFIDENCE   = 100;
const float FUSION_NEWOBJ_WEIGHT  = 0.01;

class SensorFusion
{
public:
    SensorFusion(void);
    ~SensorFusion(void);

    void Run(void);
    void GetLocalTracks(void);
    void InitTrack(const LidarObject &obj);
    void Predict(void);
    void MatchGNN(const std::vector<LidarObject>& src);
    void Update(const std::vector<LidarObject>& src);
    void RemoveTrack(int index);
    bool IsConverged(int track_index);
    void PubFusionTracks(void);

private:
    std::vector<vector6d> X;  // rx ry vx vy ax ay
    std::vector<matrix6d> P;
    std::vector<ObjectInfo> track_info;
    matrix6d init_P;
    matrix6d F;
    matrix6d Q;

    std::vector<std::pair<int, int> > local_matched_pair;
    std::vector<bool> radar_matched;
    std::vector<bool> lidar_matched;
    std::vector<std::pair<int, int> > matched_pair;
    std::vector<bool> prev_matched;
    std::vector<bool> src_matched;

    double ts;
    ros::Time time_stamp;
};

#endif // DETECTION_SENSOR_FUSION_H