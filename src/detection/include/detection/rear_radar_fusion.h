#ifndef DETECTION_REAR_RADAR_FUSION_H
#define DETECTION_REAR_RADAR_FUSION_H

#include <ros/ros.h>
#include "detection/object.h"

class RearRadarFusion
{
public:
    RearRadarFusion(void);
    ~RearRadarFusion();

    void Run(void);
    void GetLocalTracks(void);
    void PubFusionTracks(void);

private:
    vector6d min[3];  // rx ry vx vy ax ay  0:left, 1:mid, 2:right
    int lost_cnt[3];
    std::vector<LocalTrack> left_radar_tracks;
    std::vector<LocalTrack> right_radar_tracks;
    ros::Time fusion_stamp;
    double ds;
    matrix6d Fs;
    bool first;
};

#endif // DETECTION_REAR_RADAR_FUSION_H