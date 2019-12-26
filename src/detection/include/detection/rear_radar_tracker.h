#ifndef DETECTION_REAR_RADAR_TRACKER_H
#define DETECTION_REAR_RADAR_TRACKER_H

#include <ros/ros.h>
#include <raw_data/RadarRawArray.h>
#include <detection/Target.h>
#include <detection/TargetArray.h>
#include "detection/object.h"
#include "detection/dbscan.h"
#include "detection/GNN.h"
#include "detection/radar_cmkf_tracker.h"

class RearRadarTracker : public RadarCMKFTracker
{
public:
    RearRadarTracker(void);
    ~RearRadarTracker();

    void CMKF(const raw_data::RadarRawArray& input);
    void PubRadarTracks(void);
    int pre_marker_size_;
};

#endif // DETECTION_REAR_RADAR_TRACKER_H