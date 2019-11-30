#ifndef DETECTION_CAMERA_TRACKER_H
#define DETECTION_CAMERA_TRACKER_H

#include <ros/ros.h>
#include <raw_data/CameraRawArray.h>
#include "detection/object.h"
#include "detection/GNN.h"

const int CAM_MIN_CONFIDENCE   = 7;
const int CAM_MAX_CONFIDENCE   = 30;
const float CAM_NEWOBJ_WEIGHT  = 0.01;
const float CAM_RX_GATE  = 2.0;   // rx 2.0m, ry 1.0m
const float CAM_RY_GATE  = 1.0;

class CameraTracker
{
public:
    CameraTracker();
    ~CameraTracker();

    void KF(const raw_data::CameraRawArray& input);
    void InitTrack(const CameraObject &obj);
    void Predict();
    void MatchGNN(const std::vector<CameraObject>& src);
    void Update(const std::vector<CameraObject>& src);
    void RemoveTrack(int index);
    bool IsConverged(int track_index);
    void PubCameraTracks();

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

#endif // DETECTION_CAMERA_TRACKER_H