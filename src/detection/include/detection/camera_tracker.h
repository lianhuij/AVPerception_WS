#ifndef DETECTION_CAMERA_TRACKER_H
#define DETECTION_CAMERA_TRACKER_H

#include <ros/ros.h>
#include <raw_data/CameraRawArray.h>
#include "detection/object.h"
#include "detection/GNN.h"

const int cam_min_confidence   = 7;
const int cam_max_confidence   = 30;
const float cam_newobj_weight  = 0.01;
const float cam_rx_gate  = 2.0;   // rx 2.0m, ry 1.0m
const float cam_ry_gate  = 1.0;

class CameraTracker
{
public:
    CameraTracker();
    ~CameraTracker();

    void KF(const raw_data::CameraRawArray& input);
    void InitTrack(const CameraObject &obj);
    void Predict();
    void MatchGNN(std::vector<CameraObject> &src);
    void Update(std::vector<CameraObject> &src);
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