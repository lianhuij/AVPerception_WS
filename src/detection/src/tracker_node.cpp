#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <detection/TargetArray.h>
// #include "detection/radar_tracker.h"
#include "detection/radar_cmkf_tracker.h"
#include "detection/camera_tracker.h"
#include "detection/lidar_tracker.h"
#include "detection/sensor_fusion.h"

ros::Subscriber radar_sub, camera_sub, lidar_sub;                           // ROS Subscriber
ros::Publisher radar_ekf_pub, radar_cmkf_pub, camera_kf_pub, lidar_kf_pub,  // ROS Publisher
               fusion_od_pub, radar_pub, lidar_pub, fusion_pub;
std::string FIXED_FRAME;
float X_OFFSET;
int LIDAR_MIN_CONFIDENCE;
int LIDAR_MAX_CONFIDENCE;
float LIDAR_NEWOBJ_WEIGHT;
float LIDAR_RX_GATE; 
float LIDAR_RY_GATE;
int RADAR_MIN_CONFIDENCE;
int RADAR_MAX_CONFIDENCE;
float RADAR_CLUSTER_EPS;
float RADAR_CLUSTER_MINPTS;
float RADAR_NEWOBJ_WEIGHT;
float R_GATE; 
float THETA_GATE;
float VT_GATE;
int CAM_MIN_CONFIDENCE;
int CAM_MAX_CONFIDENCE;
float CAM_NEWOBJ_WEIGHT;
float CAM_RX_GATE; 
float CAM_RY_GATE;
int FUSION_MIN_CONFIDENCE;
int FUSION_MAX_CONFIDENCE;
float LOCAL_SINGLE_WEIGHT;
float FUSION_NEWOBJ_WEIGHT;
float CAMERA_TRACK_GATE;
float RX_TRACK_GATE;
float RY_TRACK_GATE;

// RadarTracker radar_tracker;
RadarCMKFTracker radar_tracker;
CameraTracker camera_tracker;
LidarTracker lidar_tracker;
SensorFusion fusion_tracker;

int main(int argc,char** argv)
{
    ros::init(argc,argv,"tracker_node");
    ros::NodeHandle nh;
    nh.param<std::string>("/tracker_node/fixed_frame", FIXED_FRAME, "velodyne");
    nh.param<float>("/tracker_node/x_offset", X_OFFSET, 1.6);
    nh.param<int>("/tracker_node/LIDAR_MIN_CONFIDENCE", LIDAR_MIN_CONFIDENCE, 7);
    nh.param<int>("/tracker_node/LIDAR_MAX_CONFIDENCE", LIDAR_MAX_CONFIDENCE, 100);
    nh.param<float>("/tracker_node/LIDAR_NEWOBJ_WEIGHT", LIDAR_NEWOBJ_WEIGHT, 0.01);
    nh.param<float>("/tracker_node/LIDAR_RX_GATE", LIDAR_RX_GATE, 1.0);  // rx 1.0m, ry 1.0m
    nh.param<float>("/tracker_node/LIDAR_RY_GATE", LIDAR_RY_GATE, 1.0);
    nh.param<int>("/tracker_node/RADAR_MIN_CONFIDENCE", RADAR_MIN_CONFIDENCE, 22);
    nh.param<int>("/tracker_node/RADAR_MAX_CONFIDENCE", RADAR_MAX_CONFIDENCE, 60);
    nh.param<float>("/tracker_node/RADAR_CLUSTER_EPS", RADAR_CLUSTER_EPS, 0.85);
    nh.param<float>("/tracker_node/RADAR_CLUSTER_MINPTS", RADAR_CLUSTER_MINPTS, 0);
    nh.param<float>("/tracker_node/RADAR_NEWOBJ_WEIGHT", RADAR_NEWOBJ_WEIGHT, 0.1);
    nh.param<float>("/tracker_node/R_GATE", R_GATE, 1.0);  // r 1.0m, theta 5deg, vt 1.0m/s
    nh.param<float>("/tracker_node/THETA_GATE", THETA_GATE, 0.08);
    nh.param<float>("/tracker_node/VT_GATE", VT_GATE, 1.0);
    nh.param<int>("/tracker_node/CAM_MIN_CONFIDENCE", CAM_MIN_CONFIDENCE, 7);
    nh.param<int>("/tracker_node/CAM_MAX_CONFIDENCE", CAM_MAX_CONFIDENCE, 30);
    nh.param<float>("/tracker_node/CAM_NEWOBJ_WEIGHT", CAM_NEWOBJ_WEIGHT, 0.01);
    nh.param<float>("/tracker_node/CAM_RX_GATE", CAM_RX_GATE, 2.0);  // rx 2.0m, ry 1.0m
    nh.param<float>("/tracker_node/CAM_RY_GATE", CAM_RY_GATE, 1.0);
    nh.param<int>("/tracker_node/FUSION_MIN_CONFIDENCE", FUSION_MIN_CONFIDENCE, 7);
    nh.param<int>("/tracker_node/FUSION_MAX_CONFIDENCE", FUSION_MAX_CONFIDENCE, 100);
    nh.param<float>("/tracker_node/LOCAL_SINGLE_WEIGHT", LOCAL_SINGLE_WEIGHT, 0.001);
    nh.param<float>("/tracker_node/FUSION_NEWOBJ_WEIGHT", FUSION_NEWOBJ_WEIGHT, 0.00001);
    nh.param<float>("/tracker_node/CAMERA_TRACK_GATE", CAMERA_TRACK_GATE, 2.5);
    nh.param<float>("/tracker_node/RX_TRACK_GATE", RX_TRACK_GATE, 1.5);  // rx 1.5m, ry 1.0m
    nh.param<float>("/tracker_node/RY_TRACK_GATE", RY_TRACK_GATE, 1.0);

    // radar_sub  = nh.subscribe("radar_rawArray", 10, &RadarTracker::EKF, &radar_tracker);
    radar_sub  = nh.subscribe("radar_rawArray", 10, &RadarCMKFTracker::CMKF, &radar_tracker);
    camera_sub = nh.subscribe("camera_rawArray", 10, &CameraTracker::KF, &camera_tracker);
    lidar_sub  = nh.subscribe("lidar_rawArray", 10, &LidarTracker::KF, &lidar_tracker);
    // radar_ekf_pub = nh.advertise<visualization_msgs::MarkerArray>("radar_ekf_rviz", 10);
    radar_cmkf_pub = nh.advertise<visualization_msgs::MarkerArray>("radar_cmkf_rviz", 10);
    camera_kf_pub  = nh.advertise<visualization_msgs::MarkerArray>("camera_kf_rviz", 10);
    lidar_kf_pub   = nh.advertise<visualization_msgs::MarkerArray>("lidar_kf_rviz", 10);
    fusion_od_pub  = nh.advertise<visualization_msgs::MarkerArray>("fusion_rviz", 10);
    // radar_pub      = nh.advertise<detection::TargetArray>("radar_ekf_array", 10);
    radar_pub      = nh.advertise<detection::TargetArray>("radar_array", 10);
    lidar_pub      = nh.advertise<detection::TargetArray>("lidar_array", 10);
    fusion_pub     = nh.advertise<detection::TargetArray>("fusion_array", 10);
    
    ros::spin();
    return 0;
}