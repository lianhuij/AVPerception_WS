#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
// #include "detection/radar_tracker.h"
#include "detection/radar_cmkf_tracker.h"
#include "detection/camera_tracker.h"
#include "detection/lidar_tracker.h"
#include "detection/sensor_fusion.h"

ros::Subscriber radar_sub, camera_sub, lidar_sub;                           // ROS Subscriber
ros::Publisher radar_ekf_pub, radar_cmkf_pub, camera_kf_pub, lidar_kf_pub,  // ROS Publisher
               fusion_pub;
std::string FIXED_FRAME;
float X_OFFSET;

// RadarTracker radar_tracker;
// RadarTracker* radar_tracker_ptr = &radar_tracker;
RadarCMKFTracker radar_tracker;
RadarCMKFTracker* radar_tracker_ptr = &radar_tracker;
CameraTracker camera_tracker;
CameraTracker* camera_tracker_ptr = &camera_tracker;
LidarTracker lidar_tracker;
LidarTracker* lidar_tracker_ptr = &lidar_tracker;
SensorFusion fusion_tracker;

int main(int argc,char** argv)
{
    ros::init(argc,argv,"tracker_node");
    ros::NodeHandle nh;
    nh.getParam("/tracker_node/fixed_frame", FIXED_FRAME);
    nh.getParam("/tracker_node/x_offset", X_OFFSET);

    // radar_sub  = nh.subscribe("radar_rawArray", 10, &RadarTracker::EKF, radar_tracker_ptr);
    radar_sub  = nh.subscribe("radar_rawArray", 10, &RadarCMKFTracker::CMKF, radar_tracker_ptr);
    camera_sub = nh.subscribe("camera_rawArray", 10, &CameraTracker::KF, camera_tracker_ptr);
    lidar_sub  = nh.subscribe("lidar_rawArray", 10, &LidarTracker::KF, lidar_tracker_ptr);
    // radar_ekf_pub = nh.advertise<visualization_msgs::MarkerArray>("radar_ekf_rviz", 10);
    radar_cmkf_pub = nh.advertise<visualization_msgs::MarkerArray>("radar_cmkf_rviz", 10);
    camera_kf_pub  = nh.advertise<visualization_msgs::MarkerArray>("camera_kf_rviz", 10);
    lidar_kf_pub   = nh.advertise<visualization_msgs::MarkerArray>("lidar_kf_rviz", 10);
    
    ros::spin();
    return 0;
}