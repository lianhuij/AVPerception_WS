#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <detection/TargetArray.h>
#include "detection/rear_radar_tracker.h"
#include "detection/rear_radar_fusion.h"

ros::Subscriber left_radar_sub, right_radar_sub;                          // ROS Subscriber
ros::Publisher left_radar_rviz_pub, right_radar_rviz_pub, left_radar_pub, // ROS Publisher
               right_radar_pub, radar_fusion_rviz_pub, radar_fusion_pub;
std::string FIXED_FRAME;
float Y_OFFSET;
int RADAR_MIN_CONFIDENCE;
int RADAR_MAX_CONFIDENCE;
float RADAR_CLUSTER_EPS;
float RADAR_CLUSTER_MINPTS;
float RADAR_NEWOBJ_WEIGHT;
float R_GATE;   
float THETA_GATE;
float VT_GATE;
float HALF_LANE_WIDTH;
float RX_GATE;
float RY_GATE;
int MAX_LOST_CNT;

RearRadarTracker left_radar_tracker;
RearRadarTracker right_radar_tracker;
RearRadarFusion  radar_fusion_tracker;

int main(int argc,char** argv)
{
    ros::init(argc,argv,"rear_tracker_node");
    ros::NodeHandle nh;
    nh.param<std::string>("/rear_tracker_node/fixed_frame", FIXED_FRAME, "rear");
    nh.param<float>("/rear_tracker_node/y_offset", Y_OFFSET, 0.20);
    nh.param<int>("/rear_tracker_node/RADAR_MIN_CONFIDENCE", RADAR_MIN_CONFIDENCE, 7);
    nh.param<int>("/rear_tracker_node/RADAR_MAX_CONFIDENCE", RADAR_MAX_CONFIDENCE, 60);
    nh.param<float>("/rear_tracker_node/RADAR_CLUSTER_EPS", RADAR_CLUSTER_EPS, 0.85);
    nh.param<float>("/rear_tracker_node/RADAR_CLUSTER_MINPTS", RADAR_CLUSTER_MINPTS, 0);
    nh.param<float>("/rear_tracker_node/RADAR_NEWOBJ_WEIGHT", RADAR_NEWOBJ_WEIGHT, 0.1);
    nh.param<float>("/rear_tracker_node/R_GATE", R_GATE, 1.0);  // r 1.0m, theta 5deg, vt 1.0m/s
    nh.param<float>("/rear_tracker_node/THETA_GATE", THETA_GATE, 0.08);
    nh.param<float>("/rear_tracker_node/VT_GATE", VT_GATE, 1.0);
    nh.param<float>("/rear_tracker_node/HALF_LANE_WIDTH", HALF_LANE_WIDTH, 1.5);
    nh.param<float>("/rear_tracker_node/RX_GATE", RX_GATE, 1.0);
    nh.param<float>("/rear_tracker_node/RY_GATE", RY_GATE, 0.8);
    nh.param<int>("/rear_tracker_node/MAX_LOST_CNT", MAX_LOST_CNT, 16);

    left_radar_sub  = nh.subscribe("left_radar_rawArray", 10, &RearRadarTracker::CMKF, &left_radar_tracker);
    right_radar_sub = nh.subscribe("right_radar_rawArray", 10, &RearRadarTracker::CMKF, &right_radar_tracker);

    left_radar_rviz_pub = nh.advertise<visualization_msgs::MarkerArray>("left_radar_rviz", 10);
    right_radar_rviz_pub  = nh.advertise<visualization_msgs::MarkerArray>("right_radar_rviz", 10);
    left_radar_pub   = nh.advertise<detection::TargetArray>("left_radar_array", 10);
    right_radar_pub  = nh.advertise<detection::TargetArray>("right_radar_array", 10);
    radar_fusion_rviz_pub  = nh.advertise<visualization_msgs::MarkerArray>("radar_fusion_rviz", 10);
    radar_fusion_pub  = nh.advertise<detection::TargetArray>("radar_fusion_array", 10);
    
    ros::spin();
    return 0;
}