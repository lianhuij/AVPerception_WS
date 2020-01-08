#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <detection/TargetArray.h>
#include "detection/radar_cmkf_tracker.h"
#include "detection/rear_radar_tracker.h"

ros::Subscriber left_radar_sub, right_radar_sub;                  // ROS Subscriber
ros::Publisher left_radar_rviz_pub, right_radar_rviz_pub,         // ROS Publisher
               left_radar_pub, right_radar_pub, radar_fusion_pub,
               radar_cmkf_pub, radar_pub;
std::string FIXED_FRAME;
float Y_OFFSET;
float X_OFFSET;

RearRadarTracker left_radar_tracker;
RearRadarTracker* left_radar_tracker_ptr = &left_radar_tracker;
RearRadarTracker right_radar_tracker;
RearRadarTracker* right_radar_tracker_ptr = &right_radar_tracker;

int main(int argc,char** argv)
{
    ros::init(argc,argv,"rear_tracker_node");
    ros::NodeHandle nh;
    nh.getParam("/rear_tracker_node/fixed_frame", FIXED_FRAME);
    nh.getParam("/rear_tracker_node/y_offset", Y_OFFSET);

    left_radar_sub  = nh.subscribe("left_radar_rawArray", 10, &RearRadarTracker::CMKF, left_radar_tracker_ptr);
    right_radar_sub = nh.subscribe("right_radar_rawArray", 10, &RearRadarTracker::CMKF, right_radar_tracker_ptr);

    left_radar_rviz_pub = nh.advertise<visualization_msgs::MarkerArray>("left_radar_rviz", 10);
    right_radar_rviz_pub  = nh.advertise<visualization_msgs::MarkerArray>("right_radar_rviz", 10);
    left_radar_pub   = nh.advertise<detection::TargetArray>("left_radar_array", 10);
    right_radar_pub  = nh.advertise<detection::TargetArray>("right_radar_array", 10);
    
    ros::spin();
    return 0;
}