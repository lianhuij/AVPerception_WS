#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
// #include "detection/radar_tracker.h"
#include "detection/radar_cmkf_tracker.h"
#include "detection/camera_tracker.h"

ros::Publisher radar_filtered_pub, radar_cmkf_pub, cam_filtered_pub;  // ROS Publisher
std::string fixed_frame;
float x_offset;

int main(int argc,char** argv)
{
    ros::init(argc,argv,"tracker_node");
    ros::NodeHandle nh;
    nh.getParam("/tracker_node/fixed_frame", fixed_frame);
    nh.getParam("/tracker_node/x_offset", x_offset);
    // RadarTracker radar_tracker;
    // RadarTracker* radar_tracker_ptr = &radar_tracker;
    RadarCMKFTracker radar_tracker;
    RadarCMKFTracker* radar_tracker_ptr = &radar_tracker;
    CameraTracker camera_tracker;
    CameraTracker* cam_tracker_ptr = &camera_tracker;

    ros::Subscriber radar_sub  = nh.subscribe("radar_rawArray", 10, &RadarCMKFTracker::CMKF, radar_tracker_ptr);
    ros::Subscriber camera_sub = nh.subscribe("cam_rawArray", 10, &CameraTracker::KF, cam_tracker_ptr);
    radar_filtered_pub = nh.advertise<visualization_msgs::MarkerArray>("radar_filtered", 10);
    cam_filtered_pub   = nh.advertise<visualization_msgs::MarkerArray>("cam_filtered", 10);
    
    ros::spin();
    return 0;
}