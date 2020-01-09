#include <ros/ros.h>
#include "detection/ultrasonic_filter.h"

ros::Subscriber left_ultrasonic_sub, right_ultrasonic_sub;   // ROS Subscriber
ros::Publisher left_ultrasonic_pub, right_ultrasonic_pub;    // ROS Publisher

UltrasonicFilter left_ultrasonic;
UltrasonicFilter right_ultrasonic;

int main(int argc,char** argv)
{
    ros::init(argc,argv,"ultrasonic_node");
    ros::NodeHandle nh;

    left_ultrasonic_sub  = nh.subscribe("left_ultrasonic_raw", 10, &UltrasonicFilter::KF, &left_ultrasonic);
    right_ultrasonic_sub = nh.subscribe("right_ultrasonic_raw", 10, &UltrasonicFilter::KF, &right_ultrasonic);

    left_ultrasonic_pub   = nh.advertise<raw_data::Ultrasonic>("left_ultrasonic", 10);
    right_ultrasonic_pub  = nh.advertise<raw_data::Ultrasonic>("right_ultrasonic", 10);
    
    ros::spin();
    return 0;
}