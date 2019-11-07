#include <ros/ros.h>
#include "raw_data/rear_raw.h"

int main(int argc,char** argv)
{
    ros::init(argc,argv,"rear_raw_node");
    ros::Time::init();
    rearDataHandler handler;
    ros::spin();
    return 0;
}