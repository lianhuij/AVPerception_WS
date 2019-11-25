#include <ros/ros.h>
#include "raw_data/MPC_raw.h"

int main(int argc,char** argv)
{
    ros::init(argc,argv,"MPC_raw_node");
    MPCDataHandler handler;
    ros::spin();
    return 0;
}