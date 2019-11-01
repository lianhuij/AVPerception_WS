#ifndef RAW_DATA_REAR_RAW_H
#define RAW_DATA_REAR_RAW_H

#include <ros/ros.h>
#include <can_msgs/Frame.h>
#include <raw_data/RearRadarRaw.h>
#include <raw_data/RearRadarRawArray.h>
#include <vector>

///////////////////////MPC单片机CAN消息处理类////////////////////////
class rearDataHandler
{
protected:
    ros::NodeHandle nh;
    ros::Subscriber can_sub;
    ros::Publisher radar_raw_pub;
    ros::Publisher radar_rawArray_pub;
    std::string fixed_frame;
    float rear_x_offset;
    std::vector<raw_data::RearRadarRaw> radarRaw;

public:
    rearDataHandler();
    ~rearDataHandler();
    void canHandler(const can_msgs::Frame& input);
    void pubRadarRaw(const std::vector<raw_data::RearRadarRaw>& input);
};

#endif //RAW_DATA_REAR_RAW_H