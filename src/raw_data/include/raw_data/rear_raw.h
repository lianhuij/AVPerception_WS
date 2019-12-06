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
    ros::Time radar_stamp;
    std::string fixed_frame;
    std::vector<raw_data::RearRadarRaw> radarRaw;

public:
    rearDataHandler(void);
    ~rearDataHandler();
    void canHandler(const can_msgs::Frame& input);
    void pubRadarRaw(const std::vector<raw_data::RearRadarRaw>& input);
};

#endif //RAW_DATA_REAR_RAW_H