#ifndef RAW_DATA_REAR_RAW_H
#define RAW_DATA_REAR_RAW_H

#include <ros/ros.h>
#include <can_msgs/Frame.h>
#include <raw_data/RadarRaw.h>
#include <raw_data/RadarRawArray.h>
#include <raw_data/Ultrasonic.h>
#include <vector>

///////////////////////MPC单片机CAN消息处理类////////////////////////
class rearDataHandler
{
protected:
    ros::NodeHandle nh;
    ros::Subscriber can_sub;
    ros::Publisher right_radar_raw_pub;
    ros::Publisher right_radar_rawArray_pub;
    ros::Publisher left_radar_raw_pub;
    ros::Publisher left_radar_rawArray_pub;
    ros::Publisher ultrasonic_ctrl_pub;
    ros::Publisher left_ultrasonic_pub;
    ros::Publisher right_ultrasonic_pub;
    ros::Time right_radar_stamp;
    ros::Time left_radar_stamp;
    std::string fixed_frame;
    float Y_OFFSET;
    int ultrasonic_mode;
    std::vector<raw_data::RadarRaw> right_radar;
    std::vector<raw_data::RadarRaw> left_radar;

public:
    rearDataHandler(void);
    ~rearDataHandler();
    void canHandler(const can_msgs::Frame& input);
    void pubRightRadarRaw(const std::vector<raw_data::RadarRaw>& input);
    void pubLeftRadarRaw(const std::vector<raw_data::RadarRaw>& input);
};

#endif //RAW_DATA_REAR_RAW_H