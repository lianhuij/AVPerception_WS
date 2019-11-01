#ifndef RAW_DATA_MPC_RAW_H
#define RAW_DATA_MPC_RAW_H

#include <ros/ros.h>
#include <can_msgs/Frame.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <raw_data/CameraRaw.h>
#include <raw_data/CameraRawArray.h>
#include <raw_data/RadarRaw.h>
#include <raw_data/RadarRawArray.h>
#include <vector>

///////////////////////MPC单片机CAN消息处理类////////////////////////
class MPCDataHandler
{
protected:
    ros::NodeHandle nh;
    ros::Subscriber can_sub;
    ros::Publisher radar_raw_pub;
    ros::Publisher cam_raw_pub;
    ros::Publisher radar_rawArray_pub;
    ros::Publisher cam_rawArray_pub;
    std::string fixed_frame;
    float x_offset;
    std::vector<raw_data::RadarRaw> radarRaw;
    std::vector<raw_data::CameraRaw> camRaw;

public:
    MPCDataHandler();
    ~MPCDataHandler();
    void canHandler(const can_msgs::Frame& input);
    void pubRadarRaw(const std::vector<raw_data::RadarRaw>& input);
    void pubCamRaw(const std::vector<raw_data::CameraRaw>& input);
};

#endif //RAW_DATA_MPC_RAW_H