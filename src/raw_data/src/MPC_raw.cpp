#include <ros/ros.h>
#include <can_msgs/Frame.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <vector>
#include <cmath>

typedef struct Radar_Data{
    float distance;
    float angle;
    float speed;
    float x;
    float y;
}radar_data;

typedef struct Cam_Data{
    float x;
    float y;
    float vx;
    int ID;
    int Type;
}cam_data;

///////////////////////MPC单片机CAN消息处理类////////////////////////
class MPCDataHandler
{
protected:
    ros::NodeHandle nh;
    ros::Subscriber can_sub;
    ros::Publisher radar_raw_pub;
    ros::Publisher cam_raw_pub;
    std::string fixed_frame;
    float x_offset;
    std::vector<radar_data> radarRaw;
    std::vector<cam_data> camRaw;

public:
    MPCDataHandler()
    {
        can_sub = nh.subscribe("received_messages", 1, &MPCDataHandler::canHandler, this);    //接收话题：received_messages
        radar_raw_pub = nh.advertise<visualization_msgs::MarkerArray>("radar_raw", 1);        //发布话题：radar_raw
        cam_raw_pub = nh.advertise<visualization_msgs::MarkerArray>("cam_raw", 1);         //发布话题：cam_raw
        nh.getParam("/MPC_raw/fixed_frame", fixed_frame);
        nh.getParam("/MPC_raw/x_offset", x_offset);
    }
    ~MPCDataHandler(){}
    void canHandler(const can_msgs::Frame& input);
    void pubRadarRaw(const std::vector<radar_data>& input);
    void pubCamRaw(const std::vector<cam_data>& input);
};

//////////////////////////MPC单片机CAN消息处理函数///////////////////////////
void MPCDataHandler::canHandler(const can_msgs::Frame& input)
{
    static int radar_num;
    static int cam_num;
    static int radar_IsFirst = 1;
    static int cam_IsFirst = 1;

//////////////////////////////解析CAN消息 ESR///////////////////////////////
    if(input.id == 0x550){
        radar_num = input.data[0];
        if(radar_num > 0){
            radar_IsFirst = 0;
            std::vector<radar_data>().swap(radarRaw);  //清除元素并回收内存
        }
    }
    if(radar_IsFirst == 0 && input.id >= 0x551 && input.id <= 0x55F)
    {
        radar_data radar_pos;
        radar_pos.distance = (float)(input.data[0]*256 + input.data[1])/10;
        radar_pos.angle = (float)(input.data[2]*256 + input.data[3]);
        if(radar_pos.angle < 0x8000)
        {
            radar_pos.angle = radar_pos.angle / 10;
        }
        else
        {
            radar_pos.angle = (radar_pos.angle - 0x10000)/10;
        }
        radar_pos.speed = (float)(input.data[4]*256 + input.data[5]);
        if(radar_pos.speed < 0x8000)
        {
            radar_pos.speed = radar_pos.speed / 100;
        }
        else
        {
            radar_pos.speed = (radar_pos.speed - 0x10000)/100;
        }
        radar_pos.x = radar_pos.distance*cos(radar_pos.angle*M_PI/180) + x_offset;
        radar_pos.y = radar_pos.distance*sin(radar_pos.angle*M_PI/180);
        radarRaw.push_back(radar_pos);
        radar_num--;
        if(radar_num == 0){   //接收完本周期数据
            pubRadarRaw(radarRaw);
            radar_IsFirst = 1;
        }
    }

//////////////////////////////解析CAN消息 Mobileye///////////////////////////////
    if(input.id == 0x570){
        cam_num = input.data[0];
        if(cam_num > 0){
            cam_IsFirst = 0;
            std::vector<cam_data>().swap(camRaw);  //清除元素并回收内存
        }
    }
    if(cam_IsFirst == 0 && input.id >= 0x571 && input.id <= 0x57F)
    {
        cam_data cam_pos;
        cam_pos.x = (float)(input.data[0]*256 + input.data[1])/16 + x_offset;
        cam_pos.y = (float)(input.data[2]*256 + input.data[3]);
        if(cam_pos.y < 0x8000)
        {
            cam_pos.y = cam_pos.y / 16;
        }
        else
        {
            cam_pos.y = (cam_pos.y - 0x10000)/16;
        }
        cam_pos.vx = (float)(input.data[4]*256 + input.data[5]);
        if(cam_pos.vx < 0x8000)
        {
            cam_pos.vx = cam_pos.vx / 16;
        }
        else
        {
            cam_pos.vx = (cam_pos.vx - 0x10000)/16;
        }
        cam_pos.ID = input.data[6];
        cam_pos.Type = input.data[7];
        camRaw.push_back(cam_pos);
        cam_num--;
        if(cam_num == 0){   //接收完本周期数据
            pubCamRaw(camRaw);
            cam_IsFirst = 1;
        }
    }
}

//////////////////////////发布radar MarkerArray///////////////////////////
void MPCDataHandler::pubRadarRaw(const std::vector<radar_data>& input){
    
    static int max_marker_size_ = 0;
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker bbox_marker;
    bbox_marker.header.frame_id = fixed_frame;
    bbox_marker.header.stamp = ros::Time::now();
    bbox_marker.color.r = 1.0f;    //radar colar red
    bbox_marker.color.g = 0.0f;
    bbox_marker.color.b = 0.0f;
    bbox_marker.color.a = 0.5;
    bbox_marker.lifetime = ros::Duration();
    //bbox_marker.frame_locked = true;
    bbox_marker.type = visualization_msgs::Marker::CUBE;
    bbox_marker.action = visualization_msgs::Marker::ADD;

    int marker_id = 0;
    for (size_t i = 0; i < input.size(); ++i)
    {
        bbox_marker.id = marker_id;
        bbox_marker.pose.position.x = input[i].x;
        bbox_marker.pose.position.y = input[i].y;
        bbox_marker.pose.position.z = -0.9;
        bbox_marker.scale.x = 0.6;
        bbox_marker.scale.y = 0.6;
        bbox_marker.scale.z = 1.7;
        marker_array.markers.push_back(bbox_marker);
        ++marker_id;
    }

    if (marker_array.markers.size() > max_marker_size_)
    {
        max_marker_size_ = marker_array.markers.size();
    }

    for (size_t i = marker_id; i < max_marker_size_; ++i)
    {
        bbox_marker.id = i;
        bbox_marker.color.a = 0;
        bbox_marker.pose.position.x = 0;
        bbox_marker.pose.position.y = 0;
        bbox_marker.pose.position.z = 0;
        bbox_marker.scale.x = 0;
        bbox_marker.scale.y = 0;
        bbox_marker.scale.z = 0;
        marker_array.markers.push_back(bbox_marker);
    }
    radar_raw_pub.publish(marker_array);
}

//////////////////////////发布camera MarkerArray///////////////////////////
void MPCDataHandler::pubCamRaw(const std::vector<cam_data>& input){
    
    static int max_marker_size_ = 0;
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker bbox_marker;
    bbox_marker.header.frame_id = fixed_frame;
    bbox_marker.header.stamp = ros::Time::now();
    bbox_marker.color.r = 0.0f;
    bbox_marker.color.g = 0.0f;
    bbox_marker.color.b = 1.0f;    //camera colar blue
    bbox_marker.color.a = 0.5;
    bbox_marker.lifetime = ros::Duration();
    //bbox_marker.frame_locked = true;
    bbox_marker.type = visualization_msgs::Marker::CUBE;
    bbox_marker.action = visualization_msgs::Marker::ADD;

    int marker_id = 0;
    for (size_t i = 0; i < input.size(); ++i)
    {
        bbox_marker.id = marker_id;
        bbox_marker.pose.position.x = input[i].x;
        bbox_marker.pose.position.y = input[i].y;
        bbox_marker.pose.position.z = -0.9;
        bbox_marker.scale.x = 0.6;
        bbox_marker.scale.y = 0.6;
        bbox_marker.scale.z = 1.7;
        marker_array.markers.push_back(bbox_marker);
        ++marker_id;
    }

    if (marker_array.markers.size() > max_marker_size_)
    {
        max_marker_size_ = marker_array.markers.size();
    }

    for (size_t i = marker_id; i < max_marker_size_; ++i)
    {
        bbox_marker.id = i;
        bbox_marker.color.a = 0;
        bbox_marker.pose.position.x = 0;
        bbox_marker.pose.position.y = 0;
        bbox_marker.pose.position.z = 0;
        bbox_marker.scale.x = 0;
        bbox_marker.scale.y = 0;
        bbox_marker.scale.z = 0;
        marker_array.markers.push_back(bbox_marker);
    }
    cam_raw_pub.publish(marker_array);
}

////////////////////////////////////////////主函数///////////////////////////////////////////////////
int main(int argc,char** argv)
{
    ros::init(argc,argv,"MPC_raw");

    MPCDataHandler handler;
        
    ros::spin();

    return 0;
}