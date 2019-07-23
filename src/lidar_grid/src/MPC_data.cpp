#include <ros/ros.h>
#include <can_msgs/Frame.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <vector>

typedef struct OBJECT {
    float x;
    float y;
} Objects;

///////////////////////MPC单片机CAN消息处理类////////////////////////
class MPCDataHandler
{
protected:
    ros::NodeHandle nh;
    ros::Subscriber can_sub;
    ros::Publisher radar_pub;
    ros::Publisher camera_pub;
    ros::Publisher radar_obj_pub;
    ros::Publisher camera_obj_pub;
    ros::Publisher fusion_pub;
    ros::Publisher nearest_obj_pub;
    std::string fixed_frame;
    float x_offset;

public:
    MPCDataHandler()
    {
        can_sub = nh.subscribe("received_messages", 1, &MPCDataHandler::canHandler, this);    //接收话题：received_messages
        radar_pub = nh.advertise<geometry_msgs::Point>("radar_pos", 1);                       //发布话题：radar_pos
        camera_pub = nh.advertise<geometry_msgs::Point>("camera_pos", 1);                     //发布话题：camera_pos
        radar_obj_pub = nh.advertise<visualization_msgs::Marker>("radar_obj", 1);             //发布话题：radar_obj
        camera_obj_pub = nh.advertise<visualization_msgs::Marker>("camera_obj", 1);           //发布话题：camera_obj
        fusion_pub = nh.advertise<geometry_msgs::Point>("fusion_pos", 1);                     //发布话题：fusion_pos
        nearest_obj_pub = nh.advertise<visualization_msgs::Marker>("nearest_obj", 1);         //发布话题：nearest_obj

        nh.getParam("/MPC_data/fixed_frame", fixed_frame);
        nh.getParam("/MPC_data/x_offset", x_offset);
    }

    void canHandler(const can_msgs::Frame& input);
};

//////////////////////////MPC单片机CAN消息处理函数///////////////////////////
void MPCDataHandler::canHandler(const can_msgs::Frame& input)
{
    int i;
//////////////////////////////解析CAN消息///////////////////////////////

    if(input.id == 0x563)
    {
        geometry_msgs::Point radar_pos;

        radar_pos.x = (float)(input.data[0]*256 + input.data[1])/100;

        if(radar_pos.x > 0)
        {
            radar_pos.x = radar_pos.x + x_offset;
        }

        radar_pos.y = (float)(input.data[2]*256 + input.data[3]);
        if(radar_pos.y < 0x8000)
        {
            radar_pos.y = radar_pos.y / 10;
        }
        else
        {
            radar_pos.y = (radar_pos.y - 0x10000)/10;
        }

        radar_pub.publish(radar_pos);     //发布毫米波雷达目标位置

        visualization_msgs::Marker radar_obj;

        radar_obj.header.frame_id = fixed_frame;
        radar_obj.header.stamp = ros::Time::now();
        radar_obj.id = 0;
        radar_obj.type = visualization_msgs::Marker::CUBE;
        radar_obj.action = visualization_msgs::Marker::ADD;
        radar_obj.pose.position.x = radar_pos.x;
        radar_obj.pose.position.y = radar_pos.y;
        radar_obj.pose.position.z = -0.9;
        radar_obj.pose.orientation.x = 0.0;
        radar_obj.pose.orientation.y = 0.0;
        radar_obj.pose.orientation.z = 0.0;
        radar_obj.pose.orientation.w = 1.0;
        radar_obj.scale.x = 0.6;
        radar_obj.scale.y = 0.6;
        radar_obj.scale.z = 1.7;
        radar_obj.color.r = 0;
        radar_obj.color.g = 0;
        radar_obj.color.b = 1;

        if(radar_pos.x == 0)
        {
            radar_obj.color.a = 0;
        }
        else
        {
            radar_obj.color.a = 0.8;
        }
        
        radar_obj.lifetime = ros::Duration();

        radar_obj_pub.publish(radar_obj);    //发布毫米波目标几何形状
    }
    else if(input.id == 0x564)
    {
        geometry_msgs::Point camera_pos;

        camera_pos.x = (float)(input.data[0]*256 + input.data[1])/100;

        if(camera_pos.x > 0)
        {
            camera_pos.x = camera_pos.x + x_offset;
        }

        camera_pos.y = (float)(input.data[2]*256 + input.data[3]);
        if(camera_pos.y < 0x8000)
        {
            camera_pos.y = camera_pos.y / 16;
        }
        else
        {
            camera_pos.y = (camera_pos.y - 0x10000)/16;
        }

        camera_pub.publish(camera_pos);   //发布摄像头目标位置

        visualization_msgs::Marker camera_obj;

        camera_obj.header.frame_id = fixed_frame;
        camera_obj.header.stamp = ros::Time::now();
        camera_obj.id = 0;
        camera_obj.type = visualization_msgs::Marker::CUBE;
        camera_obj.action = visualization_msgs::Marker::ADD;
        camera_obj.pose.position.x = camera_pos.x;
        camera_obj.pose.position.y = camera_pos.y;
        camera_obj.pose.position.z = -0.9;
        camera_obj.pose.orientation.x = 0.0;
        camera_obj.pose.orientation.y = 0.0;
        camera_obj.pose.orientation.z = 0.0;
        camera_obj.pose.orientation.w = 1.0;
        camera_obj.scale.x = 0.6;
        camera_obj.scale.y = 0.6;
        camera_obj.scale.z = 1.7;
        camera_obj.color.r = 0;
        camera_obj.color.g = 1;
        camera_obj.color.b = 0;

        if(camera_pos.x == 0)
        {
            camera_obj.color.a = 0;
        }
        else
        {
            camera_obj.color.a = 0.8;
        }
        
        camera_obj.lifetime = ros::Duration();

        camera_obj_pub.publish(camera_obj);    //发布摄像头目标几何形状
    }
    else if(input.id == 0x565)
    {
        geometry_msgs::Point fusion_pos;

        fusion_pos.x = (input.data[0]*256 + input.data[1])/100;

        if(fusion_pos.x > 0)
        {
            fusion_pos.x = fusion_pos.x + x_offset;
        }

        fusion_pos.y = input.data[2]*256 + input.data[3];
        if(fusion_pos.y < 0x8000)
        {
            fusion_pos.y = fusion_pos.y / 80;
        }
        else
        {
            fusion_pos.y = (fusion_pos.y - 0x10000) / 80;
        }

        fusion_pub.publish(fusion_pos);   //发布毫米波雷达和摄像头的融合位置

        visualization_msgs::Marker nearest;

        nearest.header.frame_id = fixed_frame;
        nearest.header.stamp = ros::Time::now();
        nearest.id = 0;
        nearest.type = visualization_msgs::Marker::CUBE;
        nearest.action = visualization_msgs::Marker::ADD;
        nearest.pose.position.x = fusion_pos.x;
        nearest.pose.position.y = fusion_pos.y;
        nearest.pose.position.z = -0.9;
        nearest.pose.orientation.x = 0.0;
        nearest.pose.orientation.y = 0.0;
        nearest.pose.orientation.z = 0.0;
        nearest.pose.orientation.w = 1.0;
        nearest.scale.x = 0.6;
        nearest.scale.y = 0.6;
        nearest.scale.z = 1.7;
        nearest.color.r = 1;
        nearest.color.g = 0;
        nearest.color.b = 0;

        if(fusion_pos.x == 0)
        {
            nearest.color.a = 0;
        }
        else
        {
            nearest.color.a = 0.8;
        }
        
        nearest.lifetime = ros::Duration();

        nearest_obj_pub.publish(nearest);    //发布最近目标几何形状
    }
}

////////////////////////////////////////////主函数///////////////////////////////////////////////////
int main(int argc,char** argv)
{
    ros::init(argc,argv,"MPC_data");

    MPCDataHandler handler;
        
    ros::spin();

    return 0;
}