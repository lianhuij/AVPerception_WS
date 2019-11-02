#include "raw_data/rear_raw.h"
#include <ros/ros.h>
#include <can_msgs/Frame.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <raw_data/RearRadarRaw.h>
#include <raw_data/RearRadarRawArray.h>
#include <vector>

rearDataHandler::rearDataHandler(){
    can_sub = nh.subscribe("received_messages", 1, &rearDataHandler::canHandler, this);        //接收话题：received_messages
    radar_raw_pub = nh.advertise<visualization_msgs::MarkerArray>("rear_radar_raw_rviz", 1);   //发布话题：rear_radar_raw_rviz
    radar_rawArray_pub = nh.advertise<raw_data::RearRadarRawArray>("rear_radar_rawArray", 1);  //发布话题：rear_radar_rawArray
    nh.getParam("/rear_raw_node/fixed_frame", fixed_frame);
    nh.getParam("/rear_raw_node/rear_x_offset", rear_x_offset);
}

rearDataHandler::~rearDataHandler(){ }

//////////////////////////MPC单片机CAN消息处理函数///////////////////////////
void rearDataHandler::canHandler(const can_msgs::Frame& input)
{
    static int radar_num;
    static int radar_IsFirst = 1;

//////////////////////////////解析CAN消息 SRR///////////////////////////////
    if(input.id == 0x561){
        radar_IsFirst = 0;
        radar_num = 6;
        std::vector<raw_data::RearRadarRaw>().swap(radarRaw);  //清除元素并回收内存
    }
    if(radar_IsFirst == 0 && input.id >= 0x561 && input.id <= 0x566)
    {
        raw_data::RearRadarRaw radar_pos;
        radar_pos.x = (float)(input.data[0]*256 + input.data[1])/50;
        radar_pos.x = rear_x_offset - radar_pos.x;
        radar_pos.y = (float)(input.data[2]*256 + input.data[3]);
        if(radar_pos.y < 0x8000)
        {
            radar_pos.y = radar_pos.y / 50;
        }
        else
        {
            radar_pos.y = (radar_pos.y - 0x10000)/50;
        }
        radar_pos.vx = (float)(input.data[4]*256 + input.data[5]);
        if(radar_pos.vx < 0x8000)
        {
            radar_pos.vx = radar_pos.vx / 8;
        }
        else
        {
            radar_pos.vx = (radar_pos.vx - 0x10000)/8;
        }
        radar_pos.vx = -radar_pos.vx;
        radar_pos.vy = (float)(input.data[6]*256 + input.data[7]);
        if(radar_pos.vy < 0x8000)
        {
            radar_pos.vy = radar_pos.vy / 8;
        }
        else
        {
            radar_pos.vy = (radar_pos.vy - 0x10000)/8;
        }
        radarRaw.push_back(radar_pos);
        radar_num--;
        if(radar_num == 0){   //接收完本周期数据
            pubRadarRaw(radarRaw);
            radar_IsFirst = 1;
        }
    }
}

//////////////////////////发布radar MarkerArray///////////////////////////
void rearDataHandler::pubRadarRaw(const std::vector<raw_data::RearRadarRaw>& input){
    
    raw_data::RearRadarRawArray raw_array;
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker bbox_marker;
    bbox_marker.header.frame_id = fixed_frame;
    bbox_marker.header.stamp = ros::Time::now();
    bbox_marker.color.a = 0.5;
    bbox_marker.lifetime = ros::Duration();
    //bbox_marker.frame_locked = true;
    bbox_marker.type = visualization_msgs::Marker::CUBE;
    bbox_marker.action = visualization_msgs::Marker::ADD;

    for (size_t i = 0; i < 6; ++i)
    {
        bbox_marker.id = i;
        bbox_marker.pose.position.x = input[i].x;
        bbox_marker.pose.position.y = input[i].y;
        bbox_marker.pose.position.z = -0.9;
        bbox_marker.scale.x = 0.6;
        bbox_marker.scale.y = 0.6;
        bbox_marker.scale.z = 1.7;
        if(input[i].x == rear_x_offset){
            bbox_marker.color.a = 0;
        }
        if(i < 3){
            bbox_marker.color.r = 0.0f;    //left rear radar color blue
            bbox_marker.color.g = 0.0f;
            bbox_marker.color.b = 1.0f;
        }else{
            bbox_marker.color.r = 1.0f;    //right rear radar color red
            bbox_marker.color.g = 0.0f;
            bbox_marker.color.b = 0.0f;
        }
        marker_array.markers.push_back(bbox_marker);
        raw_array.data.push_back(input[i]);
    }

    radar_raw_pub.publish(marker_array);
    radar_rawArray_pub.publish(raw_array);
}