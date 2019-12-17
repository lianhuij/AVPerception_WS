#include "raw_data/rear_raw.h"
#include <ros/ros.h>
#include <can_msgs/Frame.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <raw_data/RadarRaw.h>
#include <raw_data/RadarRawArray.h>
#include <raw_data/Ultrasonic.h>
#include <vector>

rearDataHandler::rearDataHandler(void){
    can_sub = nh.subscribe("received_messages", 30, &rearDataHandler::canHandler, this);        //接收话题：received_messages
    right_radar_raw_pub = nh.advertise<visualization_msgs::MarkerArray>("right_radar_raw_rviz", 10);   //发布话题：right_radar_raw_rviz
    left_radar_raw_pub = nh.advertise<visualization_msgs::MarkerArray>("left_radar_raw_rviz", 10);   //发布话题：left_radar_raw_rviz
    right_radar_rawArray_pub = nh.advertise<raw_data::RadarRawArray>("right_radar_rawArray", 10);  //发布话题：right_radar_rawArray
    left_radar_rawArray_pub = nh.advertise<raw_data::RadarRawArray>("left_radar_rawArray", 10);  //发布话题：left_radar_rawArray
    ultrasonic_pub = nh.advertise<raw_data::Ultrasonic>("ultrasonic", 10);  //发布话题：ultrasonic
    nh.getParam("/rear_raw_node/fixed_frame", fixed_frame);
    nh.getParam("/rear_raw_node/GAP", GAP);
}

rearDataHandler::~rearDataHandler(){ }

//////////////////////////MPC单片机CAN消息处理函数///////////////////////////
void rearDataHandler::canHandler(const can_msgs::Frame& input)
{
    static int right_radar_num;
    static int left_radar_num;
    static bool right_radar_head = true;
    static bool left_radar_head = true;
//////////////////////////////解析CAN消息 SRR///////////////////////////////
    //右雷达，右模式，无静态目标
    if(input.id == 0x580){
        right_radar_num = input.data[0];
        right_radar_stamp = ros::Time::now();
        if(right_radar_num > 0){
            right_radar_head = false;
            std::vector<raw_data::RadarRaw>().swap(right_radar);  //清除元素并回收内存
        }else{
            std::vector<raw_data::RadarRaw> no_radar_obj;
            pubRightRadarRaw(no_radar_obj);
        }
        return;
    }
    if(!right_radar_head && input.id >= 0x581 && input.id <= 0x58F)
    {
        raw_data::RadarRaw radar_pos;
        radar_pos.distance = (float)(input.data[0]*256 + input.data[1])/50.0;
        radar_pos.angle = (float)(input.data[2]*256 + input.data[3]);
        if(radar_pos.angle < 0x8000)
        {
            radar_pos.angle = radar_pos.angle / 8.0;
        }
        else
        {
            radar_pos.angle = (radar_pos.angle - 0x10000)/8.0;
        }
        radar_pos.speed = (float)(input.data[4]*256 + input.data[5]);
        if(radar_pos.speed < 0x8000)
        {
            radar_pos.speed = radar_pos.speed / 8.0;
        }
        else
        {
            radar_pos.speed = (radar_pos.speed - 0x10000)/8.0;
        }
        radar_pos.x = radar_pos.distance*cos(radar_pos.angle*M_PI/180);
        radar_pos.y = radar_pos.distance*sin(radar_pos.angle*M_PI/180) - GAP;
        right_radar.push_back(radar_pos);
        right_radar_num--;
        if(right_radar_num == 0){   //接收完本周期数据
            pubRightRadarRaw(right_radar);
            right_radar_head = true;
        }
        return;
    }
    //左雷达，后模式，有静态目标
    if(input.id == 0x590){
        left_radar_num = input.data[0];
        left_radar_stamp = ros::Time::now();
        if(left_radar_num > 0){
            left_radar_head = false;
            std::vector<raw_data::RadarRaw>().swap(left_radar);  //清除元素并回收内存
        }else{
            std::vector<raw_data::RadarRaw> no_radar_obj;
            pubLeftRadarRaw(no_radar_obj);
        }
        return;
    }
    if(!left_radar_head && input.id >= 0x591 && input.id <= 0x59F)
    {
        raw_data::RadarRaw radar_pos;
        radar_pos.distance = (float)(input.data[0]*256 + input.data[1])/50.0;
        radar_pos.angle = (float)(input.data[2]*256 + input.data[3]);
        if(radar_pos.angle < 0x8000)
        {
            radar_pos.angle = radar_pos.angle / 8.0;
        }
        else
        {
            radar_pos.angle = (radar_pos.angle - 0x10000)/8.0;
        }
        radar_pos.speed = (float)(input.data[4]*256 + input.data[5]);
        if(radar_pos.speed < 0x8000)
        {
            radar_pos.speed = radar_pos.speed / 8.0;
        }
        else
        {
            radar_pos.speed = (radar_pos.speed - 0x10000)/8.0;
        }
        radar_pos.x = radar_pos.distance*cos(radar_pos.angle*M_PI/180);
        radar_pos.y = radar_pos.distance*sin(radar_pos.angle*M_PI/180) + GAP;
        left_radar.push_back(radar_pos);
        left_radar_num--;
        if(left_radar_num == 0){   //接收完本周期数据
            pubLeftRadarRaw(left_radar);
            left_radar_head = true;
        }
        return;
    }
//////////////////////////////解析CAN消息 ultrasonic///////////////////////////////
    if(input.id == 0x617){
        raw_data::Ultrasonic probe;
        probe.header.stamp = ros::Time::now();
        probe.probe_01 = (float)(input.data[0]*100 + input.data[1])/1000.0;
        probe.probe_02 = (float)(input.data[2]*100 + input.data[3])/1000.0;
        probe.probe_03 = (float)(input.data[4]*100 + input.data[5])/1000.0;
        probe.probe_04 = (float)(input.data[6]*100 + input.data[7])/1000.0;
        ultrasonic_pub.publish(probe);
    }
}

//////////////////////////发布radar MarkerArray///////////////////////////
void rearDataHandler::pubRightRadarRaw(const std::vector<raw_data::RadarRaw>& input){
    
    static int pre_marker_size_ = 0;
    raw_data::RadarRawArray raw_array;
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker bbox_marker;
    bbox_marker.header.frame_id = fixed_frame;
    bbox_marker.header.stamp = right_radar_stamp;
    bbox_marker.color.r = 1.0f;    //right radar color red
    bbox_marker.color.g = 0.0f;
    bbox_marker.color.b = 0.0f;
    bbox_marker.color.a = 0.5f;
    bbox_marker.lifetime = ros::Duration();
    bbox_marker.frame_locked = true;
    bbox_marker.type = visualization_msgs::Marker::CUBE;
    bbox_marker.action = visualization_msgs::Marker::ADD;
    raw_array.header.stamp = right_radar_stamp;

    int marker_id = 0;
    raw_array.num = input.size();
    for (int i = 0; i < raw_array.num; ++i)
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

        raw_array.data[i] = input[i];
    }

    if (raw_array.num > pre_marker_size_)
    {
        pre_marker_size_ = raw_array.num;
    }

    for (int i = marker_id; i < pre_marker_size_; ++i)
    {
        bbox_marker.id = i;
        bbox_marker.action = visualization_msgs::Marker::DELETE;
        marker_array.markers.push_back(bbox_marker);
    }
    pre_marker_size_ = marker_id;
    right_radar_raw_pub.publish(marker_array);
    right_radar_rawArray_pub.publish(raw_array);
}

void rearDataHandler::pubLeftRadarRaw(const std::vector<raw_data::RadarRaw>& input){
    
    static int pre_marker_size_ = 0;
    raw_data::RadarRawArray raw_array;
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker bbox_marker;
    bbox_marker.header.frame_id = fixed_frame;
    bbox_marker.header.stamp = left_radar_stamp;
    bbox_marker.color.r = 0.0f;
    bbox_marker.color.g = 0.0f;
    bbox_marker.color.b = 1.0f;    //left radar color blue
    bbox_marker.color.a = 0.5f;
    bbox_marker.lifetime = ros::Duration();
    bbox_marker.frame_locked = true;
    bbox_marker.type = visualization_msgs::Marker::CUBE;
    bbox_marker.action = visualization_msgs::Marker::ADD;
    raw_array.header.stamp = left_radar_stamp;

    int marker_id = 0;
    raw_array.num = input.size();
    for (int i = 0; i < raw_array.num; ++i)
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

        raw_array.data[i] = input[i];
    }

    if (raw_array.num > pre_marker_size_)
    {
        pre_marker_size_ = raw_array.num;
    }

    for (int i = marker_id; i < pre_marker_size_; ++i)
    {
        bbox_marker.id = i;
        bbox_marker.action = visualization_msgs::Marker::DELETE;
        marker_array.markers.push_back(bbox_marker);
    }
    pre_marker_size_ = marker_id;
    left_radar_raw_pub.publish(marker_array);
    left_radar_rawArray_pub.publish(raw_array);
}