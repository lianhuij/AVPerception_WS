#include "raw_data/MPC_raw.h"
#include <ros/ros.h>
#include <can_msgs/Frame.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <raw_data/CameraRaw.h>
#include <raw_data/CameraRawArray.h>
#include <raw_data/RadarRaw.h>
#include <raw_data/RadarRawArray.h>
#include <vector>
#include <cmath>

MPCDataHandler::MPCDataHandler(){
    can_sub = nh.subscribe("received_messages", 20, &MPCDataHandler::canHandler, this);    //接收话题：received_messages
    radar_raw_pub = nh.advertise<visualization_msgs::MarkerArray>("radar_raw_rviz", 10);   //发布话题：radar_raw_rviz
    cam_raw_pub = nh.advertise<visualization_msgs::MarkerArray>("camera_raw_rviz", 10);    //发布话题：camera_raw_rviz
    radar_rawArray_pub = nh.advertise<raw_data::RadarRawArray>("radar_rawArray", 10);      //发布话题：radar_rawArray
    cam_rawArray_pub = nh.advertise<raw_data::CameraRawArray>("camera_rawArray", 10);      //发布话题：camera_rawArray
    nh.getParam("/MPC_raw_node/fixed_frame", fixed_frame);
    nh.getParam("/MPC_raw_node/x_offset", x_offset);
}

MPCDataHandler::~MPCDataHandler(){ }

//////////////////////////MPC单片机CAN消息处理函数///////////////////////////
void MPCDataHandler::canHandler(const can_msgs::Frame& input)
{
    static int radar_num;
    static int cam_num;
    static bool radar_head = true;
    static bool cam_head = true;
    static ros::Time radar_stamp;
    static ros::Time cam_stamp;

//////////////////////////////解析CAN消息 ESR///////////////////////////////
    if(input.id == 0x550){
        radar_num = input.data[0];
        radar_stamp = input.header.stamp;
        if(radar_num > 0){
            radar_head = false;
            std::vector<raw_data::RadarRaw>().swap(radarRaw);  //清除元素并回收内存
        }else{
            std::vector<raw_data::RadarRaw> no_radar_obj;
            pubRadarRaw(no_radar_obj, radar_stamp);
        }
        return;
    }
    if(!radar_head && input.id >= 0x551 && input.id <= 0x55F)
    {
        raw_data::RadarRaw radar_pos;
        radar_pos.distance = (float)(input.data[0]*256 + input.data[1])/10.0;
        radar_pos.angle = (float)(input.data[2]*256 + input.data[3]);
        if(radar_pos.angle < 0x8000)
        {
            radar_pos.angle = radar_pos.angle / 10.0;
        }
        else
        {
            radar_pos.angle = (radar_pos.angle - 0x10000)/10.0;
        }
        radar_pos.speed = (float)(input.data[4]*256 + input.data[5]);
        if(radar_pos.speed < 0x8000)
        {
            radar_pos.speed = radar_pos.speed / 100.0;
        }
        else
        {
            radar_pos.speed = (radar_pos.speed - 0x10000)/100.0;
        }
        radar_pos.x = radar_pos.distance*cos(radar_pos.angle*M_PI/180) + x_offset;
        radar_pos.y = radar_pos.distance*sin(radar_pos.angle*M_PI/180);
        radarRaw.push_back(radar_pos);
        radar_num--;
        if(radar_num == 0){   //接收完本周期数据
            pubRadarRaw(radarRaw, radar_stamp);
            radar_head = true;
        }
        return;
    }

//////////////////////////////解析CAN消息 Mobileye///////////////////////////////
    if(input.id == 0x570){
        cam_num = input.data[0];
        cam_stamp = input.header.stamp;
        if(cam_num > 0){
            cam_head = false;
            std::vector<raw_data::CameraRaw>().swap(camRaw);  //清除元素并回收内存
        }else{
            std::vector<raw_data::CameraRaw> no_cam_obj;
            pubCamRaw(no_cam_obj, cam_stamp);
        }
        return;
    }
    if(!cam_head && input.id >= 0x571 && input.id <= 0x57F)
    {
        raw_data::CameraRaw cam_pos;
        cam_pos.x = (float)(input.data[0]*256 + input.data[1])/16.0 + x_offset + 3.5;  //camera纵向距离补偿
        cam_pos.y = (float)(input.data[2]*256 + input.data[3]);
        if(cam_pos.y < 0x8000)
        {
            cam_pos.y = cam_pos.y / 16.0;
        }
        else
        {
            cam_pos.y = (cam_pos.y - 0x10000)/16.0;
        }
        cam_pos.vx = (float)(input.data[4]*256 + input.data[5]);
        if(cam_pos.vx < 0x8000)
        {
            cam_pos.vx = cam_pos.vx / 16.0;
        }
        else
        {
            cam_pos.vx = (cam_pos.vx - 0x10000)/16.0;
        }
        cam_pos.ID = input.data[6];
        cam_pos.target_type = input.data[7];      // 0:vehicle, 1:truck, 2:bike, 3:ped, 4:bicycle
        camRaw.push_back(cam_pos);
        cam_num--;
        if(cam_num == 0){   //接收完本周期数据
            pubCamRaw(camRaw, cam_stamp);
            cam_head = true;
        }
        return;
    }
}

//////////////////////////发布radar MarkerArray///////////////////////////
void MPCDataHandler::pubRadarRaw(const std::vector<raw_data::RadarRaw>& input, const ros::Time& radar_stamp){
    
    static int pre_marker_size_ = 0;
    raw_data::RadarRawArray raw_array;
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker bbox_marker;
    bbox_marker.header.frame_id = fixed_frame;
    bbox_marker.header.stamp = radar_stamp;
    bbox_marker.color.r = 1.0f;    //radar color red
    bbox_marker.color.g = 0.0f;
    bbox_marker.color.b = 0.0f;
    bbox_marker.color.a = 0.5;
    bbox_marker.lifetime = ros::Duration();
    bbox_marker.frame_locked = true;
    bbox_marker.type = visualization_msgs::Marker::CUBE;
    bbox_marker.action = visualization_msgs::Marker::ADD;
    raw_array.header.stamp = radar_stamp;

    int marker_id = 0;
    if(input.size() <= 15){
        raw_array.num = input.size();
    }else{
        ROS_ERROR("radar raw num > 15");
        return ;
    }
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
    radar_raw_pub.publish(marker_array);
    radar_rawArray_pub.publish(raw_array);
}

//////////////////////////发布camera MarkerArray///////////////////////////
void MPCDataHandler::pubCamRaw(const std::vector<raw_data::CameraRaw>& input, const ros::Time& cam_stamp){
    
    static int pre_marker_size_ = 0;
    raw_data::CameraRawArray raw_array;
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker bbox_marker;
    bbox_marker.header.frame_id = fixed_frame;
    bbox_marker.header.stamp = cam_stamp;
    bbox_marker.color.r = 0.0f;
    bbox_marker.color.g = 0.0f;
    bbox_marker.color.b = 1.0f;    //camera color blue
    bbox_marker.color.a = 0.5;
    bbox_marker.lifetime = ros::Duration();
    bbox_marker.frame_locked = true;
    bbox_marker.type = visualization_msgs::Marker::CUBE;
    bbox_marker.action = visualization_msgs::Marker::ADD;
    raw_array.header.stamp = cam_stamp;

    int marker_id = 0;
    if(input.size() <= 10){
        raw_array.num = input.size();
    }else{
        ROS_ERROR("camera raw num > 10");
        return ;
    }
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
    cam_raw_pub.publish(marker_array);
    cam_rawArray_pub.publish(raw_array);
}