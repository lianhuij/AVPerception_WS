#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <detection/Target.h>
#include <detection/TargetArray.h>
#include "detection/rear_radar_tracker.h"
#include "detection/rear_radar_fusion.h"

extern ros::Publisher radar_fusion_rviz_pub, radar_fusion_pub;
extern std::string FIXED_FRAME;
extern float Y_OFFSET;
extern float HALF_LANE_WIDTH;
extern float RX_GATE;
extern float RY_GATE;
extern int MAX_LOST_CNT;
extern RearRadarTracker left_radar_tracker;
extern RearRadarTracker right_radar_tracker;

RearRadarFusion::RearRadarFusion(void)
{
    left_radar_tracks.clear();
    right_radar_tracks.clear();
    for(int i=0; i<3; ++i){
      min[i] = vector6d::Zero(6);
      lost_cnt[i] = 0;
    }
    first = true;
    Fs = matrix6d::Zero(6,6);
    Fs(0,0) = Fs(1,1) = Fs(2,2) = Fs(3,3) = Fs(4,4) = Fs(5,5) = 1;
}

RearRadarFusion::~RearRadarFusion() { }

void RearRadarFusion::Run(void)
{
    // clock_t start = clock();
    if(first){
      first = false;
      fusion_stamp = ros::Time::now();
    }else{
      ds = (ros::Time::now() - fusion_stamp).toSec();
      fusion_stamp = ros::Time::now();
      Fs(0,2) = Fs(1,3) = Fs(2,4) = Fs(3,5) = ds;
      Fs(0,4) = Fs(1,5) = ds*ds/2;
    }
    GetLocalTracks();

    //将目标划分到三个车道内
    std::vector<std::pair<std::string, int> > left_lane, mid_lane, right_lane;
    std::pair<std::string, int> tmp;
    for(int i=0; i<left_radar_tracks.size(); ++i){
      if(left_radar_tracks[i].X(1) > HALF_LANE_WIDTH){
        tmp.first = "left";
        tmp.second = i;
        left_lane.push_back(tmp);
      }else if(left_radar_tracks[i].X(1) < -HALF_LANE_WIDTH){
        tmp.first = "left";
        tmp.second = i;
        right_lane.push_back(tmp);
      }else{
        tmp.first = "left";
        tmp.second = i;
        mid_lane.push_back(tmp);
      }
    }
    for(int i=0; i<right_radar_tracks.size(); ++i){
      if(right_radar_tracks[i].X(1) > HALF_LANE_WIDTH){
        tmp.first = "right";
        tmp.second = i;
        left_lane.push_back(tmp);
      }else if(right_radar_tracks[i].X(1) < -HALF_LANE_WIDTH){
        tmp.first = "right";
        tmp.second = i;
        right_lane.push_back(tmp);
      }else{
        tmp.first = "right";
        tmp.second = i;
        mid_lane.push_back(tmp);
      }
    }
    // std::cout<<left_lane.size()<<" "<<mid_lane.size()<<" "<<right_lane.size()<<std::endl;

    //选出每个车道的最近目标
    int left_min[2], mid_min[2], right_min[2];  //index 0: left，1: right
    left_min[0] = left_min[1] = -1;  //-1: no target
    mid_min[0] = mid_min[1] = -1;
    right_min[0] = right_min[1] = -1;

    for(int i=0; i<left_lane.size(); ++i){
      if(left_lane[i].first == "left"){
        if(left_min[0] == -1){
          left_min[0] = left_lane[i].second;
        }else{
          if(left_radar_tracks[left_lane[i].second].X(0) < left_radar_tracks[left_min[0]].X(0)){
            left_min[0] = left_lane[i].second;
          }
        }
      }
      if(left_lane[i].first == "right"){
        if(left_min[1] == -1){
          left_min[1] = left_lane[i].second;
        }else{
          if(right_radar_tracks[left_lane[i].second].X(0) < right_radar_tracks[left_min[1]].X(0)){
            left_min[1] = left_lane[i].second;
          }
        }
      }
    }

    for(int i=0; i<mid_lane.size(); ++i){
      if(mid_lane[i].first == "left"){
        if(mid_min[0] == -1){
          mid_min[0] = mid_lane[i].second;
        }else{
          if(left_radar_tracks[mid_lane[i].second].X(0) < left_radar_tracks[mid_min[0]].X(0)){
            mid_min[0] = mid_lane[i].second;
          }
        }
      }
      if(mid_lane[i].first == "right"){
        if(mid_min[1] == -1){
          mid_min[1] = mid_lane[i].second;
        }else{
          if(right_radar_tracks[mid_lane[i].second].X(0) < right_radar_tracks[mid_min[1]].X(0)){
            mid_min[1] = mid_lane[i].second;
          }
        }
      }
    }

    for(int i=0; i<right_lane.size(); ++i){
      if(right_lane[i].first == "left"){
        if(right_min[0] == -1){
          right_min[0] = right_lane[i].second;
        }else{
          if(left_radar_tracks[right_lane[i].second].X(0) < left_radar_tracks[right_min[0]].X(0)){
            right_min[0] = right_lane[i].second;
          }
        }
      }
      if(right_lane[i].first == "right"){
        if(right_min[1] == -1){
          right_min[1] = right_lane[i].second;
        }else{
          if(right_radar_tracks[right_lane[i].second].X(0) < right_radar_tracks[right_min[1]].X(0)){
            right_min[1] = right_lane[i].second;
          }
        }
      }
    }
    // std::cout<<left_min[0]<<" "<<left_min[1]<<" "<<mid_min[0]<<" "<<mid_min[1]<<" "<<right_min[0]<<" "<<right_min[1]<<std::endl;

    //对每个车道的最近目标融合
    float left_rx, left_ry;
    float right_rx, right_ry;
    if(left_min[0] != -1 && left_min[1] != -1){
      left_rx  = left_radar_tracks[left_min[0]].X(0);
      left_ry  = left_radar_tracks[left_min[0]].X(1);
      right_rx = right_radar_tracks[left_min[1]].X(0);
      right_ry = right_radar_tracks[left_min[1]].X(1);
      if(fabs(left_rx- right_rx) < RX_GATE && fabs(left_rx- right_rx) < RY_GATE){
        matrix6d P = (left_radar_tracks[left_min[0]].P.inverse() + right_radar_tracks[left_min[1]].P.inverse()).inverse();
        min[0] = P * (left_radar_tracks[left_min[0]].P.inverse() * left_radar_tracks[left_min[0]].X
                    + right_radar_tracks[left_min[1]].P.inverse() * right_radar_tracks[left_min[1]].X);
      }else{
        if(left_rx < right_rx){
          min[0] = left_radar_tracks[left_min[0]].X;
        }else{
          min[0] = right_radar_tracks[left_min[1]].X;
        }
      }
      lost_cnt[0] = 0;
    }else if(left_min[0] != -1){
      min[0] = left_radar_tracks[left_min[0]].X;
      lost_cnt[0] = 0;
    }else if(left_min[1] != -1){
      min[0] = right_radar_tracks[left_min[1]].X;
      lost_cnt[0] = 0;
    }else{
      if(min[0](0) > 0){
        lost_cnt[0] = lost_cnt[0] >= MAX_LOST_CNT? MAX_LOST_CNT: (lost_cnt[0]+1);
        if(lost_cnt[0] < MAX_LOST_CNT){
          min[0] = Fs * min[0];
        }else{
          min[0] = vector6d::Zero(6);
          lost_cnt[0] = 0;
        }
      }else{
        min[0] = vector6d::Zero(6);
      }
    }

    if(mid_min[0] != -1 && mid_min[1] != -1){
      left_rx  = left_radar_tracks[mid_min[0]].X(0);
      left_ry  = left_radar_tracks[mid_min[0]].X(1);
      right_rx = right_radar_tracks[mid_min[1]].X(0);
      right_ry = right_radar_tracks[mid_min[1]].X(1);
      if(fabs(left_rx- right_rx) < RX_GATE && fabs(left_rx- right_rx) < RY_GATE){
        matrix6d P = (left_radar_tracks[mid_min[0]].P.inverse() + right_radar_tracks[mid_min[1]].P.inverse()).inverse();
        min[1] = P * (left_radar_tracks[mid_min[0]].P.inverse() * left_radar_tracks[mid_min[0]].X
                    + right_radar_tracks[mid_min[1]].P.inverse() * right_radar_tracks[mid_min[1]].X);
      }else{
        if(left_rx < right_rx){
          min[1] = left_radar_tracks[mid_min[0]].X;
        }else{
          min[1] = right_radar_tracks[mid_min[1]].X;
        }
      }
      lost_cnt[1] = 0;
    }else if(mid_min[0] != -1){
      min[1] = left_radar_tracks[mid_min[0]].X;
      lost_cnt[1] = 0;
    }else if(mid_min[1] != -1){
      min[1] = right_radar_tracks[mid_min[1]].X;
      lost_cnt[1] = 0;
    }else{
      if(min[1](0) > 0){
        lost_cnt[1] = lost_cnt[1] >= MAX_LOST_CNT? MAX_LOST_CNT: (lost_cnt[1]+1);
        if(lost_cnt[1] < MAX_LOST_CNT){
          min[1] = Fs * min[1];
        }else{
          min[1] = vector6d::Zero(6);
          lost_cnt[1] = 0;
        }
      }else{
        min[1] = vector6d::Zero(6);
      }
    }

    if(right_min[0] != -1 && right_min[1] != -1){
      left_rx  = left_radar_tracks[right_min[0]].X(0);
      left_ry  = left_radar_tracks[right_min[0]].X(1);
      right_rx = right_radar_tracks[right_min[1]].X(0);
      right_ry = right_radar_tracks[right_min[1]].X(1);
      if(fabs(left_rx- right_rx) < RX_GATE && fabs(left_rx- right_rx) < RY_GATE){
        matrix6d P = (left_radar_tracks[right_min[0]].P.inverse() + right_radar_tracks[right_min[1]].P.inverse()).inverse();
        min[2] = P * (left_radar_tracks[right_min[0]].P.inverse() * left_radar_tracks[right_min[0]].X
                    + right_radar_tracks[right_min[1]].P.inverse() * right_radar_tracks[right_min[1]].X);
      }else{
        if(left_rx < right_rx){
          min[2] = left_radar_tracks[right_min[0]].X;
        }else{
          min[2] = right_radar_tracks[right_min[1]].X;
        }
      }
      lost_cnt[2] = 0;
    }else if(right_min[0] != -1){
      min[2] = left_radar_tracks[right_min[0]].X;
      lost_cnt[2] = 0;
    }else if(right_min[1] != -1){
      min[2] = right_radar_tracks[right_min[1]].X;
      lost_cnt[2] = 0;
    }else{
      if(min[2](0) > 0){
        lost_cnt[2] = lost_cnt[2] >= MAX_LOST_CNT? MAX_LOST_CNT: (lost_cnt[2]+1);
        if(lost_cnt[2] < MAX_LOST_CNT){
          min[2] = Fs * min[2];
        }else{
          min[2] = vector6d::Zero(6);
          lost_cnt[2] = 0;
        }
      }else{
        min[2] = vector6d::Zero(6);
      }
    }
    
    // clock_t end = clock();
    // float duration_ms = (float)(end-start)*1000/(float)CLOCKS_PER_SEC;  //程序用时 ms
    // std::cout << "duration(ms) = " << duration_ms << std::endl;
    PubFusionTracks();
}

void RearRadarFusion::GetLocalTracks(void){
    ros::Time left_radar_stamp, right_radar_stamp;
    left_radar_tracker.GetTimeStamp(left_radar_stamp);
    right_radar_tracker.GetTimeStamp(right_radar_stamp);
    left_radar_tracker.GetRadarTrack(left_radar_tracks);
    right_radar_tracker.GetRadarTrack(right_radar_tracks);

    //left_radar补偿
    double dt = (fusion_stamp - left_radar_stamp).toSec();
    matrix6d F = matrix6d::Zero(6,6);
    F(0,0) = F(1,1) = F(2,2) = F(3,3) = F(4,4) = F(5,5) = 1;
    F(0,2) = F(1,3) = F(2,4) = F(3,5) = dt;
    F(0,4) = F(1,5) = dt*dt/2;
    matrix6d Qr = matrix6d::Zero(6,6);
    Qr(0,0) = 0.01;      Qr(1,1) = 0.01;
    Qr(2,2) = 0.05;      Qr(3,3) = 0.05;
    Qr(4,4) = 0.15;      Qr(5,5) = 0.15;
    for(std::vector<LocalTrack>::iterator it= left_radar_tracks.begin(); it!= left_radar_tracks.end(); ++it){
        it->X = F * it->X;
        it->X(1) = it->X(1) + Y_OFFSET;
        it->P = F * it->P * F.transpose() + Qr;
    }
    // std::cout << "left_radar dt = " << dt*1000 <<std::endl;

    //right_radar补偿
    dt = (fusion_stamp - right_radar_stamp).toSec();
    F(0,2) = F(1,3) = F(2,4) = F(3,5) = dt;
    F(0,4) = F(1,5) = dt*dt/2;
    for(std::vector<LocalTrack>::iterator it= right_radar_tracks.begin(); it!= right_radar_tracks.end(); ++it){
        it->X = F * it->X;
        it->X(1) = it->X(1) - Y_OFFSET;
        it->P = F * it->P * F.transpose() + Qr;
    }
    // std::cout << "right_radar dt = " << dt*1000 <<std::endl;
}

void RearRadarFusion::PubFusionTracks(void)
{
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker bbox_marker;
    bbox_marker.header.frame_id = FIXED_FRAME;
    bbox_marker.header.stamp = fusion_stamp;
    bbox_marker.color.r = 1.0f;    //fusion color yellow
    bbox_marker.color.g = 1.0f;
    bbox_marker.color.b = 0.0f;
    bbox_marker.scale.x = PED_WIDTH;
    bbox_marker.scale.y = PED_WIDTH;
    bbox_marker.scale.z = PED_HEIGHT;
    bbox_marker.lifetime = ros::Duration();
    bbox_marker.frame_locked = true;
    bbox_marker.type = visualization_msgs::Marker::CUBE;
    bbox_marker.action = visualization_msgs::Marker::ADD;
    detection::TargetArray target_array;
    detection::Target target;
    target_array.header.stamp = fusion_stamp;
    target_array.num = 3;

    for(int i=0; i<3; ++i){
      bbox_marker.id = i;
      bbox_marker.pose.position.x = min[i](0);
      bbox_marker.pose.position.y = min[i](1);
      bbox_marker.pose.position.z = 0;
      if(min[i](0) == 0){
        bbox_marker.color.a = 0;
        target_array.num--;
      }else{
        bbox_marker.color.a = 0.5f;
      }
      marker_array.markers.push_back(bbox_marker);
      target.rx = min[i](0);
      target.ry = min[i](1);
      target.vx = min[i](2);
      target.vy = min[i](3);
      target.ax = min[i](4);
      target.ay = min[i](5);
      target_array.data[i] = target;
    }

    radar_fusion_rviz_pub.publish(marker_array);
    radar_fusion_pub.publish(target_array);
}