#include <ros/ros.h>
#include <can_msgs/Frame.h>
#include <detection/TargetArray.h>
#include <lidar_grid/DrivableMap.h>

///////////////////////CAN数据发送处理类 目标列表&可行驶区域信息////////////////////////
class CANSendHandler
{
protected:
    ros::NodeHandle nh;
    ros::Subscriber front_target_sub;
    ros::Subscriber rear_target_sub;
    ros::Subscriber drivable_map_sub;
    ros::Publisher can_pub;

public:
    CANSendHandler(void)
    {
        front_target_sub = nh.subscribe("fusion_array", 10, &CANSendHandler::front_cb, this);        //接收话题：fusion_array
        rear_target_sub  = nh.subscribe("radar_fusion_array", 10, &CANSendHandler::rear_cb, this);   //接收话题：radar_fusion_array
        drivable_map_sub = nh.subscribe("drivable_map", 10, &CANSendHandler::drivable_map_cb, this); //接收话题：drivable_map

        can_pub = nh.advertise<can_msgs::Frame>("sent_messages", 100);     //发布话题：sent_messages
    }
    ~CANSendHandler() { }
    void front_cb(const detection::TargetArray& input);
    void rear_cb(const detection::TargetArray& input);
    void drivable_map_cb(const lidar_grid::DrivableMap& input);
};

//////////////////////////前方目标列表CAN发送///////////////////////////
void CANSendHandler::front_cb(const detection::TargetArray& input){
    uint16_t rx;
    int16_t ry, vx, vy;
    can_msgs::Frame f;
    f.dlc = 8;
    f.is_error = false;
    f.is_extended = false;
    f.is_rtr = false;
    f.data[0] = input.num;
    f.header.stamp = ros::Time::now();
    f.id = 0x5A0;  //0x5A0 前方目标个数
    can_pub.publish(f);
    for(int i=0; i<input.num; ++i){
      rx = (uint16_t)(input.data[i].rx * 100);
      ry = (int16_t)(input.data[i].ry * 100);
      vx = (int16_t)(input.data[i].vx * 100);
      vy = (int16_t)(input.data[i].vy * 100);
      f.data[0] = (uint8_t)((0xff00 & rx)>>8);
      f.data[1] = (uint8_t)(0x00ff & rx);
      f.data[2] = (uint8_t)((0xff00 & ry)>>8);
      f.data[3] = (uint8_t)(0x00ff & ry);
      f.data[4] = (uint8_t)((0xff00 & vx)>>8);
      f.data[5] = (uint8_t)(0x00ff & vx);
      f.data[6] = (uint8_t)((0xff00 & vy)>>8);
      f.data[7] = (uint8_t)(0x00ff & vy);
      f.header.stamp = ros::Time::now();
      f.id = 0x5A1 + i;  //0x5A1~0x5AF 前方目标状态
      can_pub.publish(f);
    }
    return;
}

//////////////////////////后方三车道最近目标CAN发送///////////////////////////
void CANSendHandler::rear_cb(const detection::TargetArray& input){
    uint16_t rx;
    int16_t ry, vx, vy;
    can_msgs::Frame f;
    f.dlc = 8;
    f.is_error = false;
    f.is_extended = false;
    f.is_rtr = false;
    for(int i=0; i<3; ++i){
      rx = (uint16_t)(input.data[i].rx * 100);
      ry = (int16_t)(input.data[i].ry * 100);
      vx = (int16_t)(input.data[i].vx * 100);
      vy = (int16_t)(input.data[i].vy * 100);
      f.data[0] = (uint8_t)((0xff00 & rx)>>8);
      f.data[1] = (uint8_t)(0x00ff & rx);
      f.data[2] = (uint8_t)((0xff00 & ry)>>8);
      f.data[3] = (uint8_t)(0x00ff & ry);
      f.data[4] = (uint8_t)((0xff00 & vx)>>8);
      f.data[5] = (uint8_t)(0x00ff & vx);
      f.data[6] = (uint8_t)((0xff00 & vy)>>8);
      f.data[7] = (uint8_t)(0x00ff & vy);
      f.header.stamp = ros::Time::now();
      f.id = 0x5B0 + i;  //0x5B0 右车道后向最近目标 0x5B1 主车道 0x5B2 左车道
      can_pub.publish(f);
    }
    return;
}

//////////////////////////可行驶区域信息CAN发送///////////////////////////
void CANSendHandler::drivable_map_cb(const lidar_grid::DrivableMap& input){
    can_msgs::Frame f;
    f.dlc = 8;
    f.is_error = false;
    f.is_extended = false;
    f.is_rtr = false;
    for(int i=0; i<23; ++i){
      for(int j=0; j<8; ++j){
        if(i*8 +j < 180){
          f.data[j] = input.channel[i*8 +j];
        }
      }
      f.header.stamp = ros::Time::now();
      f.id = 0x5C0 + i;  //0x5C0~0x5D6 可行驶区域信息
      can_pub.publish(f);
    }
    return;
}

////////////////////////////////////////////主函数///////////////////////////////////////////////////
int main(int argc,char** argv)
{
    ros::init(argc,argv,"can_send_node");
    CANSendHandler sender;
    ros::spin();
    return 0;
}