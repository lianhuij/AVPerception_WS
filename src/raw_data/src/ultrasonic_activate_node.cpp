#include <ros/ros.h>
#include <can_msgs/Frame.h>
#include <raw_data/Ultrasonic.h>

void can_handler(const can_msgs::Frame& input);

ros::Subscriber can_sub;  // ROS Subscriber
ros::Publisher ultrasonic_ctrl_pub, left_ultrasonic_pub, right_ultrasonic_pub;  // ROS Publisher

int main (int argc, char** argv){
  ros::init(argc,argv,"ultrasonic_activate_node");
  ros::NodeHandle nh;
  can_sub = nh.subscribe("received_messages", 50, can_handler);
  ultrasonic_ctrl_pub = nh.advertise<can_msgs::Frame>("sent_messages", 10);
  left_ultrasonic_pub = nh.advertise<raw_data::Ultrasonic>("left_ultrasonic_raw", 10);  //发布话题：left_ultrasonic_raw
  right_ultrasonic_pub = nh.advertise<raw_data::Ultrasonic>("right_ultrasonic_raw", 10);  //发布话题：right_ultrasonic_raw
  int ultrasonic_mode;
  while( ros::ok() ){
    nh.param<int>("/ultrasonic_activate_node/ultrasonic_mode", ultrasonic_mode, 0xb5);
    can_msgs::Frame f;
    f.id = 0x601;
    f.dlc = 3;
    f.is_error = f.is_extended = f.is_rtr = false;
    f.data[0] = ultrasonic_mode;
    f.data[1] = 0x10;
    f.data[2] = 0xff;
    ultrasonic_ctrl_pub.publish(f);
    ros::spinOnce();
  }
  return 0;
}

void can_handler(const can_msgs::Frame& input){
  //////////////////////////////解析CAN消息 ultrasonic///////////////////////////////
    if(input.id == 0x617 || input.id == 0x618){
        raw_data::Ultrasonic raw;
        raw.header.stamp = ros::Time::now();
        char x[8];
        for(int i=0; i<8; ++i){
            x[i] = ((input.data[i] & 0xF0) >> 4)*10 + (input.data[i] & 0x0F);
        }
        raw.probe[0] = (float)(x[0]*100 + x[1])/1000.0;
        raw.probe[1] = (float)(x[2]*100 + x[3])/1000.0;
        raw.probe[2] = (float)(x[4]*100 + x[5])/1000.0;
        raw.probe[3] = (float)(x[6]*100 + x[7])/1000.0;
        if(input.id == 0x617){
            left_ultrasonic_pub.publish(raw);
        }else if(input.id == 0x618){
            right_ultrasonic_pub.publish(raw);
        }
    }
    return;
}