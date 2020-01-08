#include <ros/ros.h>
#include <can_msgs/Frame.h>

int main (int argc, char** argv){
  ros::init(argc,argv,"ultrasonic_activate_node");
  ros::NodeHandle nh;
  ros::Publisher ultrasonic_ctrl_pub = nh.advertise<can_msgs::Frame>("sent_messages", 10);
  ros::Rate loop_rate(10);  //10hz
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
    loop_rate.sleep();
  }
  return 0;
}