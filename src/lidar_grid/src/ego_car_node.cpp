#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main (int argc, char** argv){
  ros::init(argc,argv,"ego_car_node");
  ros::NodeHandle nh;
  ros::Publisher ego_pub = nh.advertise<visualization_msgs::Marker>("ego_car", 1);
  ros::Rate loop_rate(10);  //10hz
  while( ros::ok() ){
    visualization_msgs::Marker marker;
    marker.header.frame_id = "velodyne";
    marker.header.stamp = ros::Time::now();
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = -0.9;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 3;
    marker.scale.y = 1.6;
    marker.scale.z = 1.6;
    marker.color.r = 0;
    marker.color.g = 0;
    marker.color.b = 0.7;
    marker.color.a = 0.7;
    marker.lifetime = ros::Duration();
    ego_pub.publish(marker);  //发布自车几何形状
    loop_rate.sleep();
  }
  return 0;
}