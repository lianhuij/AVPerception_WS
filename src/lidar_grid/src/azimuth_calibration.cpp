#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <visualization_msgs/Marker.h>

///////////////////////激光雷达点云方位角标定处理类////////////////////////
class LidarCloudHandler
{
protected:
    ros::NodeHandle nh;
    ros::Subscriber pc_sub;
    ros::Publisher pc_pub;
    ros::Publisher ego_pub;
    std::string fixed_frame;
    float cut_x;              //裁剪近处自车点x范围
    float cut_y;              //裁剪近处自车点y范围

public:
    LidarCloudHandler()
    {
        pc_sub = nh.subscribe("velodyne_points", 1, &LidarCloudHandler::azi_calibration, this);   //接收话题：velodyne_points
        pc_pub = nh.advertise<sensor_msgs::PointCloud2>("azi_pc", 1);                             //发布话题：azi_pc
        ego_pub = nh.advertise<visualization_msgs::Marker>("ego_car", 1);                         //发布话题：ego_car 几何形状

        nh.getParam("/azimuth_calibration/fixed_frame", fixed_frame);
        nh.getParam("/azimuth_calibration/cut_x", cut_x);
        nh.getParam("/azimuth_calibration/cut_y", cut_y);
    }

    void azi_calibration(const sensor_msgs::PointCloud2& input);
};

//////////////////////////激光雷达点云方位角标定函数///////////////////////////
void LidarCloudHandler::azi_calibration(const sensor_msgs::PointCloud2& input)
{
    int i = 0;
    int m = 0;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw_ptr  (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_zero_ptr  (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_other_ptr (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromROSMsg(input, *cloud_raw_ptr);

//////////////////////////////遍历输入点云，找出零方位角点云///////////////////////////////
    for (m=0; m<cloud_raw_ptr->size(); ++m)   
    {
        if(cloud_raw_ptr->points[m].x > -cut_x && cloud_raw_ptr->points[m].x < cut_x
           && cloud_raw_ptr->points[m].y > -cut_y && cloud_raw_ptr->points[m].y < cut_y)
        {
            continue;  //裁剪近处自车点
        }

        if(cloud_raw_ptr->points[m].z > 0)   
        {
            continue;  //排除所有高于车辆高度的点
        }

        if(cloud_raw_ptr->points[m].x < 0)   
        {
            continue;  //排除所有后部的点
        }      

        if(cloud_raw_ptr->points[m].y>-0.1 && cloud_raw_ptr->points[m].y<0.1)
        {
            cloud_zero_ptr->push_back(cloud_raw_ptr->points[m]);
        }
        else
        {
            cloud_other_ptr->push_back(cloud_raw_ptr->points[m]);
        }
    }

///////////////////////////////设置各个部分点云的颜色///////////////////////////////////
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_final_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB> color;
    color.points.resize(1);

    for (i=0; i<cloud_zero_ptr->size(); ++i)
    {
        color.points[0].x = cloud_zero_ptr->points[i].x;
        color.points[0].y = cloud_zero_ptr->points[i].y;
        color.points[0].z = cloud_zero_ptr->points[i].z;

        color.points[0].r = 0;      
        color.points[0].g = 255;    //设置零方位角点云为绿色
        color.points[0].b = 0;
        cloud_final_ptr->push_back(color.points[0]);
    }

    for (i=0; i<cloud_other_ptr->size(); ++i)
    {
        color.points[0].x = cloud_other_ptr->points[i].x;
        color.points[0].y = cloud_other_ptr->points[i].y;
        color.points[0].z = cloud_other_ptr->points[i].z;

        color.points[0].r = 0;      
        color.points[0].g = 0;    
        color.points[0].b = 255;    //设置其他点云为蓝色
        cloud_final_ptr->push_back(color.points[0]);
    }

    sensor_msgs::PointCloud2 pcl_output;
    pcl::toROSMsg(*cloud_final_ptr, pcl_output);
    pcl_output.header.frame_id = fixed_frame;

    pc_pub.publish(pcl_output);  //发布零方位角点云

    visualization_msgs::Marker marker;

    marker.header.frame_id = fixed_frame;
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
    marker.scale.x = 3.2;
    marker.scale.y = 1.7;
    marker.scale.z = 1.7;
    marker.color.r = 0;
    marker.color.g = 0;
    marker.color.b = 0.5;
    marker.color.a = 0.7;
    marker.lifetime = ros::Duration();

    ego_pub.publish(marker);  //发布自车几何形状
}

////////////////////////////////////////////主函数///////////////////////////////////////////////////
int main(int argc,char** argv)
{
    ros::init(argc,argv,"azimuth_calibration");

    LidarCloudHandler handler;
        
    ros::spin();

    return 0;
}