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
    std::string fixed_frame;
    float cut_x;              //裁剪近处自车点x范围
    float cut_y;              //裁剪近处自车点y范围

public:
    LidarCloudHandler(void)
    {
        pc_sub = nh.subscribe("velodyne_points", 1, &LidarCloudHandler::azi_calibration, this);   //接收话题：velodyne_points
        pc_pub = nh.advertise<sensor_msgs::PointCloud2>("azi_pc", 1);                             //发布话题：azi_pc

        nh.getParam("/azimuth_calibration/fixed_frame", fixed_frame);
        nh.getParam("/azimuth_calibration/cut_x", cut_x);
        nh.getParam("/azimuth_calibration/cut_y", cut_y);
    }
    ~LidarCloudHandler() { }
    void azi_calibration(const sensor_msgs::PointCloud2ConstPtr& input);
};

//////////////////////////激光雷达点云方位角标定函数///////////////////////////
void LidarCloudHandler::azi_calibration(const sensor_msgs::PointCloud2ConstPtr& input)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw_ptr  (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_zero_ptr  (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_other_ptr (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromROSMsg(*input, *cloud_raw_ptr);

//////////////////////////////遍历输入点云，找出零方位角点云///////////////////////////////
    for (int m=0; m<cloud_raw_ptr->size(); ++m)   
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
    for (int i=0; i<cloud_zero_ptr->size(); ++i)
    {
        color.points[0].x = cloud_zero_ptr->points[i].x;
        color.points[0].y = cloud_zero_ptr->points[i].y;
        color.points[0].z = cloud_zero_ptr->points[i].z;

        color.points[0].r = 0;      
        color.points[0].g = 255;    //设置零方位角点云为绿色
        color.points[0].b = 0;
        cloud_final_ptr->push_back(color.points[0]);
    }
    for (int i=0; i<cloud_other_ptr->size(); ++i)
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
}

////////////////////////////////////////////主函数///////////////////////////////////////////////////
int main(int argc,char** argv)
{
    ros::init(argc,argv,"azimuth_calibration");
    LidarCloudHandler handler;
    ros::spin();
    return 0;
}