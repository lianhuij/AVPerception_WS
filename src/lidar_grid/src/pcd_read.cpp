#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>

main(int argc, char **argv)
{
    ros::init (argc, argv, "pcd_read");
    ros::NodeHandle nh;
    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("velodyne_points", 1);  //发布话题：velodyne_points

    sensor_msgs::PointCloud2 output;
    pcl::PointCloud<pcl::PointXYZ> cloud;

    std::string pcd_file;
    std::string fixed_frame;
    int pub_hz;
    nh.getParam("/pcd_read/pcd_file", pcd_file);
    nh.getParam("/pcd_read/fixed_frame", fixed_frame);
    nh.getParam("/pcd_read/pub_hz", pub_hz);
    
    pcl::io::loadPCDFile (pcd_file, cloud);   //从硬盘加载点云文件
    pcl::toROSMsg(cloud, output);
    output.header.frame_id = fixed_frame;

    ros::Rate loop_rate(pub_hz);

    while (ros::ok())
    {
        pcl_pub.publish(output);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}