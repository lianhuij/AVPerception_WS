#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <detection/LidarRaw.h>
#include <detection/LidarRawArray.h>
#include "detection/dbscan.h"
#include <cmath>
#include <time.h>
#include <vector>

// const float LIDAR_EPS  = 0.5;
// const int LIDAR_MINPTS = 6;
const float CLUSTER_TOLERANCE = 0.6;
const int MIN_CLUSTER_SIZE    = 7;
const int MAX_CLUSTER_SIZE    = 400;

///////////////////////激光雷达点云目标聚类处理类////////////////////////
class LidarClusterHandler
{
protected:
    ros::NodeHandle nh;
    ros::Subscriber pc_sub;
    ros::Publisher pc_pub;
    ros::Publisher lidar_rviz_pub;
    ros::Publisher lidar_rawArray_pub;
    // ros::Publisher time_pub;
    std::string fixed_frame;
    float ROI_width;

public:
    LidarClusterHandler()
    {
        pc_sub   = nh.subscribe("cali_pc", 1, &LidarClusterHandler::cluster, this);  //接收话题：cali_pc
        pc_pub   = nh.advertise<sensor_msgs::PointCloud2>("no_ground_pc", 1);        //发布话题：no_ground_pc
        lidar_rviz_pub = nh.advertise<visualization_msgs::MarkerArray>("lidar_raw_rviz", 10);
        lidar_rawArray_pub = nh.advertise<detection::LidarRawArray>("lidar_rawArray", 10);
        // time_pub = nh.advertise<std_msgs::Float32>("lidar_cluster_time", 1);         //发布话题：lidar_cluster_time

        nh.getParam("/lidar_cluster_node/fixed_frame", fixed_frame);
        nh.getParam("/lidar_cluster_node/ROI_width", ROI_width);
    }
    ~LidarClusterHandler(){ }
    void cluster(const sensor_msgs::PointCloud2ConstPtr& input);
    void PublidarPed(const detection::LidarRawArray& raw_array);
};

void LidarClusterHandler::cluster(const sensor_msgs::PointCloud2ConstPtr& input)
{
    // clock_t start = clock();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw_ptr  (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_roi_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_obstacle_ptr (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromROSMsg(*input, *cloud_raw_ptr);

//////////////////////////////遍历输入点云，提取ROI///////////////////////////////
    for (int m=0; m<cloud_raw_ptr->size(); ++m)   
    {
        if(cloud_raw_ptr->points[m].y > ROI_width || cloud_raw_ptr->points[m].y < -ROI_width)
        {
            continue;
        }
        cloud_roi_ptr->points.push_back(cloud_raw_ptr->points[m]);
    }

    //应用随机采样一致性算法滤除地面点云
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices());

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setOptimizeCoefficients (true);
    seg.setMaxIterations(500);
    seg.setDistanceThreshold (0.15);
    seg.setInputCloud (cloud_roi_ptr);
    seg.segment (*inliers, *coefficients);

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_roi_ptr);
    extract.setIndices(inliers);
    extract.setNegative (true);
    extract.filter (*cloud_obstacle_ptr);

    sensor_msgs::PointCloud2 pcl_output;
    pcl::toROSMsg(*cloud_obstacle_ptr, pcl_output);
    pcl_output.header.frame_id = fixed_frame;
    pcl_output.header.stamp = input->header.stamp;
    pc_pub.publish(pcl_output);

    // //障碍目标点云聚类 DBSCAN
    // std::vector<Point> vec_pts;
    // for (int i=0; i<cloud_obstacle_ptr->size(); ++i){
    //     vec_pts.push_back({cloud_obstacle_ptr->points[i].x, cloud_obstacle_ptr->points[i].y, 0, NOT_CLASSIFIED});
    // }
    // DBSCAN dbScan(LIDAR_EPS, LIDAR_MINPTS, vec_pts);    //原始目标聚类
    // dbScan.run();
    // std::vector<std::vector<int> > idx = dbScan.getCluster();
    // detection::LidarRawArray raw_array;
    // detection::LidarRaw raw;
    // for(int i=0; i<idx.size(); ++i){
    //     raw.x = raw.y = 0;
    //     int size = idx[i].size();
    //     for(int j=0; j<size; ++j){
    //         raw.x += cloud_obstacle_ptr->points[idx[i][j]].x;
    //         raw.y += cloud_obstacle_ptr->points[idx[i][j]].y;
    //     }
    //     raw.x /= size;
    //     raw.y /= size;
    //     if(i >= 15) ROS_ERROR("lidar raw num > 15");
    //     raw_array.data[i] = raw;
    //     raw_array.num = i+1;
    // }

    //障碍目标点云聚类 欧氏聚类
    detection::LidarRawArray raw_array;
    if(cloud_obstacle_ptr->size() > 0){
        std::vector<pcl::PointIndices> cluster_indices;
        // Creating the KdTree object for the search method of the extraction
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud (cloud_obstacle_ptr);
        // ClusterExtraction
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance (CLUSTER_TOLERANCE);
        ec.setMinClusterSize (MIN_CLUSTER_SIZE);
        ec.setMaxClusterSize (MAX_CLUSTER_SIZE);
        ec.setSearchMethod (tree);
        ec.setInputCloud (cloud_obstacle_ptr);
        ec.extract (cluster_indices);

        detection::LidarRaw raw;
        int mark = 0;
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++) {
                cloud_cluster->points.push_back (cloud_obstacle_ptr->points[*pit]);
            }
            Eigen::Vector4f centroid;
            pcl::PointXYZ minpoint, maxpoint;
            pcl::compute3DCentroid (*cloud_cluster, centroid);
            pcl::getMinMax3D (*cloud_cluster, minpoint, maxpoint);
            raw.x = centroid[0];
            raw.y = centroid[1];
            raw.width = maxpoint.y - minpoint.y;
            if(raw.width < 0.5) raw.width = 0.5;
            if(mark >= 15) ROS_ERROR("lidar raw num > 15");
            raw_array.data[mark++] = raw;
            raw_array.num = mark;
        }
    }else{
        raw_array.num = 0;
    }
    
    raw_array.header.stamp = input->header.stamp;
    lidar_rawArray_pub.publish(raw_array);
    PublidarPed(raw_array);
    // clock_t end = clock();
    // std_msgs::Float32 time;
    // time.data = (float)(end-start)*1000/(float)CLOCKS_PER_SEC;  //程序用时 ms
    // time_pub.publish(time);   //发布程序耗时
}

void LidarClusterHandler::PublidarPed(const detection::LidarRawArray& raw_array){
    static int pre_marker_size_ = 0;
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker bbox_marker;
    bbox_marker.header.frame_id = fixed_frame;
    bbox_marker.header.stamp = raw_array.header.stamp;
    bbox_marker.color.r = 0.0f;
    bbox_marker.color.g = 1.0f;    //lidar color green
    bbox_marker.color.b = 0.0f;
    bbox_marker.color.a = 0.5;
    bbox_marker.lifetime = ros::Duration();
    bbox_marker.frame_locked = true;
    bbox_marker.type = visualization_msgs::Marker::CUBE;
    bbox_marker.action = visualization_msgs::Marker::ADD;

    int marker_id = 0;
    int track_num = raw_array.num;
    for (int i=0; i<track_num; ++i)
    {
        bbox_marker.id = marker_id++;
        bbox_marker.pose.position.x = raw_array.data[i].x;
        bbox_marker.pose.position.y = raw_array.data[i].y;
        bbox_marker.pose.position.z = -0.9;
        bbox_marker.scale.x = PED_WIDTH;
        bbox_marker.scale.y = raw_array.data[i].width;
        bbox_marker.scale.z = PED_HEIGHT;
        marker_array.markers.push_back(bbox_marker);
    }

    if (track_num > pre_marker_size_)
    {
        pre_marker_size_ = track_num;
    }

    for (int i = marker_id; i < pre_marker_size_; ++i)
    {
        bbox_marker.id = i;
        bbox_marker.action = visualization_msgs::Marker::DELETE;
        marker_array.markers.push_back(bbox_marker);
    }
    pre_marker_size_ = marker_id;
    lidar_rviz_pub.publish(marker_array);
}

////////////////////////////////////////////主函数///////////////////////////////////////////////////
int main(int argc,char** argv)
{
    ros::init(argc,argv,"lidar_cluster_node");

    LidarClusterHandler handler;
        
    ros::spin();

    return 0;
}