#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
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
#include <detection/object.h>

////////////////////////激光雷达点云目标聚类处理类////////////////////////
class LidarClusterHandler
{
protected:
    ros::NodeHandle nh;
    ros::Subscriber pc_sub;
    ros::Publisher pc_pub;
    ros::Publisher lidar_rviz_pub;
    ros::Publisher lidar_rawArray_pub;
    std::string fixed_frame;
    float ROI_width;          //感兴趣区域宽度
    float ROI_length;         //感兴趣区域长度
    float cut_x;              //裁剪近处自车点x范围
    float cut_y;              //裁剪近处自车点y范围
    float CLUSTER_TOLERANCE;
    int MIN_CLUSTER_SIZE;
    int MAX_CLUSTER_SIZE;
    float OBJECT_MIN_WIDTH;

public:
    LidarClusterHandler()
    {
        pc_sub   = nh.subscribe("velodyne_points", 1, &LidarClusterHandler::cluster, this);  //接收话题：velodyne_points
        pc_pub   = nh.advertise<sensor_msgs::PointCloud2>("no_ground_pc", 1);                //发布话题：no_ground_pc
        lidar_rviz_pub = nh.advertise<visualization_msgs::MarkerArray>("lidar_raw_rviz", 10);
        lidar_rawArray_pub = nh.advertise<detection::LidarRawArray>("lidar_rawArray", 10);

        nh.param<std::string>("/lidar_cluster_node/fixed_frame", fixed_frame, "velodyne");
        nh.param<float>("/lidar_cluster_node/ROI_width", ROI_width, 5.5);
        nh.param<float>("/lidar_cluster_node/ROI_length", ROI_length, 22.0);
        nh.param<float>("/lidar_cluster_node/cut_x", cut_x, 1.5);
        nh.param<float>("/lidar_cluster_node/cut_y", cut_y, 0.8);
        nh.param<float>("/lidar_cluster_node/CLUSTER_TOLERANCE", CLUSTER_TOLERANCE, 0.6);
        nh.param<int>("/lidar_cluster_node/MIN_CLUSTER_SIZE", MIN_CLUSTER_SIZE, 6);
        nh.param<int>("/lidar_cluster_node/MAX_CLUSTER_SIZE", MAX_CLUSTER_SIZE, 500);
        nh.param<float>("/lidar_cluster_node/OBJECT_MIN_WIDTH", OBJECT_MIN_WIDTH, 0.5);
    }
    ~LidarClusterHandler(){ }
    void cluster(const sensor_msgs::PointCloud2ConstPtr& input);
    void PublidarPed(const detection::LidarRawArray& raw_array);
};

void LidarClusterHandler::cluster(const sensor_msgs::PointCloud2ConstPtr& input)
{
    // clock_t start = clock();
    detection::LidarRawArray raw_array;
    raw_array.header.stamp = ros::Time::now();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw_ptr  (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_roi_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_obstacle_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input, *cloud_raw_ptr);

//////////////////////////////遍历输入点云，提取ROI///////////////////////////////
    for (int m=0; m<cloud_raw_ptr->size(); ++m)   
    {
        if(cloud_raw_ptr->points[m].x < 0)   
        {
            continue;  //所有后部的点在栅格地图中排除
        }

        if(cloud_raw_ptr->points[m].z > 0)   
        {
            continue;  //所有高于车辆高度的点在栅格地图中排除
        }
        
        if(cloud_raw_ptr->points[m].x > -cut_x && cloud_raw_ptr->points[m].x < cut_x
           && cloud_raw_ptr->points[m].y > -cut_y && cloud_raw_ptr->points[m].y < cut_y)
        {
            continue;  //裁剪近处自车点
        }
        
        if(cloud_raw_ptr->points[m].x > ROI_length)
        {
            continue;  //排除x坐标大于感兴趣区域外的数据
        }
        
        if(cloud_raw_ptr->points[m].y > ROI_width || cloud_raw_ptr->points[m].y < -ROI_width)
        {
            continue;  //排除y坐标绝对值大于感兴趣宽度外的数据
        }
        cloud_roi_ptr->points.push_back(cloud_raw_ptr->points[m]);  //提取感兴趣区域点云
    }

    //应用随机采样一致性算法得到原始地面法向量,并滤除地面点云
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setOptimizeCoefficients (true);
    seg.setMaxIterations(500);
    seg.setDistanceThreshold (0.2);
    seg.setInputCloud (cloud_roi_ptr);
    seg.segment (*inliers, *coefficients);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_roi_ptr);
    extract.setIndices(inliers);
    extract.setNegative (true);
    extract.filter (*cloud_obstacle_ptr);    //滤除地面得到原始障碍物点云

    sensor_msgs::PointCloud2 pcl_output;
    pcl::toROSMsg(*cloud_obstacle_ptr, pcl_output);
    pcl_output.header.frame_id = fixed_frame;
    pcl_output.header.stamp = input->header.stamp;
    pc_pub.publish(pcl_output);              //发布原始障碍物点云

    if(cloud_obstacle_ptr->size() >= MIN_CLUSTER_SIZE){//A
        //障碍目标点云聚类 欧氏聚类
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
            // vector4d centroid;
            pcl::PointXYZ minpoint, maxpoint;
            // pcl::compute3DCentroid (*cloud_cluster, centroid);
            pcl::getMinMax3D (*cloud_cluster, minpoint, maxpoint);
            // raw.x = centroid[0];
            // raw.y = centroid[1];
            raw.x = (maxpoint.x + minpoint.x)/2;
            raw.y = (maxpoint.y + minpoint.y)/2;
            raw.width = maxpoint.y - minpoint.y;
            if(raw.width < OBJECT_MIN_WIDTH) raw.width = OBJECT_MIN_WIDTH;
            if(mark >= 15) ROS_ERROR("lidar raw num > 15");
            raw_array.data[mark++] = raw;
            raw_array.num = mark;
        }
    }//A
    else{
        raw_array.num = 0;
    }
    lidar_rawArray_pub.publish(raw_array);
    PublidarPed(raw_array);
    // clock_t end = clock();
    // float duration_ms = (float)(end-start)*1000/(float)CLOCKS_PER_SEC;  //程序用时 ms
    // std::cout << "duration(ms) = " << duration_ms << std::endl;
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
    bbox_marker.color.a = 0.5f;
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