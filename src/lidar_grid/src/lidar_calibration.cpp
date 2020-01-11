#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <cmath>
#include <time.h>
#include <Eigen/Dense>

class Grid
{
public:
    Grid(void) : grid_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>) { }
    ~Grid() { }
    pcl::PointCloud<pcl::PointXYZ>::Ptr grid_cloud_ptr;
};

///////////////////////激光雷达点云地平面校正处理类////////////////////////
class LidarCloudHandler
{
protected:
    ros::NodeHandle nh;
    ros::Subscriber pc_sub;
    ros::Publisher pc_pub;
    ros::Publisher ego_pub;
    ros::Publisher time_pub;
    std::string fixed_frame;
    int R;                    //半径上的分割数
    int TH;                   //角度上的分割数
    float grid_size_r;        //半径上的分辨率
    float grid_size_th;       //角度上的分辨率
    float grid_size;          //输出栅格大小
    float threshold;         //地面点阈值
    float cut_x;              //裁剪近处自车点x范围
    float cut_y;              //裁剪近处自车点y范围
    int y_width;              //输出栅格地图一侧宽度
    int x_forward;            //输出栅格地图前向长度
    float RANSAC_threshold;   //RANSAC阈值

public:
    LidarCloudHandler(void)
    {
        pc_sub = nh.subscribe("velodyne_points", 1, &LidarCloudHandler::calibration, this);   //接收话题：velodyne_points
        pc_pub = nh.advertise<sensor_msgs::PointCloud2>("cali_pc", 1);                        //发布话题：cali_pc
        time_pub = nh.advertise<std_msgs::Float32>("cali_time", 1);                           //发布话题：cali_time

        nh.param<std::string>("/lidar_calibration/fixed_frame", fixed_frame, "velodyne");
        nh.param<int>("/lidar_calibration/R", R, 60);
        nh.param<int>("/lidar_calibration/TH", TH, 180);
        nh.param<float>("/lidar_calibration/grid_size_r", grid_size_r, 0.4);
        nh.param<float>("/lidar_calibration/grid_size", grid_size, 0.2);
        nh.param<float>("/lidar_calibration/threshold", threshold, 0.15);
        nh.param<float>("/lidar_calibration/cut_x", cut_x, 1.5);
        nh.param<float>("/lidar_calibration/cut_y", cut_y, 0.8);
        nh.param<int>("/lidar_calibration/y_width", y_width, 50);
        nh.param<int>("/lidar_calibration/x_forward", x_forward, 100);
        nh.param<float>("/lidar_calibration/RANSAC_threshold", RANSAC_threshold, 0.2);
        grid_size_th = 2*M_PI/TH;
    }
    ~LidarCloudHandler() { }
    void calibration(const sensor_msgs::PointCloud2ConstPtr& input);
};

//////////////////////////激光雷达点云地平面校正函数///////////////////////////
void LidarCloudHandler::calibration(const sensor_msgs::PointCloud2ConstPtr& input)
{
    clock_t start = clock();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw_ptr  (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_reduce_ptr  (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_trans_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input, *cloud_raw_ptr);

    Grid grid[R][TH];      //建立极坐标栅格地图grid
    float r = 0, th = 0;
    int a = 0, t = 0;

//////////////////////////////遍历输入点云，初步筛选，将点划入极坐标栅格内///////////////////////////////
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
        
        if(cloud_raw_ptr->points[m].x > x_forward*grid_size)
        {
            continue;  //排除x坐标大于感兴趣区域的数据
        }

        if(cloud_raw_ptr->points[m].y > y_width*grid_size || cloud_raw_ptr->points[m].y < -y_width*grid_size)
        {
            continue;  //排除y坐标绝对值大于感兴趣区域的数据
        }
        
        r = sqrt(cloud_raw_ptr->points[m].x * cloud_raw_ptr->points[m].x
                + cloud_raw_ptr->points[m].y * cloud_raw_ptr->points[m].y);  //XY平面内，点到原点的距离

        th = acos(cloud_raw_ptr->points[m].x/r);//点对应的向量的角度（以x轴正方向为零角度）
    
        if(cloud_raw_ptr->points[m].y<0)
        {
            th = 2*M_PI-th;   //角度范围 车周0~TH度
        }      

        t = (int)floor(th/grid_size_th) % TH;   //坐标转换为极坐标栅格地图坐标
        a = floor(r/grid_size_r);
        a = a >= R? R-1 : a;
        grid[a][t].grid_cloud_ptr->points.push_back(cloud_raw_ptr->points[m]);  //把该点划入对应栅格内
        cloud_reduce_ptr->push_back(cloud_raw_ptr->points[m]);
    }

/////////////////////////////////初步筛选出地面点云和障碍物点云/////////////////////////////
    pcl::PointXYZ minpoint, maxpoint;
    float h = 0;
    
    for (int i=0; i<R; ++i)      //遍历栅格地图grid
    {
        for(int j=0; j<TH; ++j)
        {
            if(grid[i][j].grid_cloud_ptr->size()>0)    //如果某栅格内有数据点
            {
                pcl::getMinMax3D (*grid[i][j].grid_cloud_ptr, minpoint, maxpoint);   //这里的最大最小是值，不是对应一个点
                h = maxpoint.z - minpoint.z;                                         //该栅格内数据点的最大高度差
                
                if (h<threshold)  //最大高度差小于阈值的点归为地面候选点
                {
                    for(int m=0; m<grid[i][j].grid_cloud_ptr->size(); ++m)
                    {
                        cloud_temp_ptr->push_back(grid[i][j].grid_cloud_ptr->points[m]);      //地面候选点云
                    }       
                }
            }
        }
    }

    //应用随机采样一致性算法进一步提取地面点云，得到原始地面法向量
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setOptimizeCoefficients (true);
    seg.setMaxIterations(500);
    seg.setDistanceThreshold (RANSAC_threshold);
    seg.setInputCloud (cloud_temp_ptr);
    seg.segment (*inliers, *coefficients);
    
    Eigen::Vector3f before, after;
    before(0) = coefficients->values[0];
    before(1) = coefficients->values[1];
    before(2) = coefficients->values[2];
    after(0) = 0;
    after(1) = 0;
    after(2) = 1;
    before.normalize();
    float angle = acos(before.dot(after));            //旋转角
    Eigen::Vector3f p_rotate =before.cross(after);    //旋转轴
    p_rotate.normalize();

    //应用罗德里格旋转公式计算旋转矩阵
    Eigen::Matrix4f rotationMatrix = Eigen::Matrix4f::Identity();
    rotationMatrix(0, 0) = cos(angle) + p_rotate[0] * p_rotate[0] * (1 - cos(angle));
    rotationMatrix(0, 1) = p_rotate[0] * p_rotate[1] * (1 - cos(angle)) - p_rotate[2] * sin(angle);
    rotationMatrix(0, 2) = p_rotate[1] * sin(angle) + p_rotate[0] * p_rotate[2] * (1 - cos(angle));
 
    rotationMatrix(1, 0) = p_rotate[2] * sin(angle) + p_rotate[0] * p_rotate[1] * (1 - cos(angle));
    rotationMatrix(1, 1) = cos(angle) + p_rotate[1] * p_rotate[1] * (1 - cos(angle));
    rotationMatrix(1, 2) = -p_rotate[0] * sin(angle) + p_rotate[1] * p_rotate[2] * (1 - cos(angle));
 
    rotationMatrix(2, 0) = -p_rotate[1] * sin(angle) + p_rotate[0] * p_rotate[2] * (1 - cos(angle));
    rotationMatrix(2, 1) = p_rotate[0] * sin(angle) + p_rotate[1] * p_rotate[2] * (1 - cos(angle));
    rotationMatrix(2, 2) = cos(angle) + p_rotate[2] * p_rotate[2] * (1 - cos(angle));
    pcl::transformPointCloud(*cloud_reduce_ptr, *cloud_trans_ptr, rotationMatrix);

    sensor_msgs::PointCloud2 pcl_output;
    pcl::toROSMsg(*cloud_trans_ptr, pcl_output);
    pcl_output.header.frame_id = fixed_frame;
    pcl_output.header.stamp = input->header.stamp;
    pc_pub.publish(pcl_output);  //发布校正点云

    clock_t end = clock();
    std_msgs::Float32 cali_time;
    cali_time.data = (float)(end-start)*1000/(float)CLOCKS_PER_SEC;  //程序用时 ms
    time_pub.publish(cali_time);  //发布程序耗时
}

////////////////////////////////////////////主函数///////////////////////////////////////////////////
int main(int argc,char** argv)
{
    ros::init(argc,argv,"lidar_calibration");
    LidarCloudHandler handler;
    ros::spin();
    return 0;
}