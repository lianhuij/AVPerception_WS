#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/GridCells.h>
#include <std_msgs/Float32.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <cmath>
#include <time.h>
#include "raw_data/Ultrasonic.h"
#include "lidar_grid/DrivableMap.h"

enum RoadState {UNKNOWN, GROUND, OBSTACLE};

class Grid
{
public:
    Grid(void) : grid_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>), IsDrivable(false), IsChecked(false), state(UNKNOWN), avg_z(0) { }
    ~Grid() { }
    pcl::PointCloud<pcl::PointXYZ>::Ptr grid_cloud_ptr;
    bool IsDrivable;
    bool IsChecked;
    enum RoadState state;
    float avg_z;
};

///////////////////////激光雷达点云栅格化处理类 梯度方法////////////////////////
class LidarCloudHandler
{
protected:
    ros::NodeHandle nh;
    ros::Subscriber pc_sub;
    ros::Subscriber left_ultrasonic_sub;
    ros::Subscriber right_ultrasonic_sub;
    ros::Publisher grid_pub;
    ros::Publisher time_pub;
    ros::Publisher map_pub;
    std::string fixed_frame;
    float left_ultrasonic[4];
    float right_ultrasonic[4];
    int R;                    //半径上的分割数
    int TH;                   //角度上的分割数
    float grid_size_r;        //半径上的分辨率
    float grid_size_th;       //角度上的分辨率
    float radius;             //半径
    float grid_size;          //输出栅格大小
    float threshold;         //地面点阈值
    float ground_z;           //自车位置地面z坐标值
    float max_gradient;       //最大坡度阈值
    float cut_width;          //可通行域裁剪宽度
    int y_width;              //输出栅格地图一侧宽度
    int x_forward;            //输出栅格地图前向长度
    int x_backward;           //输出栅格地图后向长度

public:
    LidarCloudHandler(void)
    {
        pc_sub = nh.subscribe("cali_pc", 1, &LidarCloudHandler::rasterization, this);     //接收话题：cali_pc
        left_ultrasonic_sub  = nh.subscribe("left_ultrasonic", 10, &LidarCloudHandler::left_ultrasonic_cb, this);   //接收话题：left_ultrasonic
        right_ultrasonic_sub = nh.subscribe("right_ultrasonic", 10, &LidarCloudHandler::right_ultrasonic_cb, this); //接收话题：right_ultrasonic

        grid_pub = nh.advertise<nav_msgs::GridCells>("grid_cell", 1);                     //发布话题：grid_cell
        time_pub = nh.advertise<std_msgs::Float32>("time_gradient", 1);                   //发布话题：time_gradient
        map_pub  = nh.advertise<lidar_grid::DrivableMap>("drivable_map", 1);              //发布话题：drivable_map

        nh.param<std::string>("/lidar_grid2/fixed_frame", fixed_frame, "velodyne");
        nh.param<int>("/lidar_grid2/R", R, 60);
        nh.param<int>("/lidar_grid2/TH", TH, 180);
        nh.param<float>("/lidar_grid2/grid_size_r", grid_size_r, 0.4);
        nh.param<float>("/lidar_grid2/grid_size", grid_size, 0.2);
        nh.param<float>("/lidar_grid2/threshold", threshold, 0.15);
        nh.param<float>("/lidar_grid2/ground_z", ground_z, -1.8);
        nh.param<float>("/lidar_grid2/max_gradient", max_gradient, 0.17);
        nh.param<float>("/lidar_grid2/cut_width", cut_width, 1.7);
        nh.param<int>("/lidar_grid2/y_width", y_width, 50);
        nh.param<int>("/lidar_grid2/x_forward", x_forward, 100);
        nh.param<int>("/lidar_grid2/x_backward", x_backward, 0);
        grid_size_th = 2*M_PI/TH;
        radius = R*grid_size_r;
        for(int i=0; i<4; ++i){
            left_ultrasonic[i] = 5.0;
            right_ultrasonic[i] = 5.0;
        }
    }
    ~LidarCloudHandler() { }
    void rasterization(const sensor_msgs::PointCloud2ConstPtr& input);
    void left_ultrasonic_cb(const raw_data::Ultrasonic& input);
    void right_ultrasonic_cb(const raw_data::Ultrasonic& input);
};

void LidarCloudHandler::left_ultrasonic_cb(const raw_data::Ultrasonic& input){
    for(int i=0; i<4; ++i){
        left_ultrasonic[i] = input.probe[i];
    }
}

void LidarCloudHandler::right_ultrasonic_cb(const raw_data::Ultrasonic& input){
    for(int i=0; i<4; ++i){
        right_ultrasonic[i] = input.probe[i];
    }
}

//////////////////////////激光雷达点云栅格化及地面可通行区域提取函数///////////////////////////
void LidarCloudHandler::rasterization(const sensor_msgs::PointCloud2ConstPtr& input)
{
    nh.param<float>("/lidar_grid2/ground_z", ground_z, -1.8);
    nh.param<float>("/lidar_grid2/max_gradient", max_gradient, 0.17);
    clock_t start = clock();
    nav_msgs::GridCells grid_cell;
    grid_cell.header.frame_id = fixed_frame;  //rviz中的fixed frame
    grid_cell.cell_height = grid_size;        //栅格大小
    grid_cell.cell_width = grid_size;
    geometry_msgs::Point free_road;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw_ptr  (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input, *cloud_raw_ptr);

    Grid grid[R][TH];      //建立极坐标栅格地图grid
    float r = 0, th = 0;
    int a = 0, t = 0;

    //在输入点云中加入超声波数据点
    pcl::PointXYZ ultrasonic_point;
    ultrasonic_point.z = 0;
    if(left_ultrasonic[1] > 0 && left_ultrasonic[1] < 5.0 && left_ultrasonic[2] > 0 && left_ultrasonic[2] < 5.0){
        if(fabs(left_ultrasonic[1] - left_ultrasonic[2]) < 0.2){
            ultrasonic_point.x = 1.6;
            ultrasonic_point.y = 0.75 + left_ultrasonic[1];
            cloud_raw_ptr->push_back(ultrasonic_point);
        }
    }
    if(right_ultrasonic[1] > 0 && right_ultrasonic[1] < 5.0 && right_ultrasonic[2] > 0 && right_ultrasonic[2] < 5.0){
        if(fabs(right_ultrasonic[1] - right_ultrasonic[2]) < 0.2){
            ultrasonic_point.x = 1.6;
            ultrasonic_point.y = -0.75 - right_ultrasonic[1];
            cloud_raw_ptr->push_back(ultrasonic_point);
        }
    }
    for(int i=0; i<4; ++i){
        switch (i)
        {
        case 0: ultrasonic_point.x = 1.0;   break;
        case 1: ultrasonic_point.x = 0.65;  break;
        case 2: ultrasonic_point.x = -0.3;  break;
        case 3: ultrasonic_point.x = -1.05; break;
        default: break;
        }
        if(left_ultrasonic[i] > 0 && left_ultrasonic[i] < 5.0){
            ultrasonic_point.y = 0.75 + left_ultrasonic[i];
            cloud_raw_ptr->push_back(ultrasonic_point);
        }
        if(right_ultrasonic[i] > 0 && right_ultrasonic[i] < 5.0){
            ultrasonic_point.y = -0.75 - right_ultrasonic[i];
            cloud_raw_ptr->push_back(ultrasonic_point);
        }
    }
//////////////////////////////遍历输入点云，初步筛选，将点划入极坐标栅格内///////////////////////////////
    for (int m=0; m<cloud_raw_ptr->size(); ++m)   
    {
        if(x_backward == 0)
        {
            if(cloud_raw_ptr->points[m].x < 0)
            {
                continue;  //跳过后部区域
            }
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
    }

/////////////////////////////////初步筛选出地面和障碍物/////////////////////////////
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
                
                if (h < threshold)  //最大高度差小于阈值的点归为地面候选点
                {
                    grid[i][j].state = GROUND;
                    for(int m=0; m<grid[i][j].grid_cloud_ptr->size(); ++m)
                    {
                        grid[i][j].avg_z += grid[i][j].grid_cloud_ptr->points[m].z;
                    }
                    grid[i][j].avg_z /= grid[i][j].grid_cloud_ptr->size();   //计算地面候选栅格的平均高度
                }
                else
                {
                    grid[i][j].state = OBSTACLE;
                }
            }
        }
    }

////////////////////////////遍历极坐标栅格地图，用梯度判断可通行区域/////////////////////////////
    for(int j=0; j<TH; ++j)    
    {
        if(x_backward == 0)
        {
            if(j>=TH/4 && j<TH/4*3)
            {
                continue;  //跳过后部区域
            }
        }
        for(int i=0; i<R; ++i)
        {
            if(i == 0)
            {
                grid[i][j].state = GROUND;
                grid[i][j].IsDrivable = true;
                grid[i][j].avg_z = ground_z;    //设为自车处的地面z坐标值
                continue;
            }
            if(grid[i][j].state == UNKNOWN){
                grid[i][j].IsDrivable = true;
                continue;
            }
            if(grid[i][j].state == GROUND)
            {
                grid[i][j].IsDrivable = true;
                for(int m=i-1; m>=0; --m)
                {
                    if(grid[m][j].state == GROUND)
                    {
                        if(fabs(grid[i][j].avg_z-grid[m][j].avg_z) / ((i-m)*grid_size_r) > max_gradient)
                        {
                            grid[i][j].state = OBSTACLE;  //计算的梯度大于梯度阈值，设为障碍物
                            grid[i][j].IsDrivable = false;
                        }
                        break;
                    }
                }
            }
            if(grid[i][j].state == OBSTACLE)  //遇到障碍物栅格，结束本条线的遍历
            {
                break;
            }
        }
    }

////////////////////////////对极坐标可通行区域进行裁剪/////////////////////////////
    int cnt = 1;
    int left_flag = 0;
    int right_flag = 0;
    int temp_th = 0;
    int left_cut = 0;
    int right_cut = 0;
    for(int i=0; i<R; ++i)
    {
        for(int j=0; j<TH; ++j)
        {
            if(i < (int)(2.0/grid_size_r) || grid[i][j].IsDrivable==false)
            {
                continue;    //半径2m内不处理，非可行驶点不处理
            }
            if(grid[i][j].IsChecked)
            {
                continue;    //跳过已检查过的栅格
            }
            for(int m=1; i*grid_size_r*sin(0.5*grid_size_th*cnt)*2 < cut_width; ++m)  //将宽度小于cut_width 的区域裁剪掉
            {//3
                if(left_flag == 0)
                {
                    temp_th = (j+m) % TH;
                    if(grid[i][temp_th].IsDrivable)
                    {
                        grid[i][temp_th].IsChecked = true;
                        cnt++;
                    }
                    else
                    {
                        left_cut = m-1;
                        left_flag = 1;
                    }
                }
                if(right_flag == 0)
                {
                    temp_th = (j-m+TH) % TH;
                    if(grid[i][temp_th].IsDrivable)
                    {
                        grid[i][temp_th].IsChecked = true;
                        cnt++;
                    }
                    else
                    {
                        right_cut = m-1;
                        right_flag = 1;
                    }
                }
                if(left_flag==1 && right_flag==1)  //宽度小于1.7m 则置为不可行驶点
                {
                    for(m=i; m<R; ++m)             //裁剪原栅格及其上部栅格
                    {
                        grid[m][j].IsDrivable = false;
                    }
                    for(m=1; m<=left_cut; ++m)     //裁剪原栅格左侧的栅格及其上部栅格
                    {
                        temp_th = (j+m) % TH;
                        for(int n=i; n<R; ++n)
                        {
                            grid[n][temp_th].IsDrivable = false;
                        }
                    }
                    for(m=1; m<=right_cut; ++m)    //裁剪原栅格右侧的栅格及其上部栅格
                    {
                        temp_th = (j-m+TH) % TH;
                        for(int n=i; n<R; ++n)
                        {
                            grid[n][temp_th].IsDrivable = false;
                        }
                    }
                    break;
                }
            }//3
            cnt = 1;
            left_flag = 0;
            right_flag = 0;
            left_cut = 0;
            right_cut = 0;
        }
    }

////////////////////////////发送最小极坐标栅格地图可行驶区域信息/////////////////////////////
    lidar_grid::DrivableMap map;
    map.header.stamp = ros::Time::now();
    for(int j=0; j<TH; ++j)    
    {
        if(j>=TH/4 && j<TH/4*3)
        {
            map.channel[j] = 0;  //跳过后部区域
            continue;  
        }
        map.channel[j] = R;
        for(int i=0; i<R; ++i)
        {
            if(grid[i][j].IsDrivable == false)
            {
                map.channel[j] = i;
                break;
            }
        }
    }
    map_pub.publish(map);

////////////////////////极坐标系栅格地图转化为直角坐标系可通行区域栅格地图////////////////////
    for(int j=-y_width; j<=y_width; ++j)
    {//1
        for(int i=-x_backward; i<=x_forward; ++i)
        {//2
            if(i==0 && j==0)
            {
                free_road.x = 0;
                free_road.y = 0;
                free_road.z = ground_z;
                grid_cell.cells.push_back(free_road);
                continue;
            }

            r = sqrt((float)(grid_size*i) * (float)(grid_size*i)
                     + (float)(grid_size*j) * (float)(grid_size*j));
            th = acos((float)(grid_size*i)/r);

            if(j < 0)
            {
                th = 2*M_PI-th;
            }

            t = (int)floor(th/grid_size_th) % TH;      //直角坐标系下的坐标转化为极坐标系的坐标，查询对应极坐标栅格的可通行性
            a = floor(r/grid_size_r);
            a = a >= R? R-1 : a;
            if(x_backward == 0)
            {
                if(t == TH/4)
                {
                    t = t-1;
                }
                else if(t == TH/4*3-1)
                {
                    t = TH/4*3;
                }
            }

            if(grid[a][t].IsDrivable)
            {
                free_road.x = grid_size*i;
                free_road.y = grid_size*j;
                free_road.z = ground_z;
                grid_cell.cells.push_back(free_road);
            }
        }//2   
    }//1

    grid_pub.publish(grid_cell);  //发布栅格地图
    clock_t end = clock();
    std_msgs::Float32 time;
    time.data = (float)(end-start)*1000/(float)CLOCKS_PER_SEC;  //程序用时 ms
    time_pub.publish(time);  //发布程序耗时
}

////////////////////////////////////////////主函数///////////////////////////////////////////////////
int main(int argc,char** argv)
{
    ros::init(argc,argv,"lidar_grid2");
    LidarCloudHandler handler;
    ros::spin();
    return 0;
}