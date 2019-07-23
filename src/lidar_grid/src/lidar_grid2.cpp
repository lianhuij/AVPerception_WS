#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/GridCells.h>
#include <std_msgs/Float32.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <cmath>
#include <time.h>

enum RoadState
{
  unknown,
  ground,
  obstacles
};

class Grid
{
public:
    Grid() : grid_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>), IsDrivable(false), IsChecked(false), state(unknown), avg_z(0)
    {

    }
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
    ros::Publisher grid_pub;
    ros::Publisher time_pub;
    std::string fixed_frame;
    int R;                    //半径上的分割数
    int TH;                   //角度上的分割数
    float grid_size_r;        //半径上的分辨率
    float grid_size_th;       //角度上的分辨率
    float radius;             //半径
    float grid_size;          //输出栅格大小
    double threshold;         //地面点阈值
    float ground_z;           //自车位置地面z坐标值
    float max_gradient;       //最大坡度阈值
    float cut_width;          //可通行域裁剪宽度
    int y_width;              //输出栅格地图一侧宽度
    int x_forward;            //输出栅格地图前向长度
    int x_backward;           //输出栅格地图后向长度

public:
    LidarCloudHandler()
    {
        pc_sub = nh.subscribe("cali_pc", 1, &LidarCloudHandler::rasterization, this);     //接收话题：cali_pc
        grid_pub = nh.advertise<nav_msgs::GridCells>("grid_cell", 1);                     //发布话题：grid_cell
        time_pub = nh.advertise<std_msgs::Float32>("time", 1);                            //发布话题：time

        nh.getParam("/lidar_grid2/fixed_frame", fixed_frame);
        nh.getParam("/lidar_grid2/R", R);
        nh.getParam("/lidar_grid2/TH", TH);
        nh.getParam("/lidar_grid2/grid_size_r", grid_size_r);
        nh.getParam("/lidar_grid2/grid_size", grid_size);
        nh.getParam("/lidar_grid2/threshold", threshold);
        nh.getParam("/lidar_grid2/ground_z", ground_z);
        nh.getParam("/lidar_grid2/max_gradient", max_gradient);
        nh.getParam("/lidar_grid2/cut_width", cut_width);
        nh.getParam("/lidar_grid2/y_width", y_width);
        nh.getParam("/lidar_grid2/x_forward", x_forward);
        nh.getParam("/lidar_grid2/x_backward", x_backward);
        grid_size_th = 2*M_PI/TH;
        radius = R*grid_size_r;
    }

    void rasterization(const sensor_msgs::PointCloud2& input);
};

//////////////////////////激光雷达点云栅格化及地面可通行区域提取函数///////////////////////////
void LidarCloudHandler::rasterization(const sensor_msgs::PointCloud2& input)
{
    clock_t start = clock();
    int i = 0;
    int j = 0;
    int m = 0;
    int n = 0;
    nav_msgs::GridCells grid_cell;
    grid_cell.header.frame_id = fixed_frame;  //rviz中的fixed frame
    grid_cell.cell_height = grid_size;        //栅格大小
    grid_cell.cell_width = grid_size;
    geometry_msgs::Point free_road;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw_ptr  (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(input, *cloud_raw_ptr);

    Grid grid[R][TH];      //建立极坐标栅格地图grid
    float r = 0, th = 0;
    int a = 0, t = 0;

//////////////////////////////遍历输入点云，初步筛选，将点划入极坐标栅格内///////////////////////////////
    for (m=0; m<cloud_raw_ptr->size(); ++m)   
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

        if(r > radius)
        {
            continue;   //不考虑地图半径外的点    
        }

        th = acos(cloud_raw_ptr->points[m].x/r);//点对应的向量的角度（以x轴正方向为零角度）
    
        if(cloud_raw_ptr->points[m].y<0)
        {
            th = 2*M_PI-th;   //角度范围 车周0~TH度
        }                  

        t = floor(th/grid_size_th);   //坐标转换为极坐标栅格地图坐标
        a = floor(r/grid_size_r);

        if(t == TH)
        {
            t = 0;
        }

        if(a == R)
        {
            a = R-1;
        }

        grid[a][t].grid_cloud_ptr->points.push_back(cloud_raw_ptr->points[m]);  //把该点划入对应栅格内
    }

/////////////////////////////////初步筛选出地面和障碍物/////////////////////////////
    pcl::PointXYZ minpoint, maxpoint;
    double h = 0;
    
    for (i=0; i<R; ++i)      //遍历栅格地图grid
    {
        for(j=0; j<TH; ++j)
        {
            if(grid[i][j].grid_cloud_ptr->size()>0)    //如果某栅格内有数据点
            {
                pcl::getMinMax3D (*grid[i][j].grid_cloud_ptr, minpoint, maxpoint);   //这里的最大最小是值，不是对应一个点
                h = maxpoint.z - minpoint.z;                                         //该栅格内数据点的最大高度差
                
                if (h < threshold)  //最大高度差小于阈值的点归为地面候选点
                {
                    grid[i][j].state = ground;

                    for(m=0; m<grid[i][j].grid_cloud_ptr->size(); ++m)
                    {
                        grid[i][j].avg_z += grid[i][j].grid_cloud_ptr->points[m].z;
                    }

                    grid[i][j].avg_z /= m;   //计算地面候选栅格的平均高度
                }
                else
                {
                    grid[i][j].state = obstacles;
                }
            }
        }
    }

////////////////////////////遍历极坐标栅格地图，用梯度判断可通行区域/////////////////////////////
    for(j=0; j<TH; ++j)    
    {
        if(x_backward == 0)
        {
            if(j>=TH/4 && j<TH/4*3)
            {
                continue;  //跳过后部区域
            }
        }
        
        for(i=0; i<R; ++i)
        {
            if(i == 0)
            {
                grid[i][j].state = ground;
                grid[i][j].avg_z = ground_z;    //设为自车处的地面z坐标值
                continue;
            }

            if(grid[i][j].state == ground)
            {
                for(m=i-1; m>=0; --m)
                {
                    if(grid[m][j].state == ground)
                    {
                        if(fabs(grid[i][j].avg_z-grid[m][j].avg_z) / ((i-m)*grid_size_r) > max_gradient)
                        {
                            grid[i][j].state = obstacles;  //计算的梯度大于梯度阈值，设为障碍物
                        }

                        break;
                    }
                }
            }
            
            if(grid[i][j].state == obstacles)  //遇到障碍物栅格，将前面的栅格设为可通行区域
            {
                for(m=i-1; m>=0; --m)
                {
                    grid[m][j].IsDrivable = true;
                }

                break;
            }
            
            if(i == R-1)  //遍历过程没有遇到障碍物，全部设为可通行区域
            {
                for(m=i; m>=0; --m)
                {
                    grid[m][j].IsDrivable = true;
                }
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

    for(i=0; i<R; ++i)       
    {
        for(j=0; j<TH; ++j)
        {
            if(i < (int)(2/grid_size_r) || grid[i][j].IsDrivable==false)
            {
                continue;    //半径2m内不处理，非可行驶点不处理
            }

            if(grid[i][j].IsChecked)
            {
                continue;    //跳过已检查过的栅格
            }

            for(m=1; i*grid_size_r*grid_size_th*cnt<cut_width; ++m)  //将宽度小于cut_width 的区域裁剪掉
            {//3
                if(left_flag == 0)
                {
                    if(j+m >= TH)
                    {
                        temp_th = j+m-TH; 
                    }
                    else
                    {
                        temp_th = j+m;
                    }

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
                    if(j-m < 0)
                    {
                        temp_th = j-m+TH; 
                    }
                    else
                    {
                        temp_th = j-m;
                    }

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
                    for(m=i; m<R; ++m)   //裁剪原栅格及其上部栅格
                    {
                        grid[m][j].IsDrivable = false;
                    }
                    
                    for(m=1; m<=left_cut; ++m)   //裁剪原栅格左侧的栅格及其上部栅格
                    {
                        if(j+m >= TH)
                        {
                            temp_th = j+m-TH; 
                        }
                        else
                        {
                            temp_th = j+m;
                        }

                        for(n=i; n<R; ++n)
                        {
                            grid[n][temp_th].IsDrivable = false;
                        }
                    }

                    for(m=1; m<=right_cut; ++m)   //裁剪原栅格右侧的栅格及其上部栅格
                    {
                        if(j-m < 0)
                        {
                            temp_th = j-m+TH; 
                        }
                        else
                        {
                            temp_th = j-m;
                        }

                        for(n=i; n<R; ++n)
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

////////////////////////极坐标系栅格地图转化为直角坐标系可通行区域栅格地图////////////////////
    for(j=-y_width; j<=y_width; ++j)
    {//1
        for(i=-x_backward; i<=x_forward; ++i)
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

            t = floor(th/grid_size_th);      //直角坐标系下的坐标转化为极坐标系的坐标，查询对应极坐标栅格的可通行性
            a = floor(r/grid_size_r);

            if(t == TH)
            {
                t = 0;
            }
            else if(x_backward == 0)
            {
                if(t == TH/4)
                {
                    t = t-1;
                }
                else if(t == TH/4*3-1)
                {
                    t == TH/4*3;
                }
            }

            if(a == R)
            {
                a = R-1;
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