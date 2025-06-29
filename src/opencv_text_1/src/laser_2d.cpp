#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <vector>
#include <math.h>
#include <iostream>
#include "geometry_msgs/Pose2D.h"
#define PI 3.1415926

typedef struct //锥桶信息结构体
{
    float angle;
    float distance;
    float dk_x;
    float dk_y;
} dir_obstacle;

class class_laser_2d
{
private:
    int ind[600] = {0};         //排序后元素对应的位置
    float new_laser[600] = {0}; //拼接后的数据  1140-1440 + 0-300   前方150度的数据，逆时针

    ros::NodeHandle nh;
    geometry_msgs::Pose2D point; //目标中点坐标    frame_id:map
    ros::Subscriber sub;
    ros::Publisher pub;
    dir_obstacle ob_1, ob_2;

    void Get_goal(const sensor_msgs::LaserScan::ConstPtr &msg_p);
    void BubbleSort(float *p, int length, int *ind_diff);
    void Print(float str[], int n);
    void Print_int(int str[], int n);
public:
    class_laser_2d();
    ~class_laser_2d();
};

class_laser_2d::class_laser_2d()
{
    ROS_INFO("雷达数据分析节点开启");
    sub = nh.subscribe<sensor_msgs::LaserScan>("/scan_1", 100, &class_laser_2d::Get_goal, this);
    pub = nh.advertise<geometry_msgs::Pose2D>("/point_mid", 100, true); // 创建 目标中点发布节点
}

class_laser_2d::~class_laser_2d()
{
    printf("雷达数据分析节点关闭");
}

void class_laser_2d::Get_goal(const sensor_msgs::LaserScan::ConstPtr &msg_p)
{
    int num = 0,          //做雷达数据拼接用
        num_division = 1, //做雷达数据分割用
        f_division = 0;   //分割成功标志位
    //进行数据拼接
    for (int i = 1140; i <= 1440; i++)
    {
        new_laser[num] = msg_p->ranges[i];
        num++;
    }
    for (int i = 0; i <= 300; i++)
    {
        new_laser[num] = msg_p->ranges[i];
        num++;
    }
    // printf("转换前:\n");
    // Print(new_laser,600);
    BubbleSort(new_laser, 600, ind);
    // printf("转换后:\n");
    // Print(new_laser,600);
    // printf("对应序号为:\n");
    // Print_int(ind,600);
    while (1)
    {
        if (abs(ind[num_division + 1] - ind[num_division]) > 50)
        {
            break;
        }
        num_division++;
    }
    ob_1.angle = (ind[num_division] - 300) / 4;
    ob_1.distance = new_laser[num_division];
    // ROS_INFO("障碍物 1 的角度为：%f ,     距离为： %f\n", ob_1.angle, ob_1.distance);
    ob_2.angle = (ind[num_division + 1] - 300) / 4;
    ob_2.distance = new_laser[num_division + 1];
    // ROS_INFO("障碍物 2 的角度为：%f ,     距离为： %f\n", ob_2.angle, ob_2.distance);

    // ROS_INFO("X 为：%f    \n ", point.x);
    // ROS_INFO("Y 为：%f    \n ", point.y);
    // ROS_INFO("theta 为：%f\n", point.theta);
    ob_1.dk_x = ob_1.distance * cos(ob_1.angle * PI / 180);
    ob_1.dk_y = ob_1.distance * sin(ob_1.angle * PI / 180);

    ob_2.dk_x = ob_2.distance * cos(ob_2.angle * PI / 180);
    ob_2.dk_y = ob_2.distance * sin(ob_2.angle * PI / 180);

    if (ob_1.dk_y != ob_2.dk_y)
    {
        point.x = (ob_1.dk_x + ob_2.dk_x) / 2;
        point.y = (ob_1.dk_y + ob_2.dk_y) / 2;
        point.theta = -(ob_1.dk_x - ob_2.dk_x) / (ob_1.dk_y - ob_2.dk_y);
        pub.publish(point);
        // ROS_INFO("发布的目标点为 base_link X:%f , Y:%f",point.pose.position.x,point.pose.position.y);
    }
}

//参数说明 *p:待排序数组地址    length:数组长度     ind_diff:排序后对应序号的数组
void class_laser_2d::BubbleSort(float *p, int length, int *ind_diff)
{
    for (int m = 0; m < length; m++)
    {
        ind_diff[m] = m;
    }

    for (int i = 0; i < length; i++)
    {
        for (int j = 0; j < length - i - 1; j++)
        {
            if (p[j] > p[j + 1])
            {
                float temp = p[j];
                p[j] = p[j + 1];
                p[j + 1] = temp;

                int ind_temp = ind_diff[j];
                ind_diff[j] = ind_diff[j + 1];
                ind_diff[j + 1] = ind_temp;
            }
        }
    }
}

void class_laser_2d::Print(float str[], int n)
{
    int i = 0;
    for (i = 0; i < n; i++)
    {
        printf("%f ", str[i]);
    }
    printf("\n");
}

void class_laser_2d::Print_int(int str[], int n)
{
    int i = 0;
    for (i = 0; i < n; i++)
    {
        printf("%d ", str[i]);
    }
    printf("\n");
}

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "");

    ros::init(argc, argv, "laser_point_pub");

    class_laser_2d laser_point_pub;

    ros::spin();
    return 0;
}
