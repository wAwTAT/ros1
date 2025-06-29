#define _pose sqrt(pow(carpose_map.pose.position.x, 2) + pow(carpose_map.pose.position.y, 2)) //小车距离起点距离

#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "geometry_msgs/Pose2D.h"
#include "sensor_msgs/LaserScan.h"
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/PoseStamped.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h" //注意: 调用 transform 必须包含该头文件
#include <vector>
#include <cmath>
#include <iostream>
#include "dynamic_reconfigure/server.h"
#include "racecar/drConfig.h"
#include <unistd.h>
#include <sys/types.h>

double angle_pd = 0.25;
//角度PD参与阈值参数，当位置误差小于该参数后进行角度PD控制
double angle_protect = 1;
//角度误差滤波参数，角度误差大于该参数将进行过滤，不作为控制参考
double line_protect = 1;
//位置误差滤波参数，位置误差大于该参数将进行过滤，不作为控制参考
double brake = 4;
//预警刹车参数，当角度误差变化大于该阈值时触发刹车

using std::vector;

double time_lest;
double time_now;

typedef struct //方向PID结构体
{
    float error_angle;
    float error_line;
    float P_angle;
    float D_angle;
    float P_line;
    float D_line;
    float last_error_angle;
    float last_error_line;
    int out_angle;
    int out_line;
    float line_D;
    float angle_D;
} pid_dir;

pid_dir PID_dir;

class my_car_control
{
private:
    bool flag_go = 0,    //记录离开原点标志位
        flag_finish = 0; //记录第一圈完成标志位
    ros::NodeHandle n_;
    ros::Subscriber point_sub;
    ros::Subscriber carpose_sub;
    ros::Publisher vel_pub;

    tf2_ros::Buffer buffer;
    tf2::Quaternion qtn;
    geometry_msgs::Twist cmd_vel;                  //小车速度对象
    geometry_msgs::PoseStamped point_baselink_pub; //发布目标中点坐标  frame_id:baselink
    geometry_msgs::PoseStamped point_map_pub;      //发布目标中点坐标  frame_id:map
    geometry_msgs::PoseStamped carpose_odom;       //小车位置坐标    frame_id:map
    geometry_msgs::PoseStamped carpose_map;        //小车位置坐标    frame_id:map
    vector<geometry_msgs::PoseStamped> vec_pub;    //存储所有发布目标中点的容器
    void Get_point(const geometry_msgs::Pose2D::ConstPtr &Msg);
    void car_pose(const nav_msgs::Odometry::ConstPtr &odomMsg);

public:
    my_car_control();
    ~my_car_control();
    float dis_point(const geometry_msgs::PoseStamped &x1, const geometry_msgs::PoseStamped &x2);
};

my_car_control::my_car_control()
{
    ROS_INFO("小车控制节点开启");

    tf2_ros::TransformListener listener(buffer);
    ros::Rate r(1);
    r.sleep();
    //PD参数
    PID_dir.P_line = 87;  //80 90  70
    PID_dir.D_line = 40;  //20 40
    PID_dir.P_angle = 20; //60 65  30
    PID_dir.D_angle = 20; //30 50  50
    /* 订阅/发布初始化 */
    cmd_vel.angular.x = 0;
    cmd_vel.angular.y = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.linear.z = 0;

    point_map_pub.pose.position.z = 0;
    point_baselink_pub.header.frame_id = "base_link";
    carpose_odom.header.frame_id = "odom";

    cmd_vel.linear.x = 2100;

    point_sub = n_.subscribe<geometry_msgs::Pose2D>("/point_mid", 10, &my_car_control::Get_point, this); //订阅目标点
    carpose_sub = n_.subscribe<nav_msgs::Odometry>("/odometry/filtered", 10, &my_car_control::car_pose, this);
    vel_pub = n_.advertise<geometry_msgs::Twist>("car/cmd_vel", 100); //发布速度、角速度
}

my_car_control::~my_car_control()
{
    cmd_vel.linear.x = 1500;
    cmd_vel.angular.z = 90;
    vel_pub.publish(cmd_vel);

    printf("小车控制节点关闭\n");
}

//目标点回调函数
void my_car_control::Get_point(const geometry_msgs::Pose2D::ConstPtr &Msg)
{
    if (!flag_finish)
    {
        // time_now = ros::Time::now().toSec();
        // ROS_INFO("执行时间间隔为: %f\n", time_now - time_lest);
        //获取数据
        PID_dir.error_angle = Msg->theta;
        PID_dir.error_line = Msg->y;
        //计算偏差变化量
        PID_dir.line_D = abs(PID_dir.error_line - PID_dir.last_error_line);
        PID_dir.angle_D = abs(PID_dir.error_angle - PID_dir.last_error_angle);
        // ROS_INFO("位置偏差为: %f\n", PID_dir.error_line);
        // ROS_INFO("角度偏差为: %f\n", PID_dir.error_angle);
        //直线PID参数

        PID_dir.out_line = PID_dir.P_line * PID_dir.error_line + PID_dir.D_line * (PID_dir.error_line - PID_dir.last_error_line);
        //位置误差小与阈值后进行角度误差控制
        if (abs(PID_dir.error_line) < angle_pd)
            PID_dir.out_angle = PID_dir.P_angle * PID_dir.error_angle + PID_dir.D_angle * (PID_dir.error_angle - PID_dir.last_error_angle);
        else
            PID_dir.out_angle = 0;
        //方向控制最终输出
        cmd_vel.angular.z = 90 + PID_dir.out_line + PID_dir.out_angle;

        //角度滤波
        if (PID_dir.error_angle > angle_protect) //位置偏左
        {
            cmd_vel.angular.z = 75;
            vel_pub.publish(cmd_vel);
        }
        else if (PID_dir.error_angle < -angle_protect) //位置偏右
        {
            cmd_vel.angular.z = 105;
            vel_pub.publish(cmd_vel);
        }
        //位置滤波
        if (abs(PID_dir.error_line) > line_protect)
        {
            cmd_vel.angular.z = 90;
            vel_pub.publish(cmd_vel);
        }
        //角度输出限幅
        if (cmd_vel.angular.z > 150)
            cmd_vel.angular.z = 150;
        else if (cmd_vel.angular.z < 30)
            cmd_vel.angular.z = 30;

        cmd_vel.linear.x = 1810 - 8 * abs(90 - cmd_vel.angular.z);
        // cmd_vel.linear.x = 1810 - 1 * abs(90 - cmd_vel.angular.z);
        //预警刹车
        if (PID_dir.angle_D > brake)
        {
            ///cmd_vel.linear.x = 1500;
            cmd_vel.linear.x =  1810;
            // vel_pub.publish(cmd_vel);
            // try
            // {
            //     point_baselink_pub.pose.position.x = Msg->x;
            //     point_baselink_pub.pose.position.y = Msg->y;
            //     point_baselink_pub.pose.position.z = 0;
            //     qtn.setRPY(0, 0, Msg->theta);
            //     point_baselink_pub.pose.orientation.x = qtn.getX();
            //     point_baselink_pub.pose.orientation.y = qtn.getY();
            //     point_baselink_pub.pose.orientation.z = qtn.getZ();
            //     point_baselink_pub.pose.orientation.w = qtn.getW();
            //     point_map_pub = buffer.transform(point_baselink_pub, "map");

            //     vec_pub.push_back(point_map_pub);
            //     ROS_INFO("目标点坐标点相对于 map 的坐标为:(%.2f,%.2f)", point_map_pub.pose.position.x, point_map_pub.pose.position.y);
            // }
            // catch (const std::exception &e)
            // {
            //     ROS_INFO("程序异常:%s", e.what());
            // }
            // ROS_INFO("触发刹车预警 角度误差变化率为 : %f", PID_dir.angle_D);
        }
        //发布
        vel_pub.publish(cmd_vel);
        // ROS_INFO("vel:%f theta: %f", cmd_vel.linear.x, cmd_vel.angular.z);
        PID_dir.last_error_line = PID_dir.error_line;
        PID_dir.last_error_angle = PID_dir.error_angle;

        // time_lest = ros::Time::now().toSec();
    }
    else
    {
        ROS_INFO("我已完成第一圈");
    }

    if (_pose > 5 && flag_go == 0) //判断离开原点,离开原点超过半径为5m的圆，认为已经离开中点
    {

        flag_go = 1;
        ROS_INFO("我已经离开原点");
    }

    if ((_pose <= 0.5) && (flag_go == 1) && flag_finish == 0) //判断返回原点
    {
        flag_finish = 1; //第一圈完成标志位
        cmd_vel.linear.x = 1500;
        cmd_vel.angular.z = 90;
        vel_pub.publish(cmd_vel);
        // system("roslaunch racecar car.launch");
        ROS_INFO("我已完成第一圈");
    }
}

void my_car_control::car_pose(const nav_msgs::Odometry::ConstPtr &odomMsg)
{
    try
    {
        carpose_odom.pose.position.x = odomMsg->pose.pose.position.x;
        carpose_odom.pose.position.y = odomMsg->pose.pose.position.y;
        carpose_odom.pose.position.z = odomMsg->pose.pose.position.z;
        carpose_odom.pose.orientation.x = odomMsg->pose.pose.orientation.x;
        carpose_odom.pose.orientation.y = odomMsg->pose.pose.orientation.y;
        carpose_odom.pose.orientation.z = odomMsg->pose.pose.orientation.z;
        carpose_odom.pose.orientation.w = odomMsg->pose.pose.orientation.w;
        carpose_map = buffer.transform(carpose_odom, "map");
        // ROS_INFO("小车坐标点相对于 map 的坐标为:(%.2f,%.2f)", carpose_map.pose.position.x, carpose_map.pose.position.y);
    }
    catch (const std::exception &e)
    {
        // std::cerr << e.what() << '\n';
        ROS_INFO("程序异常:%s", e.what());
    }
}

float my_car_control::dis_point(const geometry_msgs::PoseStamped &x1, const geometry_msgs::PoseStamped &x2)
{
    float dis;
    dis = sqrt(pow(x1.pose.position.x - x2.pose.position.x, 2) + pow(x1.pose.position.y - x2.pose.position.y, 2));
    // ROS_INFO("小车相对于目标点的距离为:(%.2f)", dis);
    return dis;
}

void cb(mycontrol::drConfig &config, uint32_t level)
{
    angle_pd = config.angle_pd;
    angle_protect = config.angle_protect;
    line_protect = config.line_protect;
    brake = config.brake;
    PID_dir.P_line = config.P_line;
    PID_dir.P_angle = config.P_angle;
    PID_dir.D_line = config.D_line;
    PID_dir.D_angle = config.D_angle;
    ROS_INFO("参数修改成功！\n");
}

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "");

    ros::init(argc, argv, "my_car_control");
    dynamic_reconfigure::Server<mycontrol::drConfig> server; //初始化动态参数服务端
    dynamic_reconfigure::Server<mycontrol::drConfig>::CallbackType cbType;
    cbType = boost::bind(&cb, _1, _2);
    server.setCallback(cbType);
    my_car_control controller;

    ros::spin();
    return 0;
}
