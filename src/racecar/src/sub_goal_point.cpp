//1.包含头文件
#define _pose sqrt(pow(pose_map.pose.position.x, 2) + pow(pose_map.pose.position.y, 2))

#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h" //注意: 调用 transform 必须包含该头文件
#include <nav_msgs/Odometry.h>
#include <vector>
#include <iostream>
#include <math.h>

using std::vector;

tf2_ros::Buffer buffer;                      //坐标系变换用到
geometry_msgs::PoseStamped point_map_now;    //回调目标中点坐标  frame_id:map
geometry_msgs::PoseStamped point_map_last;   //回调上次目标中点坐标  frame_id:map
geometry_msgs::PoseStamped point_map_stable; //稳定的目标中点坐标  frame_id:map
geometry_msgs::PoseStamped point_map_pub;    //发布目标中点坐标  frame_id:map
geometry_msgs::PoseStamped pose_map;         //小车位置坐标    frame_id:map

vector<geometry_msgs::PoseStamped> vec_pub; //存储所有发布目标中点的容器
int counter = 0;                            //相同目标点计数器
bool flag_point = 0;                        //找到目标点标志位
bool flag_movebase = 0;                     //完成发布一次，进入导航去点模式

bool flag_go = 0,    //记录离开原点标志位
    flag_finish = 0; //记录第一圈完成标志位

void tf_goal(const geometry_msgs::PoseStamped::ConstPtr &point_base_link);
void car_pose(const nav_msgs::Odometry::ConstPtr &odomMsg);
float dis_point(const geometry_msgs::PoseStamped &x1, const geometry_msgs::PoseStamped &x2);



int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");

    ros::init(argc, argv, "goal_pub");

    ros::NodeHandle nh;
    tf2_ros::TransformListener listener(buffer);
    ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 100, true); // 创建 目标中点发布节点

    ros::Subscriber sub_mid = nh.subscribe<geometry_msgs::PoseStamped>("/point_mid", 100, tf_goal);     // 创建 目标中点订阅节点
    ros::Subscriber sub_pose = nh.subscribe<nav_msgs::Odometry>("/odometry/filtered", 10, car_pose);   // 创建 小车坐标订阅节点

    pose_map.header.frame_id = "odom";


    ros::spinOnce();

    while (ros::ok())
    {
        ros::spinOnce();
        // ROS_WARN("sub开启成功");
        if (dis_point(pose_map, point_map_stable) > 0.5) //小车距离目标点大于1m，进行发布并进入去点模式
        {
            if (flag_point == 1 && flag_movebase == 0)
            {
                point_map_pub = point_map_stable;
                vec_pub.push_back(point_map_pub);
                // ROS_INFO("发布的目标点坐标为 map:(%.2f,%.2f)", point_map_pub.pose.position.x, point_map_pub.pose.position.y);
                flag_point = 0;
                flag_movebase = 1;
            }
        }

        if (dis_point(pose_map, point_map_pub) <= 1) //抵达目标点之前不更新目标点
        {
            flag_movebase = 0;
        }

        if (_pose > 5) //判断离开原点,离开原点超过半径为5m的圆，认为已经离开中点
        {

            flag_go = 1;
            // ROS_INFO("我已经离开原点");
        }

        if ((_pose <= 2) && (flag_go == 1)) //判断返回原点
        {
            flag_finish = 1; //第一圈完成标志位
            // ROS_INFO("我已完成第一圈");
        }
        pub.publish(point_map_pub);

        // pub.publish(point_map_stable);
        ROS_INFO("发布的目标点坐标为 map:(%.2f,%.2f)", point_map_pub.pose.position.x, point_map_pub.pose.position.y);
    }
    return 0;
}

void tf_goal(const geometry_msgs::PoseStamped::ConstPtr &point_base_link)
{
    try
    {
        point_map_now = buffer.transform(*point_base_link, "map");
        
        if (dis_point(point_map_now, point_map_last) < 0.5 && dis_point(pose_map, point_map_now) < 6) //判断条件:1.本次目标点与上次距离小于0.5m   2.本次目标点坐标与小车距离小于6m
        {
            counter++;        //判定为一次合理目标点坐标
            if (counter > 10) //连续20个目标点稳定，确认稳定的目标点坐标
            {
                point_map_stable = point_map_now;
                counter = 0;
                flag_point = 1; //置位稳定目标志位，更新发布的目标点坐标
                // ROS_INFO("目标中点坐标点相对于 map 的坐标为:(%.2f,%.2f)", point_map_stable.pose.position.x, point_map_stable.pose.position.y);
            }
        }
        else
        {
            counter = 0; //产生不合理目标点，重新计数
        }
        point_map_last = point_map_now;
    }
    catch(const std::exception& e)
    {
        // std::cerr << e.what() << '\n';
        ROS_INFO("程序异常:%s",e.what());
    }
}

void car_pose(const nav_msgs::Odometry::ConstPtr &odomMsg)
{
    try
    {
        pose_map.pose.position.x = odomMsg->pose.pose.position.x;
        pose_map.pose.position.y = odomMsg->pose.pose.position.y;
        pose_map.pose.position.z = odomMsg->pose.pose.position.z;
        pose_map.pose.orientation.x = odomMsg->pose.pose.orientation.x;
        pose_map.pose.orientation.y = odomMsg->pose.pose.orientation.y;
        pose_map.pose.orientation.z = odomMsg->pose.pose.orientation.z;
        pose_map.pose.orientation.w = odomMsg->pose.pose.orientation.w;
        pose_map = buffer.transform(pose_map, "map");
        // ROS_INFO("小车坐标点相对于 map 的坐标为:(%.2f,%.2f)", pose_map.pose.position.x, pose_map.pose.position.y);
    }
    catch(const std::exception& e)
    {
        // std::cerr << e.what() << '\n';
        ROS_INFO("程序异常:%s",e.what());
    }
}


float dis_point(const geometry_msgs::PoseStamped &x1, const geometry_msgs::PoseStamped &x2)
{
    float dis;
    dis = sqrt(pow(x1.pose.position.x - x2.pose.position.x, 2) + pow(x1.pose.position.y - x2.pose.position.y, 2));
    // ROS_INFO("小车相对于目标点的距离为:(%.2f)", dis);
    return dis;
}