#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
 
void Callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    double x=msg->pose.position.x;
    double y=msg->pose.position.y;
    //四元数转欧拉角
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.orientation, quat);
    double roll, pitch, yaw;//定义存储r\p\y的容器
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
    //打印输出
    std::cout<<"pos-x: "<<x<<"pos-y: "<<y<<"pos-yaw: "<<yaw<<std::endl;
}
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "navgoal_rviz");
    ros::NodeHandle nh;
 
    ros::Subscriber sub = nh.subscribe("/move_base/goal", 10, Callback);//队列长度:1000或1或其他
 
    while(ros::ok())
    {
        ros::spinOnce();
    }
 
  return 0;
}

