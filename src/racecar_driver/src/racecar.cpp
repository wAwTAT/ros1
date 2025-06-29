//
//racecar
//

#include "../include/racecar_driver.h"
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Twist.h>

class class_racecar
{
private:
    ros::NodeHandle n_;
    ros::Subscriber sub;
    void TwistCallback(const geometry_msgs::Twist& twist);
public:
    class_racecar();
    ~class_racecar();
};

class_racecar::class_racecar()
{
    sub = n_.subscribe("/car/cmd_vel", 1, &class_racecar::TwistCallback, this); 
}

class_racecar::~class_racecar()
{
    double angle;
    //ROS_INFO("x= %f", twist.linear.x);
    //ROS_INFO("z= %f", twist.angular.z);
    angle = 2500.0 - 90 * 2000.0 / 180.0;
    //ROS_INFO("angle= %d",uint16_t(angle));
    send_cmd(uint16_t(1500),uint16_t(angle));
}

void class_racecar::TwistCallback(const geometry_msgs::Twist& twist)
{
    double angle;
    //ROS_INFO("x= %f", twist.linear.x);
    //ROS_INFO("z= %f", twist.angular.z);
    angle = 2500.0 - twist.angular.z * 2000.0 / 180.0;
    //ROS_INFO("angle= %d",uint16_t(angle));
    send_cmd(uint16_t(twist.linear.x),uint16_t(angle));
}

int main(int argc, char** argv)
{
    char data[] = "/dev/car";

    setlocale(LC_ALL, "");
    art_racecar_init(38400,data);
    ros::init(argc, argv, "racecar_driver");

    class_racecar car;

    ros::spin();
    return 0;
}