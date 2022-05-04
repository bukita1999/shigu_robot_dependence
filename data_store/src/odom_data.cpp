#include <iostream>
#include <ros/ros.h>
#include <string>
#include <sensor_msgs/NavSatFix.h>
#include <cstdlib>
#include <fstream>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

using namespace std;

void callback(const nav_msgs::Odometry &msg) //回调函数
{
    std::ofstream filename("/home/sjtu/zjd/ShiGu/src/data_store/data/odom_data.txt", ios::app);
    if (!filename)
    {
        throw "cannot open the file";
    }
    double pos_x = double(msg.pose.pose.position.x);
    double pos_y = double(msg.pose.pose.position.y);
    double pos_z = double(msg.pose.pose.position.z);
    double orientation_x = double(msg.pose.pose.orientation.x);
    double orientation_y = double(msg.pose.pose.orientation.y);
    double orientation_z = double(msg.pose.pose.orientation.z);
    double linear_x = double(msg.twist.twist.linear.x);
    double linear_y = double(msg.twist.twist.linear.y);
    double linear_z = double(msg.twist.twist.linear.z);
    double angular_x = double(msg.twist.twist.angular.x);
    double angular_y = double(msg.twist.twist.angular.y);
    double angular_z = double(msg.twist.twist.angular.z);
    filename << std::to_string(double(msg.header.stamp.sec + double(msg.header.stamp.nsec) / 1000000000)) << "  ";
    filename << std::to_string(pos_x) << "  " << std::to_string(pos_y) << "  " << std::to_string(pos_z) << "  ";
    filename << std::to_string(orientation_x) << "  " << std::to_string(orientation_y) << "  " << std::to_string(orientation_z) << "  ";
    filename << std::to_string(linear_x) << "  " << std::to_string(linear_y) << "  " << std::to_string(linear_z) << "  ";
    filename << std::to_string(angular_x) << "  " << std::to_string(angular_y) << "  " << std::to_string(angular_z) << endl;
    //filename.writeline();
    filename.close();
    //ROS_INFO("Publish GPS Info: latitude:%f,longitude:%f,altitude:%f\r\n",lat,lon,alt);
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom_data");
    ros::NodeHandle nh;
    //每次都复写文件
    std::ofstream filename("/home/sjtu/zjd/ShiGu/src/data_store/data/odom_data.txt", ios::trunc);
    filename << "time"
             << "  ";
    filename << "pos_x"
             << "  "
             << "pos_y"
             << "  "
             << "pos_z"
             << "  ";
    filename << "orientation_x"
             << "  "
             << "orientation_y"
             << "  "
             << "orientation_z"
             << "  ";
    filename << "linear_x"
             << "  "
             << "linear_y"
             << "  "
             << "linear_z"
             << "  ";
    filename << "angular_x"
             << "  "
             << "angular_y"
             << "  "
             << "angular_z" << endl;
    filename.close();
    ros::Subscriber data = nh.subscribe("/shigu_velocity_controller/odom", 10, callback); //订阅/shigu_velocity_controller/odom话题
    ros::spin();
    return 0;
}