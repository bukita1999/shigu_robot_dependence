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
#include <geometry_msgs/PoseWithCovarianceStamped.h>
using namespace std;

void callback(const geometry_msgs::PoseWithCovarianceStamped &msg) //回调函数
{
    std::ofstream filename("/home/sjtu/zjd/ShiGu/src/data_store/data/output_odom_data.txt", ios::app);
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
    filename << std::to_string(double(msg.header.stamp.sec + double(msg.header.stamp.nsec) / 1000000000)) << "  ";
    filename << std::to_string(pos_x) << "  " << std::to_string(pos_y) << "  " << std::to_string(pos_z) << "  ";
    filename << std::to_string(orientation_x) << "  " << std::to_string(orientation_y) << "  " << std::to_string(orientation_z) << endl;
    filename.close();
    //ROS_INFO("Publish GPS Info: latitude:%f,longitude:%f,altitude:%f\r\n",lat,lon,alt);
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "output_odom_data");
    ros::NodeHandle nh;
    //每次都复写文件
    std::ofstream filename("/home/sjtu/zjd/ShiGu/src/data_store/data/output_odom_data.txt", ios::trunc);
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
             << "orientation_z" << endl;
    filename.close();
    ros::Subscriber data = nh.subscribe("/robot_pose_ekf/odom_combined", 10, callback); //订阅/robot_pose_ekf/odom_combined话题
    ros::spin();
    return 0;
}