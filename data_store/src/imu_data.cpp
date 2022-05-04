#include <iostream>
#include <ros/ros.h>
#include <string>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <cstdlib>
#include <fstream>
#include <std_msgs/Float64.h>

using namespace std;

void callback(const sensor_msgs::Imu &msg) //回调函数
{
    ofstream filename("/home/sjtu/zjd/ShiGu/src/data_store/data/imu_data.txt", ios::app);
    if (!filename)
    {
        throw "cannot open the file";
    }
    double x = double(msg.orientation.x);
    double y = double(msg.orientation.y);
    double z = double(msg.orientation.z);
    double w = double(msg.orientation.w);
    double angular_velocity_x = double(msg.angular_velocity.x);
    double angular_velocity_y = double(msg.angular_velocity.y);
    double angular_velocity_z = double(msg.angular_velocity.z);
    double linear_acceleration_x = double(msg.linear_acceleration.x);
    double linear_acceleration_y = double(msg.linear_acceleration.y);
    double linear_acceleration_z = double(msg.linear_acceleration.z);
    filename << std::to_string(double(msg.header.stamp.sec + double(msg.header.stamp.nsec) / 1000000000)) << "  ";
    filename << to_string(x) << "  " << to_string(y) << "  " << to_string(z) << "  " << to_string(w) << "  ";
    filename << to_string(angular_velocity_x) << "  " << to_string(angular_velocity_y) << "  " << to_string(angular_velocity_z) << "  ";
    filename << to_string(linear_acceleration_x) << "  " << to_string(linear_acceleration_y) << "  " << to_string(linear_acceleration_z) << endl;
    filename.close();
    // ROS_INFO("Publish IMU Info: orientation_x:%f,orientation_y:%f,orientation_z:%f,orientation_w:%f\r\n",x,y,z,w);
    // ROS_INFO("Publish IMU Info: angular_velocity_x:%f,angular_velocity_y:%f,angular_velocity_z:%f\r\n",angular_velocity_x,angular_velocity_y,angular_velocity_z);
    // ROS_INFO("Publish IMU Info: linear_acceleration_x:%f,linear_acceleration_y:%f,linear_acceleration_z:%f\r\n",linear_acceleration_x,linear_acceleration_y,linear_acceleration_z);
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_data");
    ros::NodeHandle nh;
    //每次都复写文件
    ofstream filename("/home/sjtu/zjd/ShiGu/src/data_store/data/imu_data.txt", ios::trunc);
    filename << "time"
             << "  ";
    filename << "orientation_x"
             << "  "
             << "orientation_y"
             << "  "
             << "orientation_z"
             << "  "
             << "orientation_w"
             << "  ";
    filename << "angular_velocity_x"
             << "  "
             << "angular_velocity_y"
             << "  "
             << "angular_velocity_z"
             << "  ";
    filename << "linear_acceleration_x"
             << "  "
             << "linear_acceleration_y"
             << "  "
             << "linear_acceleration_z" << endl;
    filename.close();
    ros::Subscriber data = nh.subscribe("/imu/data", 10, callback); //订阅/imu/data话题
    ros::spin();

    return 0;
}