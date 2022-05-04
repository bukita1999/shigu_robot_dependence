#include <iostream>
#include <ros/ros.h>
#include <string>
#include <sensor_msgs/NavSatFix.h>
#include <cstdlib>
#include <fstream>
#include <std_msgs/Float64.h>

using namespace std;

void callback(const sensor_msgs::NavSatFix &msg) //回调函数
{
    std::ofstream filename("/home/sjtu/zjd/ShiGu/src/data_store/data/gps_valid_data.txt", ios::app);
    if (!filename)
    {
        throw "cannot open the file";
    }
    double lat = double(msg.latitude);
    double lon = double(msg.longitude);
    double alt = double(msg.altitude);
    filename << std::to_string(double(msg.header.stamp.sec + double(msg.header.stamp.nsec) / 1000000000)) << "  ";
    filename << std::to_string(lat) << "  " << std::to_string(lon) << "  " << std::to_string(alt) << endl;
    // filename.writeline();
    filename.close();
    // ROS_INFO("Publish GPS Info: latitude:%f,longitude:%f,altitude:%f\r\n",lat,lon,alt);
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "gps_valid_data");
    ros::NodeHandle nh;
    //每次都复写文件
    std::ofstream filename("/home/sjtu/zjd/ShiGu/src/data_store/data/gps_valid_data.txt", ios::trunc);
    filename << "time"
             << "  ";
    filename << "latitude"
             << "  "
             << "longitude"
             << "  "
             << "altitude" << endl;
    filename.close();
    ros::Subscriber data = nh.subscribe("/navsat/fix_valid", 10, callback); //订阅/navsat/fix话题
    ros::spin();

    return 0;
}