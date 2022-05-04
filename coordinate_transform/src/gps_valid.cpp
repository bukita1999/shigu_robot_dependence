/*将/gps/odom数据转换为/gps/odom_valid，通过是否运行本节点，人为实现GPS信号的通断
订阅/gps/odom话题
发布/gps/odom_valid话题
*/

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

static ros::Publisher gps_pub;

void callback(const sensor_msgs::NavSatFixConstPtr &msg)
{
    gps_pub.publish(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gps_valid");
    ros::NodeHandle node;
    gps_pub = node.advertise<sensor_msgs::NavSatFix>("/navsat/fix_valid", 10);
    ros::Subscriber fix_sub = node.subscribe("/navsat/fix", 10, callback);
    ros::spin();
}
