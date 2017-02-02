#include <ros/ros.h>
#include <std_msgs/String.h>

void callback(const std_msgs::String& str)
{
    ROS_INFO_STREAM(str.data);
}

int main(int argc, char** argv)
{
    // Initiate the node
    ros::init(argc, argv, "subscriber");
    ros::start();
    ROS_INFO_STREAM("Subscriber running...");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("myfirsttopic", 100, callback);

    ros::spin();
}