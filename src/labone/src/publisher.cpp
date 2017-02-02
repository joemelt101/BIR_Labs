#include <ros/ros.h>
#include <std_msgs/String.h>

int main(int argc, char** argv)
{
    // Initiate the node
    ros::init(argc, argv, "publisher");
    ros::start();
    ROS_INFO_STREAM("Hello, World!");
    ros::NodeHandle nh;
    ros::Rate sleeper(1);
    ros::Publisher publisher = nh.advertise<std_msgs::String>("myfirsttopic", (uint32_t)100, false); // Used to send messages to the other side

    int count = 5;
    while (ros::ok() && count > 0)
    {
        //Publish a message to the topic
        std_msgs::String msgToSend;
        msgToSend.data = "Hello from the other side!";
        publisher.publish(msgToSend);
        sleeper.sleep();
        ros::spinOnce();

        count--;
    }

    ros::shutdown();
}