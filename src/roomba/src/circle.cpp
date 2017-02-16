#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include "include/RoombaBot.cpp"

#define PI 3.14159

int main(int argc, char** argv)
{
    // Initiate the node
    ros::init(argc, argv, "circle");
    ros::start();

    float r, v, w;
    r = 0.5;
    v = 0.15; // m/s
    w = v / r;

    irobot::RoombaBot bot;

    bot.setVelocity(v);
    bot.setAngVelocity(w);

    while (ros::ok())
    {
        bot.update();
        ros::spinOnce();
    }

    ros::shutdown();
}