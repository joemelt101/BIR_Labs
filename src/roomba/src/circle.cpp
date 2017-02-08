#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include "include/RoombaBot.cpp"

int main(int argc, char** argv)
{
    // Initiate the node
    ros::init(argc, argv, "circle");
    ros::start();

    float r, v, w;
    r = 1.0;
    v = 1.0;
    w = v / r;

    irobot::RoombaBot bot;
    bot.setVelocity(v);
    bot.setAngVelocity(w);

    ros::shutdown();
}