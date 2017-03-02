#include <ros/ros.h>
#include "../../roomba/src/include/RoombaBot.cpp"
#define PI 3.14159

int main(int argc, char** argv)
{
    ////////////////////
    // Initiate the node

    ros::init(argc, argv, "dead_reckoning");
    ros::start();
    irobot::RoombaBot rCon;

    ros::spinOnce();

     ///////////////////////
     // Start the work cycle
     while (ros::ok())
     {
        rCon.update();
        ros::spinOnce();
    }

    ros::shutdown();
}
