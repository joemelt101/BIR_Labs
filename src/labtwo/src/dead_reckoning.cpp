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
    
    tf::Vector3 pos_v3(0, 0, 0);
    tf::Transform position;
    position.setIdentity();
    tf::Quaternion q;
    q.setRPY(0, 0, PI / 4);
    position.setRotation(q);
    position.setOrigin(pos_v3);
    rCon.goToPosition(position);

     ///////////////////////
     // Start the work cycle
     while (ros::ok())
     {
        rCon.update();
        ros::spinOnce();
    }

    ros::shutdown();
}
