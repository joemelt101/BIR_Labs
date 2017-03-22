#include <ros/ros.h>
#include "RoombaBot.cpp"
#define PI 3.14159

void goTo(irobot::RoombaBot &rCon, float x, float y, float angle)
{
    tf::Vector3 pos_v3(x, y, 0);
    tf::Transform position;
    position.setIdentity();
    tf::Quaternion q;
    q.setRPY(0, 0, angle); // Can handle positive x and y's, and negative angles --> Fails elsewhere...
    position.setRotation(q);
    position.setOrigin(pos_v3);
    rCon.goToPosition(position);
}

int main(int argc, char** argv)
{
    ////////////////////
    // Initiate the node

    float x, y, angle, kp, kb, ka;
    x = atof(argv[1]);
    y = atof(argv[2]);
    angle = atof(argv[3]);
    kp = atof(argv[4]);
    kb = atof(argv[5]);
    ka = atof(argv[6]);

    ros::init(argc, argv, "labthree");
    ros::start();
    irobot::RoombaBot rCon;

    rCon.setGains(kp, kb, ka);

    ros::spinOnce();

    ROS_INFO("Entered: (x, y, ang) = (%f, %f, %f)", x, y, angle);
    
    goTo(rCon, x, y, angle);

    ///////////////////////
    // Start the work cycle
    while (ros::ok())
    {
        rCon.update();

        if (!rCon.actionInProgress())
        {
            //goTo(rCon, -1, -1, 0);
        }

        ros::spinOnce();
    }

    ros::shutdown();
}
