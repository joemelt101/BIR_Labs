#include <ros/ros.h>
#include "include/RoombaBot.cpp"
#define PI 3.14159

// ros::Subscriber subscriber;
// ros::Publisher publisher;
// float vel, initialX, initialY, initialOmega, currentX, currentY, currentOmega;
// bool movingFoward = false;

// void get_pose(const turtlesim::Pose::ConstPtr& pose)
// {
//     currentX = pose->x;
//     currentY = pose->y;
//     currentOmega = pose->theta;
// }

// void turn(float finalOmega)
// {
// }

// void msg_send_velocity(float xVel, float yVel, float zVel)
// {
//     geometry_msgs::Twist msgToSend;
//     geometry_msgs::Vector3 velVector, angularVector;

//     velVector.x = 1.0;
//     velVector.y = 0.0;
//     velVector.z = 0.0;

//     angularVector.x = xVel;
//     angularVector.y = yVel;
//     angularVector.z = zVel;

//     msgToSend.linear = velVector;
//     msgToSend.angular = angularVector;
// }

// void move_forward(float distance)
// {
//     if (movingFoward == false)
//     {
//         //odometer starts at zero
//         initialX = initialY = 0.0f;
//         movingFoward = true;
//     }

//     //move foward until a certain distance away
//     float currentDistanceFromOrigin;
//     currentDistanceFromOrigin = sqrtf(pow(currentX - initialX, 2) + pow(currentY - initialY, 2));

//     if (currentDistanceFromOrigin < distance)
//     {
//         // Stop the robot
//         vel = 0;
//         movingFoward = false;
//     }
// }

int main(int argc, char** argv)
{
    ////////////////////
    // Initiate the node
    ros::init(argc, argv, "square");
    ros::start();
    irobot::RoombaBot rCon;
    ROS_INFO("HELLO WORLD!");

    ros::spinOnce();
    // rCon.moveFoward(1.0);
    rCon.rotateRadiansAmount(PI / 2);

    ///////////////////////
    // Start the work cycle
    while (ros::ok())
    {
        rCon.update();

        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }

    ros::shutdown();
}