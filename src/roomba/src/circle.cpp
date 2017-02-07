#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>

int main(int argc, char** argv)
{
    // Initiate the node
    ros::init(argc, argv, "circle");
    ros::start();

    // Now publish 
    ros::NodeHandle nh;
    ros::Publisher publisher = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", (uint32_t)100, false); // Used to send messages to the other side

    // Now send proper message
    geometry_msgs::Twist msgToSend;

    // r = v / w
    float r, v, w;
    r = 1.0;
    v = 1.0;
    w = v / r;

    geometry_msgs::Vector3 distVector, angularVector;
    distVector.x = v;
    distVector.y = 0.0;
    distVector.z = 0.0;

    angularVector.x = 0.0;
    angularVector.y = 0.0;
    angularVector.z = w;

    // v = m/s
    // r = m
    // omega = radians per second
    // v = 2pir * omega / 2pi
    // v = r*omega
    // r = v / omega

    msgToSend.linear = distVector;
    msgToSend.angular = angularVector;

    while (ros::ok())
    {
        publisher.publish(msgToSend);
        ros::Duration(1).sleep();
    }

    ros::shutdown();
}