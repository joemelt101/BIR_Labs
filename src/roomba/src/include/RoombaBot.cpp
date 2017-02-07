#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

#define PI 3.14159

namespace irobot
{
    class RoombaBot
    {
        public:
            RoombaBot(void);
            void update(void);
            void setVelocity(float velocity);
            void setAngVelocity(float angularVelocity);
            void moveFoward(float distance);
            void rotateRadiansAmount(float radianOffset);
        private:
            float _velocity;
            float _angularVelocity;
            float _originX;
            float _originY;
            float _currentX;
            float _currentY;
            float _currentAngle;
            float _desiredAngle;
            bool _movingFoward;
            float _distanceToTravel;
            bool _rotating;
            ros::Publisher _velocityPublisher;
            ros::Subscriber _odomSubscriber;
            void sendVelocityMessage();
            void getPose(const nav_msgs::Odometry::ConstPtr& pose);
    };

    RoombaBot::RoombaBot(void)
    {
        // Initiate subscribers and publishers
        this->_movingFoward = true;
        ros::NodeHandle nh;
        _velocityPublisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100, false);
        _odomSubscriber = nh.subscribe<nav_msgs::Odometry>("/odom", 100, &RoombaBot::getPose, this);
        ros::Duration(1).sleep();
    }

    void RoombaBot::setVelocity(float velocity)
    {
        ROS_INFO("Setting Velocity to: %lf", velocity);
        this->_velocity = velocity;
        this->sendVelocityMessage();
    }

    void RoombaBot::setAngVelocity(float angularVelocity)
    {
        ROS_INFO("Setting Angular Velocity to: %lf", angularVelocity);
        this->_angularVelocity = angularVelocity;
        this->sendVelocityMessage();
    }

    void RoombaBot::update(void)
    {
        // Handle update functionality here...
        if (this->_movingFoward == true)
        {
            ROS_INFO_THROTTLE(1, "X Value: %lf", _currentX);
            ROS_INFO_THROTTLE(1, "Y Value: %lf", _currentY);

            if (sqrt(pow(_currentX - _originX, 2) + pow(_currentY - _originY, 2)) >= _distanceToTravel)
            {
                ROS_INFO("Done moving!");
                // arrived at destination
                this->_movingFoward = false;
                this->setVelocity(0.0);
            }
        }

        ROS_INFO("Current Angle: %f", this->_currentAngle);
        if (this->_rotating)
        {
            // ROS_INFO_THROTTLE(1, "Omega Value: %lf", this->_currentAngle);
            float ratio = abs(this->_currentAngle / this->_desiredAngle);
            // ROS_INFO("RATIO: %f", ratio);

            if (ratio >= 0.95 && ratio <= 1.05)
            {
                ROS_INFO("Done Turning!");
                this->_rotating = false;
                this->setAngVelocity(0.0);
            }
        }
    }

    void RoombaBot::sendVelocityMessage(void)
    {
        // Now send proper message
        geometry_msgs::Twist msgToSend;

        geometry_msgs::Vector3 linearVelocityVector, angularVector;
        linearVelocityVector.x = this->_velocity;
        linearVelocityVector.y = 0.0;
        linearVelocityVector.z = 0.0;

        angularVector.x = 0.0;
        angularVector.y = 0.0;
        angularVector.z = this->_angularVelocity;

        msgToSend.linear = linearVelocityVector;
        msgToSend.angular = angularVector;
        ROS_INFO("Publishing Velocity!");
        this->_velocityPublisher.publish(msgToSend);
    }

    void RoombaBot::getPose(const nav_msgs::Odometry::ConstPtr& odom)
    {
        this->_currentX = odom->pose.pose.position.x;
        this->_currentY = odom->pose.pose.position.x;
        this->_currentAngle = odom->pose.pose.orientation.w;
    }

    void RoombaBot::moveFoward(float distance)
    {
        this->_movingFoward = true;

        // get original coordinates for later comparison
        this->_originX = this->_currentX;
        this->_originY = this->_currentY;

        this->_distanceToTravel = distance;
        
        // set velocity to .5 until within certain distance of endpoint
        this->setVelocity(0.5);
    }

    void RoombaBot::rotateRadiansAmount(float radianOffset)
    {
        this->_rotating = true;

        ROS_INFO("Offset: %f", _currentAngle);
        this->_desiredAngle = this->_currentAngle + (radianOffset / (2*PI));
        ROS_INFO("Offset: %f", _desiredAngle);
        
        if (radianOffset < 0)
        {
            this->setAngVelocity(-0.5);
        }

        if (radianOffset > 0)
        {
            this->setAngVelocity(0.5);
        }
    }
}