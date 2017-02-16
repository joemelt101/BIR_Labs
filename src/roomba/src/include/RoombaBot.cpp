#include <ros/ros.h>
#include <tf/transform_datatypes.h>
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
            void rotateCounterClockwise(float radians);
            void rotateClockwise(float radians);
            bool actionInProgress(void);
        private:
            float _velocity;
            float _angularVelocity;
            float _originX;
            float _originY;
            float _currentX;
            float _currentY;
            float _currentAngleX;
            float _currentAngleY;
            float _currentAngleZ;
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
        _movingFoward = false;
        _rotating = false;
        ros::NodeHandle nh;
        _velocityPublisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100, false);
        _odomSubscriber = nh.subscribe<nav_msgs::Odometry>("/odom", 100, &RoombaBot::getPose, this);
        ros::Duration(1).sleep();
    }

    bool RoombaBot::actionInProgress()
    {
        ROS_INFO("Action in Progress = %s", (_movingFoward || _rotating) ? "True" : "False");
        return _movingFoward || _rotating;
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
        sendVelocityMessage();

        // Handle update functionality here...
        if (this->_movingFoward == true)
        {
            // ROS_INFO_THROTTLE(1, "X Value: %lf", _currentX);
            // ROS_INFO_THROTTLE(1, "Y Value: %lf", _currentY);

            ROS_INFO("OX: %f and OY: %f", _originX, _originY);
            ROS_INFO("X: %f and Y: %f", _currentX, _currentY);

            float currentDistanceFromStart = sqrtf(pow(_currentX - _originX, 2) + pow(_currentY - _originY, 2));
            ROS_INFO("Distance: %f", currentDistanceFromStart);
            if (currentDistanceFromStart >= _distanceToTravel)
            {
                ROS_INFO("Done moving!");
                // arrived at destination
                this->_movingFoward = false;
                this->setVelocity(0.0);
            }
        }


        // ROS_INFO("_desiredAngle: %f, _currentAngleZ: %f", _desiredAngle, _currentAngleZ);
        
        if (this->_rotating)
        {
            float ratio = _currentAngleZ / _desiredAngle;
            
            if (ratio < 0)
                ratio *= -1;

            // ROS_INFO("Ratio: %f", ratio);


            if (ratio >= 0.95 && ratio <= 1.05)
            {
                this->_rotating = false;
                // ROS_INFO("Done Moving!");
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
        // Inertial Reference Frame Coordinates
        this->_currentX = odom->pose.pose.position.x;
        this->_currentY = odom->pose.pose.position.y;

        // Inertial Reference Frame Orientation
        float x = odom->pose.pose.orientation.x;
        float y = odom->pose.pose.orientation.y;
        float z = odom->pose.pose.orientation.z;
        float w = odom->pose.pose.orientation.w;
        tf::Quaternion q(x, y, z, w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        // ROS_INFO_THROTTLE(0.5, "(Roll, Pitch, Yaw) = (%f,%f,%f)", roll, pitch, yaw);

        this->_currentAngleX = roll + PI;
        this->_currentAngleY = pitch + PI;
        this->_currentAngleZ = yaw + PI;
        // ROS_INFO_THROTTLE(0.5, "(xAng, yAng, zAng) = (%f,%f,%f)", _currentAngleX, _currentAngleY, _currentAngleZ);
    }
    

    void RoombaBot::moveFoward(float distance)
    {
        this->_movingFoward = true;

        // get original coordinates for later comparison
        this->_originX = this->_currentX;
        this->_originY = this->_currentY;

        ROS_INFO("OriginX: %f and OriginY: %f", _originX, _originY);

        this->_distanceToTravel = distance;
        
        // set velocity to .5 until within certain distance of endpoint
        this->setVelocity(0.1);
    }

    void RoombaBot::rotateClockwise(float radianOffset)
    {
        _desiredAngle = _currentAngleZ - radianOffset;

        if (_desiredAngle > 2*PI)
            _desiredAngle -= 2*PI;
        else if (_desiredAngle < 0.0)
            _desiredAngle += 2*PI;

        setAngVelocity(-0.1);
        _rotating = true;
    }

    void RoombaBot::rotateCounterClockwise(float radianOffset)
    {
        _desiredAngle = _currentAngleZ + radianOffset;

        if (_desiredAngle > 2*PI)
            _desiredAngle -= 2*PI;
        else if (_desiredAngle < 0.0)
            _desiredAngle += 2*PI;

        setAngVelocity(0.1);
        _rotating = true;
    }
}