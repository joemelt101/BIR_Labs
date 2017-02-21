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
            float _closestDistance; // The closest distace the robot has traveled to its destination.
            bool _movingFoward; // Whether or not the robot is moving currently turning
            bool _rotating; // Whether or not the robot is currently turning
            void sendVelocityMessage(); // Sends the velocity message to the hardware. Called repeatedly via update.
            void getPose(const nav_msgs::Odometry::ConstPtr& pose); // Callback. Sets _currentPosition to the updated value.

            tf::Transform _currentPosition; // The current position of the robot.
            tf::Transform _desiredPosition; // The desired position of the robot.

            ros::Subscriber _odomSubscriber; // The odometer subscriber. This takes values and populates local variables via callbacks.
            ros::Publisher _velocityPublisher; // The publisher for the velocity. Sends the velocity to the hardware.
    };

    RoombaBot::RoombaBot(void)
    {
        // Initiate subscribers and publishers
        _movingFoward = false;
        _rotating = false;
        ros::NodeHandle nh;
        _velocityPublisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100, false);
        _odomSubscriber = nh.subscribe<nav_msgs::Odometry>("/odom", 100, &RoombaBot::getPose, this);
        _currentPosition.setIdentity();
        ros::Duration(1).sleep();
    }

    bool RoombaBot::actionInProgress()
    {
        // ROS_INFO("Action in Progress = %s", (_movingFoward || _rotating) ? "True" : "False");
        return _movingFoward || _rotating;
    }

    void RoombaBot::setVelocity(float velocity)
    {
        // ROS_INFO("Setting Velocity to: %lf", velocity);
        _velocity = velocity;
        sendVelocityMessage();
    }

    void RoombaBot::setAngVelocity(float angularVelocity)
    {
        // ROS_INFO("Setting Angular Velocity to: %lf", angularVelocity);
        _angularVelocity = angularVelocity;
        sendVelocityMessage();
    }

    void RoombaBot::update(void)
    {
        sendVelocityMessage();

        if (_movingFoward == true)
        {
            // Get the goal point
            tf::Vector3 goal_pt = _desiredPosition.getOrigin();

            // Get current point
            tf::Vector3 current_pt = _currentPosition.getOrigin();

            // Get the difference
            tf::Vector3 diff = goal_pt - current_pt;

            float distanceToGoal = diff.length();

            if (distanceToGoal < _closestDistance)
            {
                _closestDistance = distanceToGoal;
            }

            if (distanceToGoal < 0.1 || distanceToGoal > _closestDistance * 1.10)
            {
                ROS_INFO("Done moving!");                
                _movingFoward = false;
                setVelocity(0.0);
            }
        }

        if (_rotating)
        {
            // In radians
            float currentAngle = _currentPosition.getRotation().getAngle();
            float desiredAngle = _desiredPosition.getRotation().getAngle();

            float diff = desiredAngle - currentAngle;

            if (diff < 0.0)
                diff *= -1.0;
            
            if (diff < .03 * 2 * PI) // Three percent of a full circle for the threshold
            {
                ROS_INFO("Done rotating!");
                _rotating = false;
                setAngVelocity(0.0);
            }
        }
    }

    void RoombaBot::sendVelocityMessage(void)
    {
        // Now send proper message
        geometry_msgs::Twist msgToSend;
        msgToSend.linear.x = _velocity;
        msgToSend.linear.y = 0.0;
        msgToSend.linear.z = 0.0;
        msgToSend.angular.x = 0.0;
        msgToSend.angular.y = 0.0;
        msgToSend.angular.z = _angularVelocity;
        _velocityPublisher.publish(msgToSend);
    }

    void RoombaBot::getPose(const nav_msgs::Odometry::ConstPtr& odom)
    {
        // Inertial Reference Frame Coordinates
        float currentX = odom->pose.pose.position.x;
        float currentY = odom->pose.pose.position.y;
        float currentZ = odom->pose.pose.position.z;

        // Get orientation Quaternion
        float x = odom->pose.pose.orientation.x;
        float y = odom->pose.pose.orientation.y;
        float z = odom->pose.pose.orientation.z;
        float w = odom->pose.pose.orientation.w;
        tf::Quaternion q(x, y, z, w);

        // Set the current position
        tf::Transform transform;
        transform.setIdentity();
        transform.setOrigin(tf::Vector3(currentX, currentY, currentZ));
        transform.setRotation(q);
        _currentPosition = transform;
    }

    void RoombaBot::moveFoward(float distance)
    {
        _movingFoward = true;

        tf::Transform t1;
        t1.setIdentity();
        t1.setOrigin(tf::Vector3(distance, 0.0, 0.0));
        _desiredPosition = _currentPosition * t1;
        _closestDistance = 10000; //set to a high number so it decreases...

        setVelocity(0.25);
    }

    void RoombaBot::rotateClockwise(float radianOffset)
    {
        _rotating = true;

        tf::Transform t1;
        t1.setIdentity();
        tf::Quaternion rotationOffset;
        rotationOffset.setRPY(0.0, 0.0, -radianOffset);
        t1.setRotation(rotationOffset);
        _desiredPosition = _currentPosition * t1;

        ROS_INFO("Started turning!");

        setAngVelocity(-0.1);
    }

    void RoombaBot::rotateCounterClockwise(float radianOffset)
    {
        _rotating = true;

        tf::Transform t1;
        t1.setIdentity();
        tf::Quaternion rotationOffset;
        rotationOffset.setRPY(0.0, 0.0, radianOffset);
        t1.setRotation(rotationOffset);
        _desiredPosition = _currentPosition * t1;

        ROS_INFO("Started turning!");

        setAngVelocity(0.1);
    }
}