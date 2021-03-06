#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <math.h>

#include "ca_msgs/WheelVelocity.h"
#include "IMUSensor.cpp"

#define PI 3.14159

#define CHASSIS_RAD 0.1175
#define WHEEL_RADIUS 0.036
#define DEG_TO_RAD(x) (float)x / 360 * 2 * 3.14159

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
            void goToPosition(tf::Transform goalPosition);
            void setGains(float kp, float kb, float ka);
        private:
            float _velocity;
            float _angularVelocity;
            float _closestDistance; // The closest distace the robot has traveled to its destination.
            tf::Transform _oTi; // The transformation from the initial frame to the origin
            tf::Transform _iTg; // The transformation from the goal frame to the initial frame
            tf::Transform _gTi; // The transformation from the initial frame to the goal frame.
            float _kp;
            float _kb; 
            float _ka;             
            bool _movingFoward; // Whether or not the robot is moving currently turning
            bool _rotating; // Whether or not the robot is currently turning
            bool _moving; // Whether or not the robot is moving directly to a location
            bool _reverseVelocity;
            void sendVelocityMessage(); // Sends the velocity message to the hardware. Called repeatedly via update.
            void getPose(const nav_msgs::Odometry::ConstPtr& pose); // Callback. Sets _oPr to the updated value.
            IMUSensor _imuSensor;

            tf::Transform _oPr; // The current position of the robot.
            tf::Transform _desiredPosition; // The desired position of the robot.

            ros::Subscriber _odomSubscriber; // The odometer subscriber. This takes values and populates local variables via callbacks.
            ros::Publisher _velocityPublisher; // The publisher for the velocity. Sends the velocity to the hardware.
            ros::Publisher _wheelPublisher;
    };

    RoombaBot::RoombaBot(void)
    {
        // Initiate subscribers and publishers
        _movingFoward = false;
        _rotating = false;
        ros::NodeHandle nh;
        _velocityPublisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100, false);
        _wheelPublisher = nh.advertise<ca_msgs::WheelVelocity>("/ca_msgs/WheelVelocity", 100, false);
        _odomSubscriber = nh.subscribe<nav_msgs::Odometry>("/odom", 100, &RoombaBot::getPose, this);

        _kp = 0.5;
        _kb = -0.5;
        _ka = 1.5;

        _oPr.setIdentity();
        // Wait for sensors to calibrate and for subscribers to hook into publishers

        // Wait for IMU sensor to calibrate
        ROS_INFO("IMU Calibrating...");
        while (! _imuSensor.isReady() && ros::ok())
        {
            ros::spinOnce();
        }
        ROS_INFO("IMU Calibration Complete...");
    }

    bool RoombaBot::actionInProgress()
    {
        // ROS_INFO("Action in Progress = %s", (_movingFoward || _rotating) ? "True" : "False");
        return _movingFoward || _rotating || _moving;
    }

    void RoombaBot::setGains(float kp, float kb, float ka)
    {
        _kp = kp;
        _kb = kb;
        _ka = ka;
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
        // ROS_INFO_THROTTLE(.2, "Raw Yaw: %f", _imuSensor.getYaw());
        // ROS_INFO_THROTTLE(.2, "_currentPositionYaw: %f", tf::getYaw(_oPr.getRotation()));
        

        //ROS_INFO("Hello!");
        // sendVelocityMessage();

        if (_movingFoward == true)
        {
            // Get the goal point
            tf::Vector3 goal_pt = _desiredPosition.getOrigin();

            // Get current point
            tf::Vector3 current_pt = _oPr.getOrigin();

            // Get the difference
            tf::Vector3 diff = goal_pt - current_pt;

            float distanceToGoal = diff.length();

            if (distanceToGoal < _closestDistance)
            {
                _closestDistance = distanceToGoal;
            }

            if (distanceToGoal < 0.1 || distanceToGoal > _closestDistance * 1.10)
            {
                //ROS_INFO("Done moving!");                
                _movingFoward = false;
                setVelocity(0.0);
            }
        }

        if (_rotating)
        {
            // In radians
            float currentAngle = _oPr.getRotation().getAngle();
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

        if (_moving)
        {
            tf::Transform oPr = _oPr;
            tf::Transform iTo = _oTi.inverse();
            tf::Transform gPr = _gTi * iTo * oPr;

            float theta = tf::getYaw(gPr.getRotation());

            /////////////////////////////////////
            // Calculate the alpha, beta, and rho

            tf::Vector3 gPr_point = gPr.getOrigin(); // includes dx, dy, dz
            float dx = -gPr_point.getX();
            float dy = -gPr_point.getY();
            
            if (dx == 0)
            {
                dx = 0.00001;
            }

            // float beta = -atan(dy / dx);
            // float alpha = -beta - theta;
            float alpha = -theta + atan2(dy, dx);
            float rho = gPr_point.length(); 

            if (_reverseVelocity)
            {
                if (alpha < 0)
                {
                    alpha += PI;
                }
                else
                {
                    alpha -= PI;
                }
            }

            float beta = -theta - alpha;

            // ROS_INFO_THROTTLE(1, "Plain: (rho, alpha, beta, theta): (%f, %f, %f, %f)", rho, alpha, beta, theta);

            /////////////////////////////////
            // See if close enough to the end

            if (fabs(beta) < DEG_TO_RAD(3) && fabs(rho) < 0.02 && fabs(alpha) < DEG_TO_RAD(3))
            {
                // Yay!!! We made it...
                ROS_INFO("Done Moving!");
                _moving = false;
            }
            else
            {
                ////////////////////
                // Keep progressing!

                // Plug into the loop to determine the wheel velocities

                //ROS_INFO_THROTTLE(.1, "(rho, alpha, beta, theta): (%f, %f, %f, %f)", rho, alpha, beta, theta);
                float phi_r = 1 / WHEEL_RADIUS * (_kp*rho + _ka*alpha*CHASSIS_RAD + _kb*beta*CHASSIS_RAD);
                float phi_l = 1 / WHEEL_RADIUS * (_kp*rho - _ka*alpha*CHASSIS_RAD - _kb*beta*CHASSIS_RAD);

                //ROS_INFO_THROTTLE(.1, "(phi_r, phi_l) = (%f, %f)", phi_r, phi_l);

                if (_reverseVelocity)
                {
                    // Have a backwards perspective
                    // Need to flip the wheel velocities and reverse them to compensate
                    float temp = -phi_r;
                    phi_r = -phi_l;
                    phi_l = temp;
                }

                // Send off wheel velocities and we're done!
                ca_msgs::WheelVelocity wheelVelocity;
                wheelVelocity.velocityRight = (double)phi_r;
                wheelVelocity.velocityLeft = (double)phi_l;
                _wheelPublisher.publish(wheelVelocity);
            }
        }
    }

    void RoombaBot::sendVelocityMessage(void)
    {
        // Now send proper message
        // geometry_msgs::Twist msgToSend;
        // msgToSend.linear.x = _velocity;
        // msgToSend.linear.y = 0.0;
        // msgToSend.linear.z = 0.0;
        // msgToSend.angular.x = 0.0;
        // msgToSend.angular.y = 0.0;
        // msgToSend.angular.z = _angularVelocity;

        // Do my own conversion to wheel angles
        // ca_msgs::WheelVelocity wheelVelocity;

        // //ROS_INFO_THROTTLE(5, "v = %f, av = %f, r = %f", _velocity, _angularVelocity, WHEEL_RADIUS);
        // float w1 = (_velocity + CHASSIS_RAD * _angularVelocity) / WHEEL_RADIUS;
        // float w2 = (_velocity - CHASSIS_RAD * _angularVelocity) / WHEEL_RADIUS;
        // wheelVelocity.velocityRight = w1;
        // wheelVelocity.velocityLeft = w2;
        // //ROS_INFO_THROTTLE(5, "Velocity Left: %f, Velocity Right: %f", w1, w2);
        // _wheelPublisher.publish(wheelVelocity);

        //_velocityPublisher.publish(msgToSend);
    }

    void RoombaBot::getPose(const nav_msgs::Odometry::ConstPtr& odom)
    {
        // Inertial Reference Frame Coordinates
        float currentX = odom->pose.pose.position.x;
        float currentY = odom->pose.pose.position.y;
        float currentZ = odom->pose.pose.position.z;

        // Get orientation Quaternion
        // float x = odom->pose.pose.orientation.x;
        // float y = odom->pose.pose.orientation.y;
        // float z = odom->pose.pose.orientation.z;
        // float w = odom->pose.pose.orientation.w;
        // tf::Quaternion q(x, y, z, w);

        //ROS_INFO_THROTTLE(.1, "RoombaBot: Ready Yaw = %f", _imuSensor.getYaw());
        tf::Quaternion q;
        q.setRPY(0.0, 0.0, _imuSensor.getYaw());
        tf::Vector3 newOrigin(currentX, currentY, currentZ);

        if (_oPr.getOrigin() == newOrigin)
        {
            // The robot's wheels aren't moving
            // Make sure the sensor is locked to avoid any issues
            //ROS_INFO_THROTTLE(1, "Sensor Locked!");
            _imuSensor.lockSensor();
        }
        else
        {
            //ROS_INFO_THROTTLE(1, "Sensor Unlocked!");
            _imuSensor.unlockSensor();
        }

        // Set the current position
        _oPr.setOrigin(newOrigin);
        _oPr.setRotation(q);
    }

    void RoombaBot::moveFoward(float distance)
    {
        _movingFoward = true;

        tf::Transform t1;
        t1.setIdentity();
        t1.setOrigin(tf::Vector3(distance, 0.0, 0.0));
        _desiredPosition = _oPr * t1;
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
        _desiredPosition = _oPr * t1;

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
        _desiredPosition = _oPr * t1;

        setAngVelocity(0.1);
    }

    void RoombaBot::goToPosition(tf::Transform iPg)
    {
        // Set Initial Reference Frame to Origin
        tf::Transform oTi(_oPr);
        _oTi = oTi;

        // Calculate the Goal Ref Frame from the Initial Frame using the goalPosition variables
        _iTg = iPg;
        _gTi = _iTg.inverse();

        // Calculate the position of the robot with respect to the goal
        tf::Transform oPr = _oPr;
        tf::Transform iTo = _oTi.inverse();
        tf::Transform gPr = _gTi * iTo * oPr;

        // Determine whether you move forward or behind
        tf::Vector3 gPr_point = gPr.getOrigin(); // includes dx, dy, dz
        float dx = -gPr_point.getX();

        if (dx < 0)
        {
            // reverse direction
            _reverseVelocity = true;
        }

        _moving = true;
    }
}