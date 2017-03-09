#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

#include "ca_msgs/WheelVelocity.h"

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
            void getPose(const nav_msgs::Odometry::ConstPtr& pose); // Callback. Sets _currentPose to the updated value.

            tf::Transform _currentPose; // The current position of the robot.
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

        _currentPose.setIdentity();
        ros::Duration(1).sleep();
    }

    bool RoombaBot::actionInProgress()
    {
        // ROS_INFO("Action in Progress = %s", (_movingFoward || _rotating) ? "True" : "False");
        return _movingFoward || _rotating || _moving;
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
        //ROS_INFO("Hello!");
        sendVelocityMessage();

        if (_movingFoward == true)
        {
            // Get the goal point
            tf::Vector3 goal_pt = _desiredPosition.getOrigin();

            // Get current point
            tf::Vector3 current_pt = _currentPose.getOrigin();

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
            float currentAngle = _currentPose.getRotation().getAngle();
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
            tf::Transform oPr = _currentPose;
            tf::Transform iTo = _oTi.inverse();
            tf::Transform gPr = _gTi * iTo * oPr;

            // Calculate the alpha, beta, and rho
            tf::Vector3 gPr_point = gPr.getOrigin(); // includes dx, dy, dz
            float dx = -gPr_point.getX();
            float dy = -gPr_point.getY();
            //ROS_INFO_THROTTLE(5, "(dx, dy) = (%f, %f)", dx, dy);
            
            if (dx == 0)
            {
                dx = 0.00001;
            }

            float theta = tf::getYaw(gPr.getRotation());
            //ROS_INFO("Theta: %f", theta);
            float beta = -atan(dy / dx);
            float alpha = -beta - theta;

            // float alpha = -theta + atan2(dy, dx);
            // float beta = -theta - alpha;

            //if (dx < 0)
            if (_reverseVelocity)
            {
                // Reverse the direction of the goal and robot
                tf::Transform rotateAroundYaw180Degrees;
                rotateAroundYaw180Degrees.setIdentity();
                tf::Quaternion q;
                q.setRPY(0, 0, PI); //180 degree rotation
                rotateAroundYaw180Degrees.setRotation(q);
                gPr = gPr * rotateAroundYaw180Degrees;

                tf::Vector3 gPr_point = gPr.getOrigin(); // includes dx, dy, dz
                dx = -gPr_point.getX();
                dy = -gPr_point.getY();
                theta = tf::getYaw(gPr.getRotation()) - PI;
                beta = -atan(dy / dx);
                alpha = -beta - theta;
                // alpha = -theta + atan2(dy, dx);
                // beta = -theta - alpha;

                // float bookAlpha = -theta + atan2(dy, dx);
                // float bookBeta = -theta - alpha;

                //ROS_INFO_THROTTLE(.1, "Pre Adjustment: (alpha, beta) = (%f, %f)", alpha, beta);
                //ROS_INFO_THROTTLE(.1, "Ours: (%f, %f) v. Book: (%f, %f)", beta, alpha, bookAlpha, bookBeta);
                
                // if (fabs(beta) > PI / 2)
                // {
                //     float factor = fabs(PI / 2 / beta);
                //     alpha *= factor;
                //     beta *= factor;
                // }

                // if (fabs(alpha) > PI / 2)
                // {
                //     float factor = fabs(PI / 2 / alpha);
                //     beta *= factor;
                //     alpha *= factor;
                // }

                ROS_INFO_THROTTLE(.1, "Post Adjustment: (alpha, beta) = (%f, %f)", alpha, beta);

                // dx *= -1;
                // dy *= -1;
                // theta = tf::getYaw(gPr.getRotation()) - PI; // Need to flip the theta too!!!
                // beta = -atan(dy / dx);
                // alpha = -beta - theta;

                // Reverse the direction of the velocity
            }

            float rho = gPr_point.length(); 

            // Check to see if close enough to the end
            // ensure BETA < Threshhold
            // Ensure length of x, y, z < threshhold

            // ROS_INFO_THROTTLE(1, "Plain: (rho, alpha, beta, theta): (%f, %f, %f, %f)", rho, alpha, beta, theta);
            // ROS_INFO_THROTTLE(1, "Fabulous: (rho, alpha, beta, theta): (%f, %f, %f, %f)", fabs(rho), fabs(alpha), fabs(beta), fabs(theta));
            // ROS_INFO_THROTTLE(1, "DEG_TO_RAD(5) = %f", DEG_TO_RAD(5));


            if (fabs(beta) < DEG_TO_RAD(3) && fabs(rho) < 0.02 && fabs(alpha) < DEG_TO_RAD(3))
            {
                // Yay!!! We made it...
                ROS_INFO("Done Moving!");
                _moving = false;
            }
            else
            {
                // _kp = 0.5;
                // _kb = -0.5;
                // _ka = 1.5;

                // Plug into the loop to determine the wheel velocities
                //ROS_INFO_THROTTLE(.1, "(rho, alpha, beta) = (%f, %f, %f)", rho, alpha, beta);

                float phi_r = 1 / WHEEL_RADIUS * (_kp*rho + _ka*alpha*CHASSIS_RAD + _kb*beta*CHASSIS_RAD);
                float phi_l = 1 / WHEEL_RADIUS * (_kp*rho - _ka*alpha*CHASSIS_RAD - _kb*beta*CHASSIS_RAD);

                // if (alpha < -PI / 2)
                // {
                //     phi_r = 1;
                //     phi_l = 0;
                // }

                // if (alpha > PI / 2)
                // {
                //     phi_r = 0;
                //     phi_l = 1;
                // }

                if (_reverseVelocity)
                {
                    float temp = -phi_r;
                    phi_r = -phi_l;
                    phi_l = temp;
                }

                //ROS_INFO_THROTTLE(.1, "(phi_l, phi_r), = (%f, %f)", phi_l, phi_r);

                //ROS_INFO_THROTTLE(1, "(phi_r, phi_l): (%f, %f)", phi_r, phi_l);
                //ROS_INFO_THROTTLE(1, "(rho, alpha, beta, theta): (%f, %f, %f, %f)", rho, alpha, beta, theta);

                // Send off wheel velocities and we're done!
                ca_msgs::WheelVelocity wheelVelocity;
                wheelVelocity.velocityRight = phi_r;
                wheelVelocity.velocityLeft = phi_l;
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
        ca_msgs::WheelVelocity wheelVelocity;

        //ROS_INFO_THROTTLE(5, "v = %f, av = %f, r = %f", _velocity, _angularVelocity, WHEEL_RADIUS);
        float w1 = (_velocity + CHASSIS_RAD * _angularVelocity) / WHEEL_RADIUS;
        float w2 = (_velocity - CHASSIS_RAD * _angularVelocity) / WHEEL_RADIUS;
        wheelVelocity.velocityRight = w1;
        wheelVelocity.velocityLeft = w2;
        //ROS_INFO_THROTTLE(5, "Velocity Left: %f, Velocity Right: %f", w1, w2);
        _wheelPublisher.publish(wheelVelocity);

        //_velocityPublisher.publish(msgToSend);
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
        _currentPose = transform;
    }

    void RoombaBot::moveFoward(float distance)
    {
        _movingFoward = true;

        tf::Transform t1;
        t1.setIdentity();
        t1.setOrigin(tf::Vector3(distance, 0.0, 0.0));
        _desiredPosition = _currentPose * t1;
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
        _desiredPosition = _currentPose * t1;

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
        _desiredPosition = _currentPose * t1;

        setAngVelocity(0.1);
    }

    void RoombaBot::goToPosition(tf::Transform iPg)
    {
        // Set Initial Reference Frame to Origin
        tf::Transform oTi(_currentPose);

        tf::Vector3 p = iPg.getOrigin();
        float x, y, z, theta;
        x = p.getX();
        y = p.getY();
        z = p.getZ();
        theta = tf::getYaw(iPg.getRotation());

        // Calculate the alpha, beta, and rho
        tf::Vector3 gPr_point = gPr.getOrigin(); // includes dx, dy, dz
        float dx = -gPr_point.getX();
        float dy = -gPr_point.getY();

        float alpha = -theta + atan2(dy, dx);

        if (fabs(alpha) > PI / 2)
        {
            // reverse direction
            _reverseVelocity = true;
        }
        
        ROS_INFO("Go to position: (%f, %f, %f, %f)", x, y, z, theta);

        _oTi = oTi;

        tf::Transform oPr = _currentPose;
        tf::Transform iTo = _oTi.inverse();
        tf::Transform gPr = _gTi * iTo * oPr;

        // Calculate the Goal Ref Frame from the Initial Frame using the goalPosition variables
        _iTg = iPg;
        _gTi = _iTg.inverse();
        _moving = true;
    }
}