#include <ros/ros.h>
#include <math.h>
#include <std_msgs/Float64.h>

#define PI 3.14159
#define MAX_ACCEPTABLE_DIFFERENCE 0.02 / 360.0 * 2.0 * 3.14
#define NUM_TO_COLLECT_BEFORE_TEST 20

namespace irobot
{
    class IMUSensor
    {
        public:
            IMUSensor(void);
            bool isReady();
            void zeroOut();
            float getYaw();
            void lockSensor();
            void unlockSensor();
        private:
            void readYawFromIMU(const std_msgs::Float64::ConstPtr& yaw); // Callback. Sets _currentPose yaw.
            ros::Subscriber _yawSubscriber; // The yaw subscriber (from imu).
            float _zeroValue;
            float _currentValue;
            int _numValues;
            float _runningTotal;
            bool _ready;
            bool _locked;
    };

    IMUSensor::IMUSensor(void)
    {
        ros::NodeHandle nh;
        _yawSubscriber = nh.subscribe<std_msgs::Float64>("/imu", 100, &IMUSensor::readYawFromIMU, this);
        _zeroValue = 0.0;
        _currentValue = 0.0;
        _numValues = 0;
        _ready = false;
        _locked = false;
    }

    bool IMUSensor::isReady()
    {
        return _ready;
    }

    // Belongs to [0, 2*PI]
    float IMUSensor::getYaw()
    {
        return _currentValue - _zeroValue;
    }

    void IMUSensor::readYawFromIMU(const std_msgs::Float64::ConstPtr& yaw)
    {
        // Orientation Quaternion
        //tf::Quaternion q;

        float lastValue = _currentValue;

        _currentValue = -(yaw->data / 360 * 2 * PI);

        if (_locked)
        {
            float delta = _currentValue - lastValue;
            _zeroValue += delta; //Adjust zero value so that the reading doesn't change at all...
        }

        if (_ready == false)
        {
            _numValues++;
            _runningTotal += _currentValue;
            
            if (_numValues == NUM_TO_COLLECT_BEFORE_TEST)
            {
                float ave = _runningTotal / NUM_TO_COLLECT_BEFORE_TEST;

                //ROS_INFO("%f < %f?", fabs(ave - _currentValue), MAX_ACCEPTABLE_DIFFERENCE);

                if (fabs(ave - _currentValue) < MAX_ACCEPTABLE_DIFFERENCE)
                {
                    // Ready to go!
                    _ready = true;
                    _zeroValue = _currentValue;
                }

                _numValues = 0;
                _runningTotal = 0.0;
            }
        }

        // q.setRPY(0.0, 0.0, radians);
        // ROS_INFO_THROTTLE(.1, "Raw Yaw: %f", yaw->data);
        // ROS_INFO_THROTTLE(.1, "Raw Yaw (Radians): %f", radians);
        // _currentPose.setRotation(q);
    }

    void IMUSensor::lockSensor()
    {
        _locked = true;
    }

    void IMUSensor::unlockSensor()
    {
        _locked = false;
    }
}