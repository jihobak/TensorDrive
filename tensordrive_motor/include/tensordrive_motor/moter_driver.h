#ifndef TENSOR_DRIVE_MOTER_DRIVER_H_
#define TENSOR_DRIVE_MOTER_DRIVER_H_

#include <string>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <boost/optional.hpp>

#include "tensordrive_moter/moter_interface.h"

namespace moter_driver
{

class MoterDriver
{
    public:
        MoterDriver(ros::NodeHandle nh,
                    ros::NodeHandle private_nh);
    private:
        // Moter interface
        PCA9685 moter;
        
        // ROS services
        ros::Subscriber ackermann_sub_;

        // ROS callbacks
        void ackermannCmdCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& cmd);
};

} // namespace moter_driver
#endif // TENSOR_DRIVE_MOTER_DRIVER_H_
