#include "motor/motor_interface.h"
#include "ros/ros.h"
#include "ackermann_msgs/AckermannDriveStamped.h"


class Motor
{
    public:
        Motor()
        {   
            steering_angle = 200;
            speed = 389;
            
            hardware = new PCA9685;
            r = hardware->openPCA9685();
            if(r <0)
            {
                std::cout << "Error: " << hardware->error << std::endl;
            }
            else
            {
                printf("PCA9685 Device Address: 0x%02X\n", hardware->kI2CAddress);
                hardware->setAllPWM(0, 0);
                hardware->reset();
                hardware->setPWMFrequency(60);
                
                sleep(1);
                hardware->setPWM(0, 0, 200);
                sleep(1);
                
                std::cout << "Finish Motor Initialization" << std::endl;
            }
        }
        ~Motor()
        {
            hardware->setAllPWM(0, 0);
            delete hardware;
        }
        
        PCA9685 *hardware;
        int r;
        int speed;
        int steering_angle;
        // ROS callbacks
        void driveCmdCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& cmd);
};

void Motor::driveCmdCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& cmd)
{
    steering_angle = (int)cmd->drive.steering_angle;
    speed = (int)cmd->drive.speed;
    //hardware->setPWM(1, 0, speed);
    //hardware->setPWM(0, 0, steering_angle);
    std::cout << "Drivecmd, speed: " << speed << "steer: " << steering_angle << std::endl;
}


int main(int argc, char **argv)
{
    Motor motor;
    ros::init(argc, argv, "drive_cmd_to_motor");
    ros::NodeHandle nh;
    
    ros::Subscriber drive_cmd_sub = nh.subscribe("drive_cmd", 100, &Motor::driveCmdCallback, &motor);
    
    while(ros::ok())
    {
        motor.hardware->setPWM(1, 0, motor.speed);
        motor.hardware->setPWM(0, 0, motor.steering_angle);
        //std::cout << "Listening>>>>>>" << std::endl;
        ros::spinOnce();
    }
    
    return 0;
}








