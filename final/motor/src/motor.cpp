#include "motor/motor_interface.h"
#include "ros/ros.h"
#include "ackermann_msgs/AckermannDriveStamped.h"

#define _USE_MATH_DEFINES //*
#include <math.h>         //* for use value of pi in c++

#include <algorithm>      //* for use min, max function


/*** Global Variables ***/
/*
 * servo motor
 * motor angle pi/6(0.5) ~ -pi/6(-0.5) [+30 ~ -30]
 * pwm signal 300 ~ 0 ~ 100
 *
*/
float servo_min = M_PI/4;
float servo_max = -M_PI/4;
int pwm_servo_min = 330;

int pwm_servo_max = 130;

template <typename T>
static inline int map (T x, T in_min, T in_max, int out_min, int out_max) {
    T value_clipped = std::min(in_min, x); // x must be smaller than 'in_min'
    value_clipped = std::max(in_max, value_clipped); // must be bigeer than 'in_max'
     
    int toReturn =  (int)((value_clipped - in_min) * (out_max - out_min) / (in_max - in_min) + out_min) ;
    // For debugging:
    std::cout << "VALUE: " << x << " clipped to " << value_clipped << "<<< " << std::endl;
    std::cout << "MAPPED " << x << " to: " << toReturn << std::endl;
    return toReturn;
}


class Motor
{
    public:
        Motor()
        {   
            steering_angle = 0;
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
        float steering_angle;
        // ROS callbacks
        void driveCmdCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& cmd);
};

void Motor::driveCmdCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& cmd)
{   //static inline int map (T x, T in_min, T in_max, int out_min, int out_max)
    int i=0;
    steering_angle = cmd->drive.steering_angle;
    if(speed < 390 && ((int)cmd->drive.speed > 400))
    {
        hardware->setPWM(1, 0, 389);
        for( i=0; i<200; i++)
        {   std::cout << "back<><><><><><><><><><><><><><><><><><><><><><>" << std::endl;
            std::cout << speed << "  <><><><><><><><><><><><><><><><><><><><><><>" << std::endl;
            //hardware->setPWM(1, 0, 402);
            hardware->setPWM(1, 0, 389);
            
            //hardware->setPWM(1, 0, 404);
        }
        
        for(i=0; i<200; i++)
        {   std::cout << "back<><><><><><><><><><><><><><><><><><><><><><>" << std::endl;
            std::cout << speed << "  <><><><><><><><><><><><><><><><><><><><><><>" << std::endl;
            //hardware->setPWM(1, 0, 402);
            hardware->setPWM(1, 0, 400);
            
            //hardware->setPWM(1, 0, 404);
        }
         for( i=0; i<200; i++)
        {   std::cout << "back<><><><><><><><><><><><><><><><><><><><><><>" << std::endl;
            std::cout << speed << "  <><><><><><><><><><><><><><><><><><><><><><>" << std::endl;
            //hardware->setPWM(1, 0, 402);
            hardware->setPWM(1, 0, 389);
            
            //hardware->setPWM(1, 0, 404);
        }
        for(i=0; i<200; i++)
        {   std::cout << "back<><><><><><><><><><><><><><><><><><><><><><>" << std::endl;
            std::cout << speed << "  <><><><><><><><><><><><><><><><><><><><><><>" << std::endl;
            //hardware->setPWM(1, 0, 402);
            hardware->setPWM(1, 0, 401);
            
            //hardware->setPWM(1, 0, 404);
        }
        
        for( i=0; i<200; i++)
        {   std::cout << "back<><><><><><><><><><><><><><><><><><><><><><>" << std::endl;
            std::cout << speed << "  <><><><><><><><><><><><><><><><><><><><><><>" << std::endl;
            //hardware->setPWM(1, 0, 402);
            hardware->setPWM(1, 0, 389);
            
            //hardware->setPWM(1, 0, 404);
        }
        
        for(i=0; i<200; i++)
        {   std::cout << "back<><><><><><><><><><><><><><><><><><><><><><>" << std::endl;
            std::cout << speed << "  <><><><><><><><><><><><><><><><><><><><><><>" << std::endl;
            //hardware->setPWM(1, 0, 402);
            hardware->setPWM(1, 0, 402);
            
            //hardware->setPWM(1, 0, 404);
        
        }
        /*
        for(int i=0; i<50; i++)
        {
        hardware->setPWM(1, 0, 402);
        hardware->setPWM(1, 0, 403);
        hardware->setPWM(1, 0, 404);
        }
         
        for(int i=0; i<50; i++)
        {   std::cout << "back---------------------------------------------" << std::endl;
            hardware->setPWM(1, 0, 404);
            hardware->setPWM(1, 0, 405);
            //hardware->setPWM(1, 0, 406);
            //hardware->setPWM(1, 0, 407);
        }
       
        for(int i=390; i<500; i++)
        {
            hardware->setPWM(1, 0, i);
        }
        for(int i=0; i<50; i++)
        {   std::cout << "back<><><><><><><><><><><><><><><><><><><><><><>" << std::endl;
            hardware->setPWM(1, 0, 404);
            hardware->setPWM(1, 0, 405);
            hardware->setPWM(1, 0, 406);
            hardware->setPWM(1, 0, 407);
        }*/
    }
    
    speed = (int)cmd->drive.speed;
    hardware->setPWM(1, 0, speed); 
    //hardware->setPWM(1, 0, 373);
    //hardware->setPWM(0, 0, steering_angle);
    hardware->setPWM(0, 0, map(steering_angle, servo_min, servo_max, pwm_servo_min, pwm_servo_max));
    std::cout << "Drivecmd, speed: " << speed << " steer: " << steering_angle << std::endl;
}


int main(int argc, char **argv)
{
    Motor motor;
    ros::init(argc, argv, "drive_cmd_to_motor");
    ros::NodeHandle nh;
    
    ros::Subscriber drive_cmd_sub = nh.subscribe("drive_cmd", 5, &Motor::driveCmdCallback, &motor);
    
    /*
    ros::Rate r(100);
    while(ros::ok())
    {
        motor.hardware->setPWM(1, 0, motor.speed);
        motor.hardware->setPWM(0, 0, motor.steering_angle);
        //std::cout << "Listening>>>>>>" << std::endl;
        ros::spinOnce();
        r.sleep();
    }
    */
    ros::spin();
    return 0;
}








