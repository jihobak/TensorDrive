#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h> // for changing terminal property
#include <unistd.h> // for using const value, 'STDIN_FILENO'.
#include <time.h>


#include "ros/ros.h"
#include "ackermann_msgs/AckermannDriveStamped.h"


class KeyBoard
{
    public:
        KeyBoard()
        {
            key = 0;
            org_term = NonBlockingTerminalMode();
        }
        
        ~KeyBoard()
        {
            ResetNonBlockingTerminalMode(org_term);
        }
    
    int key;
    struct termios org_term;
    
    struct termios NonBlockingTerminalMode();
    void GetNonBlockingInput();
    void ResetNonBlockingTerminalMode(struct termios a_org_term);
};


struct termios KeyBoard::NonBlockingTerminalMode()
{
    struct termios org_term;
    struct termios new_term;

    // Get a original terminal status
    tcgetattr(STDIN_FILENO, &org_term);
    new_term = org_term;
    
    new_term.c_lflag &= ~(ECHO | ICANON); // remove 'echo', 'canoical(wait until pressed Enter key')
    new_term.c_cc[VMIN] = 0;  // length of input
    new_term.c_cc[VTIME] = 0; // timeout   
   
    //setting terminal status we set
    tcsetattr(STDIN_FILENO, TCSANOW, &new_term);
    
    // return for roll back to original status
    return org_term;
}


void KeyBoard::GetNonBlockingInput()
{
    char input_key = 0;
  
    // Check if user enter the keyboard.
    if(read(STDIN_FILENO, &input_key, 1) !=1) input_key = 0;
    else {
        if (input_key == 27) {
		char dummy;
		while(read(STDIN_FILENO, &dummy, 1) == 1);
    	}
        printf("------> %d <-------\n", input_key);
    }

    key = input_key;
}


void KeyBoard::ResetNonBlockingTerminalMode(struct termios a_org_term)
{
   //recover
   tcsetattr(STDIN_FILENO, TCSANOW, &a_org_term);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "teleop_publisher");
    ros::NodeHandle nh;
    
    
    ros::Publisher teleop_pub = nh.advertise<ackermann_msgs::AckermannDriveStamped>("drive_cmd", 100);
    
    ackermann_msgs::AckermannDriveStamped ackermann_status;
    KeyBoard keyboard;
    
    ackermann_status.drive.steering_angle = 0;
    ackermann_status.drive.speed = 389;
    ros::Rate r(100);
    while(ros::ok())
    {   
        keyboard.GetNonBlockingInput();
        if(keyboard.key == 27) break;
        
        switch(keyboard.key)
        {
            case 97: ackermann_status.drive.steering_angle+=0.1;
                     std::cout << "left, servo " <<  ackermann_status.drive.steering_angle << std::endl;
                     break;
            
            case 100: ackermann_status.drive.steering_angle-=0.1;
                     std::cout << "right, servo " <<  ackermann_status.drive.steering_angle << std::endl;
                     break;
                     
            case 119: ackermann_status.drive.speed-=1;
                     std::cout << "go, speed " <<  ackermann_status.drive.speed << std::endl;
                     break;
                      
            case 115: ackermann_status.drive.speed+=1;
                     std::cout << "back, speed " <<  ackermann_status.drive.speed << std::endl;
                     break;
            case 112: ackermann_status.drive.speed=389;
                     std::cout << "stop!!!! " << ackermann_status.drive.speed << std::endl;
                     break;
        }
        
       teleop_pub.publish(ackermann_status);
       r.sleep();
   }
    
    return 0;
}



