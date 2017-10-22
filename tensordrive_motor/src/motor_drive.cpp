#include "tensordrive/moter_driver.h"

#include <cmath>
#include <sstream>

/* ROS */
#include <std_msgs/Float64.h>

namespace moter_driver
{

template <typename T>
inline boll getRequiredParam(const ross::NodeHandle& nh, std::string name, T& value);

int map ( int x, int in_min, int in_max, int out_min, int out_max);

MoterDriver::MoterDriver(ros::NodeHandle nh, ros::NodeHandle private_nh)
{
    // get parameters
    // code here!
    /*
     * if(!getRequiredParam(nh, "speed_to_erpm_gain", speed_to_erpm_gain_))
     *     return;
     */
    
    // subscribe to ackermann topic
    ackermann_sub_ = nh.subscribe("ackermann_cmd", 10 &MoterDriver::ackermannCmdCallback, this);
}

typedef ackermann_msgs::AckermannDriveStamped::ConstPtr AckermannMsgPtr;
void MoterDriver::ackermannCmdCallback(const AckermannMsgPtr& cmd)
{
    // calc speed
    std_msgs::Float64::Ptr rpm_msg(new std_msgs::Float64);
    rpm_msg->data = //map(); 
    // calc steering angle 





} // end namespace


// Calibrated for a Robot Geek RGS-13 Servo
// Make sure these are appropriate for the servo being used!
int servoMin = 100 ;
int servoMax = 300 ;
int servo = 200;
int servo_engine = 389;
int gear_change = 0;
int key = 0;

int getkey() {
    int character;
    struct termios orig_term_attr;
    struct termios new_term_attr;

    /* set the terminal to raw mode */
    tcgetattr(fileno(stdin), &orig_term_attr);
    memcpy(&new_term_attr, &orig_term_attr, sizeof(struct termios));
    new_term_attr.c_lflag &= ~(ECHO|ICANON);
    new_term_attr.c_cc[VTIME] = 0;
    new_term_attr.c_cc[VMIN] = 0;
    tcsetattr(fileno(stdin), TCSANOW, &new_term_attr);

    /* read a character from the stdin stream without blocking */
    /*   returns EOF (-1) if no character is available */
    character = fgetc(stdin);

    /* restore the original terminal attributes */
    tcsetattr(fileno(stdin), TCSANOW, &orig_term_attr);

    return character;
}

struct termios NonBlockingTerminalMode()
{
    struct termios org_term;
    struct termios new_term;

    tcgetattr(STDIN_FILENO, &org_term);
    new_term = org_term;
    
    new_term.c_lflag &= ~(ECHO | ICANON); // remove 'echo', 'canoical(wait until pressed Enter key')
    new_term.c_cc[VMIN] = 0;  // length of input
    new_term.c_cc[VTIME] = 0; // timeout   
   
    //setting
    tcsetattr(STDIN_FILENO, TCSANOW, &new_term);
    
    return org_term;
}


char GetNonBlockingInput()
{
    char input_key = 0;
  
    if(read(STDIN_FILENO, &input_key, 1) !=1) input_key = 0;
    else {
         if (input_key == 27) {
		char dummy;
		while(read(STDIN_FILENO, &dummy, 1) == 1);
    	}
        printf("------> %d <-------\n", input_key);
    }

    return input_key;
}


void ResetNonBlockingTerminalMode(struct termios a_org_term)
{
   //recover
   tcsetattr(STDIN_FILENO, TCSANOW, &a_org_term);
}

// Map an integer from one coordinate system to another
// This is used to map the servo values to degrees
// e.g. map(90,0,180,servoMin, servoMax)
// Maps 90 degrees to the servo value

int map ( int x, int in_min, int in_max, int out_min, int out_max) {
    int toReturn =  (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min ;
    // For debugging:
    // printf("MAPPED %d to: %d\n", x, toReturn);
    return toReturn ;
}


void driveCmdCallback(const AckermannMsgPtr msg)
{
    float speed = 0;
    float steering_angle = 0;

    speed = cmd->drive.speed;
    steering_angle = cmd->drive.steering_angle;



}


int main(int argc, char** argv) {
    
    ros::init(argc, argv, "tensordrive_motor_node");
    ros::NodeHandle nh;
    ros::Subscriber = nh.subscribe("drive_cmd", 10, driveCmdCallback);
    
    
    PCA9685 *pca9685 = new PCA9685();
    int r = pca9685->openPCA9685();
    if(r < 0){
        ROS_INFO("Error: %d\n", pca9685->error);
    }
    else{
        printf("PCA9685 Device Address: 0x%02X\n", pca9685->kI2CAddress);
        pca9685->setAllPWM(0, 0);
        pca9685->reset();
        pca9685->setPWMFrequency(60);

    }
    
    ros::spin();


    return 0;
}



    struct termios org_term = NonBlockingTerminalMode();

    PCA9685 *pca9685 = new PCA9685() ;
    int err = pca9685->openPCA9685();
    if (err < 0){
        printf("Error: %d", pca9685->error);
    } else {
        printf("PCA9685 Device Address: 0x%02X\n",pca9685->kI2CAddress) ;
        pca9685->setAllPWM(0,0) ;
        pca9685->reset() ;
        pca9685->setPWMFrequency(60) ;
        // 27 is the ESC key
        printf("Hit ESC key to exit\n");
        while(pca9685->error >= 0){

            //pca9685->setPWM(0,0,servoMin) ;
            //pca9685->setPWM(1,0,servoMin) ;
	    key = GetNonBlockingInput();
            if(key == 27) break;
            switch(key)
            {
                case 107: printf("left, servo:%d\n", servo); servo+=15; break;
                case 59: printf("right, servo:%d\n", servo); servo-=15; break;

                case 111: if(servo_engine >= 404){
                              gear_change=1;
                              break;
                          }
                          servo_engine -=1;
                          printf("go, engine:%d\n", servo_engine);
                          break; 
                case 108: if(servo_engine < 375){
                              gear_change=-1;
                              break;
                          }
                          servo_engine +=1;
                          printf("back, engine:%d\n", servo_engine);
                          break;
            }
            
           switch(gear_change)
           {
               case 1: servo_engine =375;
                       printf("Direction change move forward\n");
                       gear_change = 0;
                       break;
               case -1:
                        for(int i=390; i<500; i++)
                        {
                            pca9685->setPWM(1, 0, i);
                        }
                       
                        servo_engine=387;
                        printf("Direction change move backward\n");
                        gear_change = 0;
                        break;
           }
	   pca9685->setPWM(0, 0, servo);
           pca9685->setPWM(1, 0, servo_engine);

            //pca9685->setPWM(0,0,servoMax) ;
            //pca9685->setPWM(1,0,map(90,0,180,servoMin, servoMax)) ;
            //sleep(2) ;
        }
        pca9685->setPWM(1,0,389);
        pca9685->setPWM(0,0,map(90,0,180,servoMin, servoMax));
        sleep(1);
    }
    pca9685->closePCA9685();

    ResetNonBlockingTerminalMode(org_term);
    printf("Finish ~~~~~\n");
}
