/*
 * The MIT License (MIT)

Copyright (c) 2015 Jetsonhacks

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
// servoExample.cpp
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h> // for changing terminal property
#include <unistd.h> // for using const vluae, 'STDIN_FILENO'.
#include <time.h>
#include <JHPWMPCA9685.h>


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

int main() {
    
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
