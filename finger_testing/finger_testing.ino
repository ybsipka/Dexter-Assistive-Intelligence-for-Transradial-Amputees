#include <SPH_PID.h>


/********************************************************
 * finger test!!
 * 
 ********************************************************/
/********************* PINS ******************/
#define AIN1 4
#define AIN2 5
#define PWMA 3

#define BIN1 7
#define BIN2 8
#define PWMB 6

#define AIN21 2
#define AIN22 10
#define PWM2A 9

#define BIN21 13
#define BIN22 12
#define PWM2B 11

#define MAX_OUT_CHARS 16

/* connections
 *  might change
Motor_PinA - Motor_pinB - Ground - B - A
    Red         Black             Blue(gnd) - green(pwr)      
*/    

/**************** VARIABLES *******************/
#define kPIndex 4.5
#define kPMiddle 4.5
#define kPThumbLeft 4.5
#define kPThumbTop 2
#define TARGET_POSITION_INDEX 5
#define TARGET_POSITION_MIDDLE 5
#define TARGET_POSITION_THUMBLEFT 530
#define TARGET_POSITION_THUMBTOP 800
#define indexPotPin 0
#define middlePotPin 1
#define thumbLeftPotPin 3
#define thumbTopPotPin 4

char buffer[MAX_OUT_CHARS + 1];

/*****************SPH_PID(PWM,IN1,IN2);*************************/
SPH_PID indexPID(PWMA, AIN1, AIN2);
SPH_PID middlePID(PWMB, BIN1, BIN2);
SPH_PID thumbLeftPID(PWM2A, AIN21, AIN22);
SPH_PID thumbTopPID(PWM2B, BIN21, BIN22);

unsigned long timePassed, timeStart, timeEnd;

void setup()
{
  Serial.begin(9600);
}

void loop()
{ 
  while(true){
    timeStart= millis();
   
  //get current position readings   
    int currentPositionIndex = analogRead(indexPotPin);
    int currentPositionMiddle = analogRead(middlePotPin);
    int currentPositionThumbLeft = analogRead(thumbLeftPotPin);
    int currentPositionThumbTop = analogRead(thumbTopPotPin);
    
  //Serial.print("Current Position old= ");
  //Serial.print(currentPositionIndex);
  
   int outputIndex;
   int outputMiddle;
   int outputLeft;
   int outputTop;
     
   int targetIndex = TARGET_POSITION_INDEX;
   int targetMiddle = TARGET_POSITION_MIDDLE;    
   int targetThumbLeft = TARGET_POSITION_THUMBLEFT;
   int targetThumbTop = TARGET_POSITION_THUMBTOP;
   
   //float currentPositionIndexNew = map(currentPositionIndex,0,195,0,1000);
   Serial.print("Current Position Left= ");
   Serial.print(currentPositionThumbLeft);
   Serial.print("Current Position Top= ");
   Serial.print(currentPositionThumbTop);
    indexPID.pid(currentPositionIndex,targetIndex,kPIndex,outputIndex);
    middlePID.pid(currentPositionMiddle,targetMiddle,kPMiddle,outputMiddle);
    //ringPID.pid(currentPositionRing,targetRing,kPRing,outputRing);
  thumbLeftPID.pid(currentPositionThumbLeft,targetThumbLeft,kPThumbLeft,outputLeft);
  //thumbTopPID.pid(currentPositionThumbTop,targetThumbTop,kPThumbTop,outputTop);
    timeEnd = millis();
    timePassed = timeEnd - timeStart;
    //Serial.print("Time = ");
    //Serial.println(timePassed);
    }
}
