#include <SPH_PID.h>


/********************************************************
 * finger test!!
 * 
 ********************************************************/
/********************* PINS ******************/
//index
#define AIN1 22
#define AIN2 23
#define PWMA 2

//middle
#define BIN1 24
#define BIN2 25
#define PWMB 3

#define AIN21 26
#define AIN22 27
#define PWM2A 4

#define BIN21 28
#define BIN22 29
#define PWM2B 5

#define AIN31 30
#define AIN32 31
#define PWM3A 6

#define MAX_OUT_CHARS 16

/* connections
 *  might change
Motor_PinA - Motor_pinB - Ground - B - A
    Red         Black             Blue(gnd) - green(pwr)      
*/    

/**************** VARIABLES *******************/
#define kPIndex 4.5
#define kPMiddle 4.5
#define kPRing 4.5
#define kPThumbLeft 4.5
#define kPThumbTop 4.5
#define TARGET_POSITION_INDEX 50
#define TARGET_POSITION_MIDDLE 150
#define TARGET_POSITION_RING 150
#define TARGET_POSITION_THUMBLEFT 430
#define TARGET_POSITION_THUMBTOP 850

#define INDEX_STRAIGHT 170
#define INDEX_CURLED  5
#define MIDDLE_STRAIGHT 170
#define MIDDLE_CURLED 10
#define RING_STRAIGHT 210
#define RING_CURLED 35
#define LEFT_LEFT 700
#define LEFT_RIGHT  430
#define TOP_TOP 1000
#define TOP_MIDDLE 890
#define TOP_DOWN  770

#define indexPotPin 0
#define middlePotPin 1
#define ringPotPin 2
#define thumbLeftPotPin 4
#define thumbTopPotPin 3

char buffer[MAX_OUT_CHARS + 1];

/*****************SPH_PID(PWM,IN1,IN2);*************************/
SPH_PID indexPID(PWMA, AIN1, AIN2);
SPH_PID middlePID(PWMB, BIN1, BIN2);
SPH_PID thumbLeftPID(PWM2A, AIN21, AIN22);
SPH_PID thumbTopPID(PWM2B, BIN21, BIN22);
SPH_PID ringPID(PWM3A, AIN31, AIN32);

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
    int currentPositionRing = analogRead(ringPotPin);
    int currentPositionThumbLeft = analogRead(thumbTopPotPin);
    int currentPositionThumbTop = analogRead(thumbLeftPotPin);
    
  //Serial.print("Current Position old= ");
  //Serial.print(currentPositionIndex);
  
   int outputIndex;
   int outputMiddle;
   int outputRing;
   int outputLeft;
   int outputTop;
     /*
   int targetIndex = INDEX_STRAIGHT;
   int targetMiddle = MIDDLE_STRAIGHT; 
   int targetRing = RING_STRAIGHT;   
   int targetThumbLeft = LEFT_RIGHT;
   int targetThumbTop = TOP_DOWN;
   */
   int targetIndex = INDEX_CURLED;
   int targetMiddle = MIDDLE_CURLED; 
   int targetRing = RING_CURLED;   
   int targetThumbLeft = LEFT_LEFT;
   int targetThumbTop = TOP_MIDDLE;
   
   //float currentPositionIndexNew = map(currentPositionIndex,0,195,0,1000);
   //Serial.print("Current Position INDEX= ");
   //Serial.print(currentPositionIndex);
   //Serial.print("Current Position Ring= ");
   //Serial.print(currentPositionRing);
    indexPID.pid(currentPositionIndex,targetIndex,kPIndex,outputIndex);
    middlePID.pid(currentPositionMiddle,targetMiddle,kPMiddle,outputMiddle);
    ringPID.pid(currentPositionRing,targetRing,kPRing,outputRing);
    thumbLeftPID.pid(currentPositionThumbLeft,targetThumbLeft,kPThumbLeft,outputLeft);
    thumbTopPID.pid(currentPositionThumbTop,targetThumbTop,kPThumbTop,outputTop);
    timeEnd = millis();
    timePassed = timeEnd - timeStart;
    //Serial.print("Time = ");
    //Serial.println(timePassed);
    }
}
