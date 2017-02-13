#include <SPH_PID.h>


/********************************************************
 * finger test!!
 * 
 ********************************************************/
/********************* PINS ******************/
//index
#define AIN1 30
#define AIN2 31
#define PWMA 3

//middle
#define BIN1 32
#define BIN2 33
#define PWMB 4

//ring
#define AIN21 46
#define AIN22 47
#define PWM2A 5

//thumb left
#define BIN21 48
#define BIN22 49
#define PWM2B 6

//thumb top
#define AIN31 22
#define AIN32 23
#define PWM3A 7

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
#define INDEX_CURLED  10
#define MIDDLE_STRAIGHT 170
#define MIDDLE_CURLED 10
#define RING_STRAIGHT 210
#define RING_CURLED 35
#define LEFT_LEFT 700
#define LEFT_RIGHT  430
#define TOP_TOP 1000
#define TOP_MIDDLE 890
#define TOP_DOWN  770

#define INDEX_CYL 5
#define MIDDLE_CYL 15
#define RING_CYL 75
#define LEFT_CYL 600
#define TOP_CYL 870

#define INDEX_BALL 0
#define MIDDLE_BALL 21
#define RING_BALL 210
#define LEFT_BALL 600
#define TOP_BALL 870

#define indexPotPin 0
#define middlePotPin 1
#define ringPotPin 2
#define thumbLeftPotPin 3
#define thumbTopPotPin 4

char buffer[MAX_OUT_CHARS + 1];

/*****************SPH_PID(PWM,IN1,IN2);*************************/
SPH_PID indexPID(PWMA, AIN1, AIN2);
SPH_PID middlePID(PWMB, BIN1, BIN2);
SPH_PID ringPID(PWM2A, AIN21, AIN22);
SPH_PID thumbLeftPID(PWM2B, BIN21, BIN22);
//SPH_PID thumbTopPID(PWM2B, BIN21, BIN22);

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
  //Serial.println(currentPositionIndex);
  
   int outputIndex;
   int outputMiddle;
   int outputRing;
   int outputLeft;
   int outputTop;
 
   int targetIndex = INDEX_CURLED;
   int targetMiddle = MIDDLE_CURLED; 
   int targetRing = RING_CURLED;  
   int targetThumbLeft = LEFT_LEFT;
   int targetThumbTop = INDEX_CURLED;
   
   /*
   int targetIndex = INDEX_CURLED;
   int targetMiddle = MIDDLE_CURLED; 
   int targetRing = RING_CURLED;   
   int targetThumbLeft = LEFT_LEFT;
   int targetThumbTop = TOP_MIDDLE;
   */
   /*
   int targetIndex = INDEX_CYL;
   int targetMiddle = MIDDLE_CYL; 
   int targetRing = RING_CYL;   
   int targetThumbLeft = LEFT_CYL;
   int targetThumbTop = TOP_CYL;
   */
   /*
    int targetIndex = INDEX_BALL;
   int targetMiddle = MIDDLE_BALL; 
   int targetRing = RING_BALL;   
   int targetThumbLeft = LEFT_BALL;
   int targetThumbTop = TOP_BALL;
   */
   
   //float currentPositionIndexNew = map(currentPositionIndex,0,195,0,1000);
   Serial.print("Current Position INDEX = ");
   Serial.print(currentPositionIndex);
   Serial.print(" Current Position MIDDLE = ");
   Serial.print(currentPositionMiddle);
   Serial.print(" Current Position Left = ");
   Serial.println(currentPositionThumbLeft);
   
   indexPID.pid(currentPositionIndex,targetIndex,kPIndex,outputIndex);
   middlePID.pid(currentPositionMiddle,targetMiddle,kPMiddle,outputMiddle);
   ringPID.pid(currentPositionRing,targetRing,kPRing,outputRing);
   thumbLeftPID.pid(currentPositionThumbLeft,targetThumbLeft,kPThumbLeft,outputLeft);
   //thumbTopPID.pid(currentPositionThumbTop,targetThumbTop,kPThumbTop,outputTop);
   
   timeEnd = millis();
   timePassed = timeEnd - timeStart;
   //Serial.print("Time = ");
   //Serial.println(timePassed);
   }
}
