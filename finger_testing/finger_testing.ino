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

#define MAX_OUT_CHARS 16

/* connections
 *  might change
Motor_PinA - Motor_pinB - Ground - B - A
    Red         Black             Blue(gnd) - green(pwr)      
*/    

/**************** VARIABLES *******************/
#define kPIndex 4.5
#define kPMiddle 4.5
#define TARGET_POSITION_INDEX 20
#define TARGET_POSITION_MIDDLE 20
#define indexPotPin 0
#define middlePotPin 1

char buffer[MAX_OUT_CHARS + 1];

/*****************SPH_PID(PWM,IN1,IN2);*************************/
SPH_PID indexPID(PWMA, AIN1, AIN2);
SPH_PID middlePID(PWMB, BIN1, BIN2);

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

  Serial.print("Current Position old= ");
  Serial.print(currentPositionIndex);
  
   int outputIndex;
   int outputMiddle;
   
   int targetIndex = TARGET_POSITION_INDEX;
   int targetMiddle = TARGET_POSITION_MIDDLE;
    
   //float currentPositionIndexNew = map(currentPositionIndex,0,195,0,1000);
   //Serial.print("Current Position new= ");
   //Serial.print(currentPositionIndexNew);
    Serial.print(indexPID.pid(currentPositionIndex,targetIndex,kPIndex,outputIndex));
    Serial.print(middlePID.pid(currentPositionMiddle,targetMiddle,kPMiddle,outputMiddle));

    timeEnd = millis();
    timePassed = timeEnd - timeStart;
    //Serial.print("Time = ");
    //Serial.println(timePassed);
    }
}
