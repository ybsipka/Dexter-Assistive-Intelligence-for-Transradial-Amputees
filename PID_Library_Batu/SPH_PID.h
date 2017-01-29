/*
  SPH_PID.h - Library for PID control loop that is used in 'Dexter'.
  Created by Batu Sipka, January 29, 2017.
*/
#ifndef SPH_PID_h
#define SPH_PID_h

#include "Arduino.h"

#define CLOCKWISE 0
#define COUNTER_CLOCKWISE 1
#define ERROR_MARGIN 10

class SPH_PID
{
public:
  SPH_PID(int motorPWM, int motorIN1,int motorIN2);
  int pid(int currentPosition, int targetPosition, float pConstant, int output);
  void clockwise();
  void c_clockwise();
  void runMotor(int output, int direction);
  void setErrorMargin(int errorMargin);
private:
  int _motorIN1;
  int _motorIN2;
  int _motorPWM;
};

#endif
