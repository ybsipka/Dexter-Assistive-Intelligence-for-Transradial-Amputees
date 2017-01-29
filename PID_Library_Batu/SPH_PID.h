/*
  SPH_PID.h - Library for PID control loop that is used in 'Dexter'.
  Created by Batu Sipka, January 29, 2017.
*/
#ifndef SPH_PID_h
#define SPH_PID_h

#include "Arduino.h"

#define CLOCKWISE 0
#define COUNTER_CLOCKWISE 1

class SPH_PID
{
public:
  SPH_PID(int motorPWM, int motorIN1,int motorIN2);
  void blink();
  /*int pid(int currentPosition, int targetPosition, float pConstant,int motorPWM, int motorIN1, int motorIN2, int output);
  void clockwise(int motorIN1, int motorIN2);
  void c_clockwise(int motorIN1, int motorIN2);
  void runMotor(int motorPWM, int motorIN1, int motorIN2, int output, int direction);*/
private:
  int _motorIN1;
  int _motorIN2;
  int _motorPWM;
};

#endif
