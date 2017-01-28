/*
  SPH_PID.h - Library for PID control loop that is used in 'Dexter'.
  Created by Batu Sipka, January 28, 2017.
*/
#ifndef SPH_PID_h
#define SPH_PID_h

#include "Arduino.h"

class SPH_PID
{
public:
  SPH_PID(int wristMotorIN1,int wristMotorIN2,int wristMotorPWM, int forearmMotorIN1, int forearmMotorIN2, int forearmMotorPWM);
  void pid(int currentPosition, int targetPosition,int error, int pConstant,int motor, int output);
  void clockwise(int motorIN1, int motorIN2);
  void c_clockwise(int motorIN1, int motorIN2);
  int runMotor(int motorPWM, int motorIN1, int motorIN2, int output, int direction);
private:
  int _wristMotorIN1;
  int _wristMotorIN2;
  int _wristMotorPWM;
  int _forearmMotorIN1;
  int _forearmMotorIN2;
  int _forearmMotorPWM;
};

#endif
/*
private:
  int _currentPosition;
  int _targetPosition;
  int _pConstant;
  char _motor;*/
