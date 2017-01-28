#include "Arduino.h"
#include "SPH_PID.h"

SPH_PID::SPH_PID(int wristMotorIN1,int wristMotorIN2,int wristMotorPWM, int forearmMotorIN1, int forearmMotorIN2, int forearmMotorPWM)
{
  pinMode(wristMotorIN1, OUTPUT);
  pinMode(wristMotorIN2, OUTPUT);
  pinMode(wristMotorPWM, OUTPUT);
  pinMode(forearmMotorIN1, OUTPUT);
  pinMode(forearmMotorIN2, OUTPUT);
  pinMode(forearmMotorPWM, OUTPUT);

  _wristMotorIN1 = wristMotorIN1;
  _wristMotorIN2 = wristMotorIN2;
  _wristMotorPWM = wristMotorPWM;
  _forearmMotorIN1 = forearmMotorIN1;
  _forearmMotorIN2 = forearmMotorIN2;
  _forearmMotorPWM = forearmMotorPWM;

}

void pid(int currentPosition, int targetPosition,int error, int pConstant,int motorPWM, int motorIN1, int motorIN2, int output){
    error = targetPosition - currentPosition;
    output = pConstant * error;
    int direction; //0 for clockwise 1 for c_clockwise

    //get the output in the range of -255 to 255
    if(output > 255){ output = 255;}
    else if(output < -255){ output = -255;}
    else{output = output;}

    if(output > 0){
        direction = COUNTER_CLOCKWISE;
        runMotor(motorPWM,motorIN1,motorIN2,output,direction);
    }
    else if (output < 0 ) {
      direction = CLOCKWISE;
      runMotor(motorPWM,motorIN1,motorIN2,-output,direction);
      }
    else{
      runMotor(motorPWM,motorIN1,motorIN2,0,0);
    }
    if(abs(error) < ERROR_MARGIN){
      runMotor(motorPWM,motorIN1,motorIN2,0,0);
      //initialPosition = 1;
    }
}

int SPH_PID::runMotor(int motorPWM, int motorIN1, int motorIN2, int output, int direction){
  if(direction == CLOCKWISE){
    clockwise(motorIN1, motorIN2);
  }else if (direction == COUNTER_CLOCKWISE){
    c_clockwise(motorIN1, motorIN2);
  }
    analogWrite(motorPWM,output);
    return 1;
}

int SPH_PID::clockwise(int motorIN1, motorIN2){
  digitalWrite(motorIN2, LOW);
  digitalWrite(motorIN1, HIGH);
  return 1;
}

int SPH_PID::c_clockwise(int motorIN1, motorIN2){
  digitalWrite(motorIN1, LOW);
  digitalWrite(motorIN2, HIGH);
  return 1;
}
