#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include <SPH_PID.h>

SPH_PID::SPH_PID(int motorPWM, int motorIN1,int motorIN2)
{
  pinMode(motorIN1, OUTPUT);
  pinMode(motorIN2, OUTPUT);
  pinMode(motorPWM, OUTPUT);

  _motorIN1 = motorIN1;
  _motorIN2 = motorIN2;
  _motorPWM = motorPWM;
}

int SPH_PID::pid(int currentPosition, int targetPosition, float pConstant, int output){
    int error = targetPosition - currentPosition;

    output = pConstant * error;
    int direction; //0 for clockwise 1 for c_clockwise

    //get the output in the range of -255 to 255
    if(output > 255){ output = 255;}
    else if(output < -255){ output = -255;}
    else{output = output;}
  //  Serial.print("Error Value =");
  //  Serial.print(abs(error));
    if(abs(error) < ERROR_MARGIN){
      runMotor(0,0);
      return 0;
      //initialPosition = 1;
    }
    if(output > 0){
      direction = COUNTER_CLOCKWISE;
      runMotor(output,direction);
      return 1;
    }
    else if (output < 0 ) {
      direction = CLOCKWISE;
      runMotor(-output,direction);
      return 2;
      }
    else{
      runMotor(0,0);
      return 0;
    }

}

void SPH_PID::runMotor(int output, int direction){
  if(direction == CLOCKWISE){
    clockwise();
  }else if (direction == COUNTER_CLOCKWISE){
    c_clockwise();
  }
    analogWrite(_motorPWM,output);
    //Serial.print("Output = ");
    //Serial.println(output);
}

void SPH_PID::clockwise(){
  digitalWrite(_motorIN2, LOW);
  digitalWrite(_motorIN1, HIGH);
}

void SPH_PID::c_clockwise(){
  digitalWrite(_motorIN1, LOW);
  digitalWrite(_motorIN2, HIGH);
}
void SPH_PID::setErrorMargin(int errorMargin){
  //use to change ERROR_MARGIN in the future
}
