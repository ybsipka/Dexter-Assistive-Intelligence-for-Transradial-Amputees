#include <SPH_PID.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/********************************************************
 * Don't Spill the Cup!!
 * IMU + Motor Control 2DOF Wrist
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
  
    Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3V DC
   Connect GROUND to common ground
         
*/    
/**************** VARIABLES *******************/
#define kPForearm 1.8
#define kPWrist 6
#define TARGET_POSITION 1000
#define INITIAL_POSITION_X 500
#define BNO055_SAMPLERATE_DELAY_MS (100)
#define wristPotPin 0
#define forearmPotPin 1

char buffer[MAX_OUT_CHARS + 1];


Adafruit_BNO055 bno = Adafruit_BNO055();

/*****************SPH_PID(PWM,IN1,IN2);*************************/
SPH_PID wristPID(PWMA, AIN1, AIN2);
SPH_PID forearmPID(PWMB, BIN1, BIN2);

int xAngle,  zAngle;
int xAngleOld = 0;
int zAngleOld = 0;
unsigned long timePassed, timeStart, timeEnd;

void setup()
{
  Serial.begin(9600);
  /*while(initialPosition == 0){
    c_pos_x = analogRead(0);
    t_pos_x = INITIAL_POSITION_X;
    pid(c_pos_x,t_pos_x);
    //Serial.print("in loop");
  }*/
  
   /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);

  bno.setExtCrystalUse(true);

  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
  
   imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

/* Display calibration status for each sensor. */
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
}

void loop()
{ 
  while(true){
    timeStart= millis();
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  
  //read angles 
    xAngle = euler.x() + 180;
    zAngle = euler.z() + 180;
    
  //set limits to angles  
    if(xAngle > 360){xAngle = xAngle %360;}
    if(zAngle > 360){zAngle = zAngle %360;}

  //get current position readings   
    int currentPositionWrist = analogRead(wristPotPin);
    int currentPositionForearm = analogRead(forearmPotPin);

   //get angle differences
    int angleDifferenceX = xAngleOld - xAngle;
    int angleDifferenceZ = zAngleOld - zAngle;

   //map IMU values to potentiometer values
    int targetX = currentPositionWrist + angleDifferenceX*2.841;
    int targetZ = currentPositionForearm + angleDifferenceZ*2.841;
    if(targetX > 1023){
      targetX = targetX -1023;
    }
    if(targetZ > 1023){
      targetZ = targetZ -1023;
    }
    Serial.print("Angle diff = ");
    Serial.print(angleDifferenceX);
    Serial.print("target = ");
    Serial.print(targetX);
    Serial.print("current pot value = ");
    Serial.print(currentPositionWrist);
    
    int outputX;
    int outputZ;
    
    //pid control
    wristPID.pid(currentPositionWrist,targetX,kPWrist,outputX);
    forearmPID.pid(currentPositionForearm, targetZ, kPForearm, outputZ);

   
    //store old angles
    xAngleOld = xAngle; 
    zAngleOld = zAngle;  
     
    //delay(BNO055_SAMPLERATE_DELAY_MS); 
    timeEnd = millis();
    timePassed = timeEnd - timeStart;
    //Serial.print("Time = ");
    //Serial.println(timePassed);
    }
    
}
