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
#define AIN1 7
#define AIN2 8
#define PWMA 6

#define BIN1 4
#define BIN2 5
#define PWMB 3

#define AIN1 7
#define AIN2 8
#define PWMA 6

#define MAX_OUT_CHARS 16

/* might change
Motor_PinA - Motor_pinB - Ground - B - A
    Red         Black             Blue(gnd) - green(pwr)

         
*/    
/**************** VARIABLES *******************/
#define kPForearm 0.3
#define kPWrist 0.3
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
    int targetX = map(currentPositionWrist, 0, 360, 1023, 0);
    int targetZ = map(currentPositionForearm, 0, 360, 1023, 0);
        
    int outputX;
    int outputZ;
    
    //pid control
    wristPID.pid(currentPositionWrist,targetX,kPWrist,outputX);
    forearmPID.pid(currentPositionForearm, targetZ, kPForearm, outputZ);
    
    //store old angles
    xAngleOld = xAngle; 
    zAngleOld = zAngle;  
     
    //delay(BNO055_SAMPLERATE_DELAY_MS); 
    }
    
}


/*pot_target = c_pos_x + IMU_turn*2.841;
    pot_target_z = c_pos_z + IMU_turn_z*2.841;

    if(pot_target > 1023){
      pot_target = pot_target -1023;
    }
    if(pot_target_z > 1023){
      pot_target_z = pot_target_z -1023;
    }*/
