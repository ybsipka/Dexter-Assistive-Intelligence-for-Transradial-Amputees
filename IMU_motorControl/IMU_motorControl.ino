#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/********************************************************
 * PID Basic Example
 * Reading analog input 0 to control analog PWM output 3
 ********************************************************/
/********************* PINS ******************/
#define AIN1 4
#define AIN2 5
#define PWMA 3
#define MAX_OUT_CHARS 16

/* might change
Motor_PinA - Motor_pinB - Ground
    Red         Black       
*/    
/**************** VARIABLES *******************/
#define kP 0.3
#define ERROR_MARGIN 5
#define TARGET_POSITION 1000
#define BNO055_SAMPLERATE_DELAY_MS (100)


char buffer[MAX_OUT_CHARS + 1];

int t_pos, c_pos,error;
int pwm = 0;
int flag = 0;
int xAngle,yAngle,zAngle,xAngleInit,yAngleInit,zAngleInit,xAngleOld;
int IMU_turn;
int pot_target;

Adafruit_BNO055 bno = Adafruit_BNO055();

void setup()
{
  Serial.begin(9600);
   /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);

  /* Display the current temperature 
  int8_t temp = bno.getTemp();
  Serial.print("Current Temperature: ");
  Serial.print(temp);
  Serial.println(" C");
  Serial.println("");
*/
  bno.setExtCrystalUse(true);

  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");


  
   imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    
      /* Display the floating point data */
      xAngleInit = euler.x();
      yAngleInit = euler.y();
      zAngleInit = euler.z();
      
      Serial.print("X: ");
      Serial.print(xAngle);
      Serial.print(" Y: ");
      Serial.print(yAngle);
      Serial.print(" Z: ");
      Serial.print(zAngle);
      Serial.print("\t\t");

/* Display calibration status for each sensor. */
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  //t_pos = TARGET_POSITION;


}

void loop()
{ 
  while(true){
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  //angle starts at 0, if turn left goes 360,359,358
  //if goes right, 1,2,3,4 
  //when going left, new angle might be 350 and old might be 5, so that give 5 - 350 =  -345, but normally
  // it is 15 degrees, so if negative, add 360, gives 15 to the left
  //what if it's 50 degrees and turns left and gives 15, 50 = 15 = 35 so that's right amount.
  //turning left, if the IMU_turn is negative, add 360!???
  //same with turning right starting at 350 going to 10,350 - 10 = 340, I need to subtract that from 360, gives 20 which is right.
  //start at 40 and go to 75, 40 - 75 = -35, if add 360, gives 335 so need absolute value.
  
  //old = 5 new = 350   turn = -345 right = -15 turn backward -> abs() -360 OR (-) -360 OR %360
  //old = 150 new = 100 turn = 50   right = -50 turn backward -> abs()
  //old = 40 new = 350   turn = -310  right = 310  turn forward  -> abs()
  //old = 355 new = 35  turn = 320  right = 40  turn forward  -> -360 (-)
    xAngle = euler.x()+ 180;
    if(xAngle > 360){xAngle = xAngle %360;}
    
    c_pos = analogRead(0);
    
    IMU_turn = xAngleOld - xAngle;
    if(IMU_turn > 0){
      //counter clockwise
    }
    else{
      //clockwise
    }
    
    pot_target = c_pos + IMU_turn*2.841;
    if(pot_target > 1023){
      pot_target = pot_target -1023;
    }
    t_pos = pot_target;

    pid();
    
    sprintf(buffer, "Output= %d",pwm);
    Serial.print(buffer); 
    sprintf(buffer, "Input= %d\n",c_pos);
    Serial.print(buffer); 
    
    xAngleOld = xAngle; 
    delay(BNO055_SAMPLERATE_DELAY_MS);
    
    }
    
}

void pid(){
    error = t_pos - c_pos;
   
    pwm = kP * error;
    
    if(pwm > 255){ pwm = 255;}
    else if(pwm < -255){ pwm = -255;}
    else{pwm = pwm;}

    if(pwm>0){
      clockwise();
      analogWrite(PWMA,pwm); 
    }
    else if (pwm < 0 ) {
      c_clockwise();
      analogWrite(PWMA,-pwm);
    }
    else{
      clockwise();
      analogWrite(PWMA,0);
    }    
    if(abs(error) < ERROR_MARGIN){
      analogWrite(PWMA,0);
    }
}
void clockwise(){
  digitalWrite(AIN1, HIGH);  
  digitalWrite(AIN2, LOW);
  flag = 1;
}

void c_clockwise(){
  digitalWrite(AIN1, LOW);  
  digitalWrite(AIN2, HIGH);
  flag = 0;
}
