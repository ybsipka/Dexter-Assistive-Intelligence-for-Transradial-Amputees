#include <SPH_PID.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include <TimerOne.h>

/********************************************************
 * Don't Spill the Cup!!
 * IMU + Motor Control 2DOF Wrist
 ********************************************************/
/********************* PINS ******************/
#define AIN1 51
#define AIN2 50
#define PWMA 12

#define BIN1 53
#define BIN2 52
#define PWMB 13

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
/*IMPORTANT *************************/
/* FOREARM = Z AXIS WRIST = Y AXIS */

/**************** VARIABLES *******************/
#define kPForearm 8  //******** HAVE TO ADJUST THE GAIN SO THAT IT MATCHES THE ACTIVATION VOLTAGE OF THE MOTOR
#define kPWrist 12    //WEIRD
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

int xAngle,  zAngle, yAngle;
int xAngleOld = 0;
int yAngleOld = 0;
int zAngleOld = 0;


unsigned long timePassed, timeStart, timeEnd;
int xAngleInit, yAngleInit, zAngleInit = 0;
void setup()
{
  Serial.begin(9600);
  /*while(initialPosition == 0){
    c_pos_x = analogRead(0);
    t_pos_x = INITIAL_POSITION_X;
    pid(c_pos_x,t_pos_x);
    //Serial.print("in loop");
  }*/
  
   // Initialise the sensor 
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
   Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
   while(1);
  }

  delay(100);

  bno.setExtCrystalUse(true);

  //Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
  
  //imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

/* Display calibration status for each sensor. */
  //uint8_t system, gyro, accel, mag = 0;
  //bno.getCalibration(&system, &gyro, &accel, &mag);

//delay(1000);

/* Interrupt Initializations */
  Timer1.initialize(1000000);                   // initialize timer1, and set a 1/2 second period
  Timer1.attachInterrupt(DoMeSomething);        // attaches callback() as a timer overflow interrupt  

  cli();
  
  // Setup Timer2. Set fot control frequency.
  TCCR2B &= ~(1 << CS22); // Set prescaler as n = 64
  TCCR2B |= (1 << CS21); 
  TCCR2B |= (1 << CS20);  

  TCCR2B |= (1 << WGM22);
  TIMSK2 |= (1 << OCIE2A); // Output compare mode 
  OCR2A = 250;             // Frequency = f_clock/[2*n(1+OCR2A)] ~ 1 kHz ocr2a = 250

// Setup Timer0. Set fot control frequency.
  TCCR0B |= (1 << CS02); // Set prescaler as n = 256
  TCCR0B &= ~(1 << CS01); 
  TCCR0B &= ~(1 << CS00);  

  TCCR0B |= (1 << WGM02);
  TIMSK0 |= (1 << OCIE0A); // Output compare mode 
  OCR0A = 125;             // Frequency = f_clock/[2*n(1+OCR2A)]  ~500 Hz ocr0a = 125 456hz imu

  sei();

imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
Serial.println("ONE SEC!");
  delay(1000);
  //read angles 
  xAngleInit = euler.x();
  yAngleInit = euler.y();
  zAngleInit = euler.z();
  Serial.print("xAngle Initial = ");
    Serial.print(xAngleInit);
    Serial.print("yAngle Init = ");
    Serial.print(yAngleInit);
    Serial.print("zAngle Init = ");
    Serial.println(zAngleInit);
  
}

int interruptCounterTimer0 = 0;
int interruptCounterTimer2 = 0;
int controlLoopCounterIMU = 0;
int controlLoopCounterPID = 0;

void DoMeSomething() //every second
{
  //Serial.print("controlLoopCounter=");
 /*Serial.println(controlLoopCounterIMU);
  
  Serial.println(interruptCounterTimer0);
  Serial.println(controlLoopCounterPID);
  //Serial.print("interruptCounter=");  
  Serial.println(interruptCounterTimer2);

  */
  /*Serial.print("xAngle = ");
  Serial.print(xAngle);
  Serial.print("yAngle = ");
  Serial.print(yAngle);
  Serial.print("zAngle = ");
  Serial.println(zAngle);*/
  
  interruptCounterTimer0 = 0;
  interruptCounterTimer2 = 0;
  controlLoopCounterIMU = -1;
  controlLoopCounterPID = -1;
  
}

// Control logic goes here. Exected at ~1.0khz.
ISR(TIMER2_COMPA_vect)
{  
  interruptCounterTimer2++;
}
// Control logic goes here. Exected at ~1.0khz.
ISR(TIMER0_COMPA_vect)
{  
  interruptCounterTimer0++;
}
void loop()
{ 
  if(controlLoopCounterIMU < interruptCounterTimer0)
  {
    IMUCode();
    controlLoopCounterIMU++;
  }
  if(controlLoopCounterPID < interruptCounterTimer2)
  {
    PIDCode();
    controlLoopCounterPID++;
  }
}

void IMUCode(){
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  
  //read angles 
  xAngle = euler.x();
  yAngle = euler.y();
  zAngle = euler.z();
    Serial.print("xAngle = ");
    Serial.print(xAngle);
    Serial.print("yAngle = ");
    Serial.print(yAngle);
    Serial.print("zAngle = ");
    Serial.println(zAngle);
}
void PIDCode(){
   
    /*Serial.print("xAngle = ");
    Serial.print(xAngle);
    Serial.print("yAngle = ");
    Serial.print(yAngle);
    Serial.print("zAngle = ");
    Serial.println(zAngle);*/
    
  //set limits to angles  
    if(yAngle > 360){yAngle = yAngle %360;}
    if(zAngle > 360){zAngle = zAngle %360;}

  //get current position readings   
    int currentPositionWrist = yAngle;
    int currentPositionForearm = zAngle;
/* don't need
   //get angle differences
    int angleDifferenceX = xAngleInit - xAngle;
    int angleDifferenceY = yAngleInit - yAngle;
    int angleDifferenceZ = zAngleInit - zAngle;
*/
   //map IMU values to potentiometer values
    //int targetX = currentPositionWrist + angleDifferenceX*2.841;
    //int targetZ = currentPositionForearm + angleDifferenceZ*2.841;
    int targetZ = zAngleInit;
    int targetY = yAngleInit;
    /*
    if(targetX > 1023){
      targetX = targetX -1023;
    }
    if(targetZ > 1023){
      targetZ = targetZ -1023;
    }
   */
    int outputY, outputZ;

    wristPID.pid(currentPositionWrist,targetY,kPWrist,outputY);
    //TESTING FOREARM
    forearmPID.pid(currentPositionForearm, targetZ, kPForearm, outputZ);

   /*dont need
    //store old angles
    xAngleOld = xAngle; 
    yAngleOld = yAngle;
    zAngleOld = zAngle;  
     */
    //delay(BNO055_SAMPLERATE_DELAY_MS); 
}

