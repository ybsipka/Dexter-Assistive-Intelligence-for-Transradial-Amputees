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
#define kPForearm 2
#define kPWrist 4.5
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
  OCR2A = 250;             // Frequency = f_clock/[2*n(1+OCR2A)] ~ 500 Hz ocr2a = 250

// Setup Timer0. Set fot control frequency.
  TCCR0B |= (1 << CS02); // Set prescaler as n = 64
  TCCR0B &= ~(1 << CS01); 
  TCCR0B &= ~(1 << CS00);  

  TCCR0B |= (1 << WGM02);
  TIMSK0 |= (1 << OCIE0A); // Output compare mode 
  OCR0A = 250;             // Frequency = f_clock/[2*n(1+OCR2A)] 

  sei();
  
}

int interruptCounterTimer0 = 0;
int interruptCounterTimer2 = 0;
int controlLoopCounterIMU = 0;
int controlLoopCounterPID = 0;

void DoMeSomething() //every second
{
  //Serial.print("controlLoopCounter=");
  Serial.println(controlLoopCounterIMU);
  
  Serial.println(interruptCounterTimer0);
  Serial.println(controlLoopCounterPID);
  //Serial.print("interruptCounter=");  
  Serial.println(interruptCounterTimer2);
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
  //yAngle = euler.y();
  zAngle = euler.z();
}
void PIDCode(){
    /*Serial.print("xAngle = ");
    Serial.print(xAngle);
    Serial.print("yAngle = ");
    Serial.print(yAngle);
    Serial.print("zAngle = ");
    Serial.println(zAngle);*/
    
  //set limits to angles  
    if(xAngle > 360){xAngle = xAngle %360;}
    if(zAngle > 360){zAngle = zAngle %360;}

  //get current position readings   
    int currentPositionWrist = analogRead(wristPotPin);
    int currentPositionForearm = analogRead(forearmPotPin);

   //get angle differences
    int angleDifferenceX = xAngleOld - xAngle;
    int angleDifferenceY = yAngleOld - yAngle;
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
   
    int outputX;
    int outputZ;

    wristPID.pid(currentPositionWrist,targetX,kPWrist,outputX);
    
    forearmPID.pid(currentPositionForearm, targetZ, kPForearm, outputZ);

   
    //store old angles
    xAngleOld = xAngle; 
    yAngleOld = yAngle;
    zAngleOld = zAngle;  
     
    //delay(BNO055_SAMPLERATE_DELAY_MS); 
}

