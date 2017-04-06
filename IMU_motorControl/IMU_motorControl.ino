/********************************************************

    Dexter: An Assistive Intelligence for Transradial Amputees
       - Don't Spill The Cup!
       
 This software is for 2 degree of freedom wrist and forearm control. With the instructions below,
 Dexter can hold a cup and it will compansate in the Z and X axis of movement from the 
 attached arm. 

 The axis of movement are determined by the IMU's orientation;
  *Holding Dexter in front of you* 

  * ↷ is determined as +Z.
  * → is determined as +Y.
  * ↑  is determined as +X.
 
 An Adafruit 9 DOF IMU was used for orientation and the SPH_PID library 
 for controlling a 6V high power Pololu 1000:1 Micrometal Gear Motor (1) and 
 a Pololu 25D mm High Power 6V 172:1 Metal Gear motor (2). Motor (1) is located
 at the wrist with a gear reduction of 1 to 2 to give the motion in Y-axis, and 
 the motor (2) is located at the forearm to give the motion in Z-Axis. The motors
 are run off Adafruit TB6612 1.2A DC/Stepper Motor Driver Breakout Board and the 
 motor driver is connected to an Arduino Mega 2560 for control. The motor driver
 is capable of driving 2 DC motors.
 
 This software runs on two timer interrupts, Timer0 and Timer2. Both timers are
 configured at low level by sending bits to certain registers. Timer0 is used to 
 control the sensor feedback loop which runs at 500 Hz, and Timer2 is used for 
 the organization of the control loop which runs at 1 kHz frequency. 

 Connections for Adafruit IMU for Arduino Mega 2560
     ================================
   Connect SCL to analog 21
   Connect SDA to analog 20
   Connect VDD to 3.3V DC | 5V DC
   Connect GROUND to common ground

 Connection for DC Motor for Arduino Mega 2560 with Adafruit TB6612 1.2A DC/Stepper Motor Driver Breakout Board
     ================================
 Wrist Motor 
     =======
   Connect AIN1 to digital 51
   Connect AIN2 to digital 50
   Connect PWMA to analog 12
   
 Forearm motor 
     ======
   Connect BIN1 to digital 53
   Connect BIN2 to digital 52
   Connect PWMB to analog 13
  
 Written by Batu Sipka. Contributors -> Gabrielle O'Dell.
 
  * This software has been written for the Major Qualifying Project at Worcester Polytechnic Institute.
  * Dexter : An Assistive Intelligence for Transradial Amputees.
  * https://github.com/ybsipka/Dexter-Assistive-Intelligence-for-Transradial-Amputees
  
 ********************************************************/


/******** INCLUDES ******************/
#include <SPH_PID.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <TimerOne.h>

/********************* PINS ******************/
#define AIN1 51
#define AIN2 50
#define PWMA 12

#define BIN1 53
#define BIN2 52
#define PWMB 13
    
/**************** CONSTANT VARIABLES *******************/
#define kPForearm 8  //******** HAVE TO ADJUST THE GAIN SO THAT IT MATCHES THE ACTIVATION VOLTAGE OF THE MOTOR
#define kPWrist 12    //WEIRD
#define BNO055_SAMPLERATE_DELAY_MS (100)

// Instantiate the IMU
Adafruit_BNO055 bno = Adafruit_BNO055();

// Instantiate PId library - SPH_PID(PWM,IN1,IN2) ----
SPH_PID wristPID(PWMA, AIN1, AIN2);
SPH_PID forearmPID(PWMB, BIN1, BIN2);

// Global Variables
int xAngle,  zAngle, yAngle, xAngleInit, yAngleInit, zAngleInit;
int interruptCounterTimer0 = 0;
int interruptCounterTimer2 = 0;
int controlLoopCounterIMU = 0;
int controlLoopCounterPID = 0;

/**************** SETUP ****************************/
void setup()
{
  
  // Start the serial monitor 
  Serial.begin(9600);
  
  // Check if the IMU is connected
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
   Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
   while(1);
  }
  
  // Set IMU crystal for external use
  bno.setExtCrystalUse(true);

/* Interrupt Initializations */
  Timer1.initialize(1000000);                   // initialize timer1, and set a 1/2 second period
  Timer1.attachInterrupt(DoMeSomething);        // attaches callback() as a timer overflow interrupt  

  cli();
  
  // Setup Timer2. Set for control loop frequency.
  timer2_setup();
  
  // Setup Timer0. Set fot sensor feedback frequency.
  timer0_setup();
  
  sei();
  // Read initial angles and keep them
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  xAngleInit = euler.x();
  yAngleInit = euler.y();
  zAngleInit = euler.z();

  // Print the initial angles
  Serial.print("xAngle Initial = ");
  Serial.print(xAngleInit);
  Serial.print("yAngle Init = ");
  Serial.print(yAngleInit);
  Serial.print("zAngle Init = ");
  Serial.println(zAngleInit);
}


void DoMeSomething() // Fire every second
{
  /*Serial.print("controlLoopCounter=");
  Serial.println(controlLoopCounterIMU);
  Serial.println(interruptCounterTimer0);
  Serial.println(controlLoopCounterPID);
  Serial.print("interruptCounter=");  
  Serial.println(interruptCounterTimer2);
  */  
  // Reset values
  interruptCounterTimer0 = 0;
  interruptCounterTimer2 = 0;
  controlLoopCounterIMU = -1;
  controlLoopCounterPID = -1;
}

// Control logic goes here. Expected at ~1.0 kHz.
ISR(TIMER2_COMPA_vect)
{  
  interruptCounterTimer2++;
}

// Sensor feedback loop goes here. Expected at ~500 Hz.
ISR(TIMER0_COMPA_vect)
{  
  interruptCounterTimer0++;
}

// Main loop
void loop()
{ 
  // When Timer0 fires
  if(controlLoopCounterIMU < interruptCounterTimer0)
  {
    IMUCode();
    controlLoopCounterIMU++;
  }

  //When Timer2 fires
  if(controlLoopCounterPID < interruptCounterTimer2)
  {
    PIDCode();
    controlLoopCounterPID++;
  }
}

void IMUCode()
{  
  // Read angles
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER); 
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

void PIDCode()
{
  // Set limits to angles  
  if(yAngle > 360){yAngle = yAngle %360;}
  if(zAngle > 360){zAngle = zAngle %360;}

  // Get current orientation readings   
  int currentPositionWrist = yAngle;
  int currentPositionForearm = zAngle;

  // Set target values to initial positions
  int targetZ = zAngleInit;
  int targetY = yAngleInit;
  
  // Define where output values are going to be stored
  int outputY, outputZ;

  // Call the PID library
  wristPID.pid(currentPositionWrist,targetY,kPWrist,outputY);
  forearmPID.pid(currentPositionForearm, targetZ, kPForearm, outputZ);
}

void timer2_setup()
{
  TCCR2B &= ~(1 << CS22); // Set prescaler as n = 64
  TCCR2B |= (1 << CS21); 
  TCCR2B |= (1 << CS20);  

  TCCR2B |= (1 << WGM22);
  TIMSK2 |= (1 << OCIE2A); // Output compare mode 
  OCR2A = 250;             // Frequency = f_clock/[2*n(1+OCR2A)] ~ 1 kHz ocr2a = 250
}
void timer0_setup()
{
  TCCR0B |= (1 << CS02); // Set prescaler as n = 256
  TCCR0B &= ~(1 << CS01); 
  TCCR0B &= ~(1 << CS00);  

  TCCR0B |= (1 << WGM02);
  TIMSK0 |= (1 << OCIE0A); // Output compare mode 
  OCR0A = 125;             // Frequency = f_clock/[2*n(1+OCR2A)]  ~500 Hz ocr0a = 125 456hz imu
}

