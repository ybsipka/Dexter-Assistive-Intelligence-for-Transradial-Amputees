/*
  Main Program for Dexter: An Assistive Intelligence for Transradial Amputees

  DEXTER has 2 different Modes: a) self balancing mode b) grasping mode

  a ) SELF BALANCING MODE 

    This part of the software is for 2 degree of freedom wrist and forearm control. With the instructions below,
 Dexter can hold a cup and it will compansate in the Z and X axis of movement from the 
 attached arm. 

 The axis of movement are determined by the IMU's orientation;
  *Holding Dexter in front of you* 

  * ↷ is determined as +Z.
  * → is determined as +X.
  * ↑  is determined as +Y.
 
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


  b ) GRASPING MODE
  
*/
/********************** INCLUDES **************************/
#include <SPH_PID.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <TimerOne.h>

/********************* PINS ******************/
// Forearm Pins
#define MD4_AIN1 53
#define MD4_AIN2 52
#define MD4_PWMA 13

// Wrist Pins
#define MD3_AIN1 51
#define MD3_AIN2 50
#define MD3_PWMA 12

// Thumb Top
#define MD3_BIN1 49
#define MD3_BIN2 48
#define MD3_PWMB 11

// Thumb Left
#define MD2_AIN1 47
#define MD2_AIN2 46
#define MD2_PWMA 10

// Ring
#define MD2_BIN1 45
#define MD2_BIN2 44
#define MD2_PWMB 9

// Middle
#define MD1_AIN1 43
#define MD1_AIN2 42
#define MD1_PWMA 8

// Index
#define MD1_BIN1 41
#define MD1_BIN2 40
#define MD1_PWMB 7

/**************** VARIABLES *******************/
// P constants for motor control
#define kPForearm 8  //******** HAVE TO ADJUST THE GAIN SO THAT IT MATCHES THE ACTIVATION VOLTAGE OF THE MOTOR
#define kPWrist 12   //WEIRD
#define kPIndex 3
#define kPMiddle 3
#define kPRing 3
#define kPThumbLeft 3
#define kPThumbTop 3

// Potentiometer Pins
#define indexPotPin 0
#define middlePotPin 1
#define ringPotPin 2
#define thumbLeftPotPin 3
#define thumbTopPotPin 4

// Force Sensor Pins
#define indexFSRpinTop 5
#define middleFSRpinTop 6
#define ringFSRpinTop 7
#define thumbFSRpinTop 8
#define indexFSRpinBottom 9
#define middleFSRpinBottom 10
#define ringFSRpinBottom 11


// Touch Sensor Pins
#define touchOne 2 //digital Pin 2
#define touchTwo 3 // digital Pin 3

// Closed and opened grasp values
#define TP_INDEX_CLOSED 400
#define TP_INDEX_OPENED 540
#define TP_MIDDLE_CLOSED 150
#define TP_MIDDLE_OPENED 150
#define TP_RING_CLOSED 150
#define TP_RING_OPENED 150
#define TP_THUMBLEFT_CLOSED 150
#define TP_THUMBLEFT_OPENED 150
#define TP_THUMBTOP_CLOSED 150
#define TP_THUMBTOP_OPENED 150

// Force Sensor Thresholds
#define indexFSRTopThreshold 300
#define indexFSRBottomThreshold 300
#define middleFSRTopThreshold 300
#define middleFSRBottomThreshold 300
#define ringFSRTopThreshold 300
#define ringFSRBottomThreshold 300
#define thumbFSRTopThreshold 300

// Instantiate the IMU
Adafruit_BNO055 bno = Adafruit_BNO055();
#define BNO055_SAMPLERATE_DELAY_MS (100)

// Instantiate PID library - SPH_PID(PWM,IN1,IN2) ----
SPH_PID forearmPID(MD4_PWMA, MD4_AIN1, MD4_AIN2);
SPH_PID wristPID(MD3_PWMA, MD3_AIN1, MD3_AIN2);
SPH_PID thumbTopPID(MD3_PWMB, MD3_BIN1, MD3_BIN2);
SPH_PID thumbLeftPID(MD2_PWMA, MD2_AIN1, MD2_AIN2);
SPH_PID ringPID(MD2_PWMB, MD2_BIN1, MD2_BIN2);
SPH_PID middlePID(MD1_PWMA, MD1_AIN1, MD1_AIN2);
SPH_PID indexPID(MD1_PWMB, MD1_BIN1, MD1_BIN2);

// Global Variables
int xAngle,  zAngle, yAngle, xAngleInit, yAngleInit, zAngleInit, targetIndex, targetMiddle, targetRing, targetThumbLeft, targetThumbTop, targetWrist, targetForearm, outputIndex, outputMiddle, outputRing, outputLeft, outputTop, outputWrist, outputForearm;
int currentPositionIndex,currentPositionMiddle,currentPositionRing,currentPositionThumbLeft,currentPositionThumbTop;
int indexFSRTopPos ,indexFSRBottomPos,middleFSRTopPos,middleFSRBottomPos,ringFSRTopPos,ringFSRBottomPos;  
int interruptCounterTimer0 = 0;
int interruptCounterTimer2 = 0;
int controlLoopCounterIMU = 0;
int controlLoopCounterPID = 0;

// Determines which mode the hand is in
#define GRASP 100
#define RELEASEGRASP 200
#define SELFBALANCE 300
volatile int MODE = 0;
int targetCounter = 0;


/* Setup Loop */
void setup()
{
  // Initialize the Serial Monitor at 9600 Baud Rate
  Serial.begin(9600);

  // Set Potentiometer Pins to Inputs
  pinModeInitialize();

  // Check if the IMU is connected
  //IMU_check();

  // Set IMU crystal for external use
  bno.setExtCrystalUse(true);

  /* Interrupt Initializations */
  attachInterrupt(digitalPinToInterrupt(touchOne), lockJoints, RISING);
  attachInterrupt(digitalPinToInterrupt(touchTwo), releaseGrasp, RISING);

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
  printInitialAngles();

  readPotsAndFSRs();
}

// Motor control interrupt timer. Expected at ~1.0 kHz.
ISR(TIMER2_COMPA_vect)
{
  interruptCounterTimer2++;
}

// Sensor feedback loop goes here. Expected at ~500 Hz.
ISR(TIMER0_COMPA_vect)
{
  interruptCounterTimer0++;
}

/* Main Loop */


  
void loop()
{
  switch(MODE)
  {
    case GRASP: // Grasping MODE
      while(targetCounter == 0){
        targetCounter = 1;
      }
      // When Timer0 fires
        if(controlLoopCounterIMU < interruptCounterTimer0)
         {
           readPotsAndFSRs();
           controlLoopCounterIMU++;
         }
      //When Timer2 fires
      if(controlLoopCounterPID < interruptCounterTimer2)
        {
          grasping();
          controlLoopCounterPID++;
        }
    break;
    case RELEASEGRASP: // Releasing MODE
      releasingGrasp();
      targetCounter = 0;
    break;
    case SELFBALANCE: // Self Balancing MODE
      //selfBalance();
    break;
    default: // initial MODE
    //do nothing
    Serial.println("Initial Mode or Something is wrong");
    break;
  }
}

void readPotsAndFSRs(){
  // Get current position readings
  readPots();

  // Get current force sensor readings
  readForceSensors();
}

/* The function for grasping an object */
void grasping()
{
  if (targetCounter == 1)
  {
    setTarget2Closed();
    targetCounter = 2;
  }

  if(indexFSRTopPos > indexFSRTopThreshold || indexFSRBottomPos > indexFSRBottomThreshold)
  {
    targetIndex = currentPositionIndex;   
  }
  if(middleFSRTopPos > middleFSRTopThreshold || middleFSRBottomPos > middleFSRBottomThreshold)
  {
    targetMiddle = currentPositionMiddle;   
  }
  if(ringFSRTopPos > ringFSRTopThreshold || ringFSRBottomPos > ringFSRBottomThreshold)
  {
    targetRing = currentPositionRing;   
  }
  
 // Set Targets - Probably going to be max values Closed
 indexPID.pid(currentPositionIndex,targetIndex,kPIndex,outputIndex);
 middlePID.pid(currentPositionMiddle,targetMiddle,kPMiddle,outputMiddle);
 ringPID.pid(currentPositionRing,targetRing,kPRing,outputRing);
 thumbLeftPID.pid(currentPositionThumbLeft,targetThumbLeft,kPThumbLeft,outputLeft);
 thumbTopPID.pid(currentPositionThumbTop,targetThumbTop,kPThumbTop,outputTop);
}

void setTarget2Closed()
{
  targetIndex = TP_INDEX_CLOSED;
  targetMiddle = TP_MIDDLE_CLOSED;
  targetRing = TP_RING_CLOSED;
  targetThumbLeft = TP_THUMBLEFT_CLOSED;
  targetThumbTop =TP_THUMBTOP_CLOSED; 
}

void setTarget2Opened()
{
  targetIndex = TP_INDEX_OPENED;
  targetMiddle = TP_MIDDLE_OPENED;
  targetRing = TP_RING_OPENED;
  targetThumbLeft = TP_THUMBLEFT_OPENED;
  targetThumbTop =TP_THUMBTOP_OPENED; 
}

void readPots()
{
  currentPositionIndex = analogRead(indexPotPin);
  currentPositionMiddle = analogRead(middlePotPin);
  currentPositionRing = analogRead(ringPotPin);
  currentPositionThumbLeft = analogRead(thumbTopPotPin);
  currentPositionThumbTop = analogRead(thumbLeftPotPin);
}
void readForceSensors()
{
  indexFSRTopPos = analogRead(indexFSRpinTop);
  indexFSRBottomPos = analogRead(indexFSRpinBottom);  
  middleFSRTopPos = analogRead(middleFSRpinTop);  
  middleFSRBottomPos = analogRead(middleFSRpinBottom);  
  ringFSRTopPos = analogRead(ringFSRpinTop);  
  ringFSRBottomPos = analogRead(ringFSRpinBottom);  
  //int thumbFSRTopPos = analogRead(thumbFSRTopPin);  
  
}
/* The function for releasing the grasping*/
void releasingGrasp()
{
  setTarget2Opened();
  // Get current position readings
  int currentPositionIndex = analogRead(indexPotPin);
  int currentPositionMiddle = analogRead(middlePotPin);
  int currentPositionRing = analogRead(ringPotPin);
  int currentPositionThumbLeft = analogRead(thumbTopPotPin);
  int currentPositionThumbTop = analogRead(thumbLeftPotPin);

 // Set Targets - Probably going to be max values Closed
 indexPID.pid(currentPositionIndex,targetIndex,kPIndex,outputIndex);
 middlePID.pid(currentPositionMiddle,targetMiddle,kPMiddle,outputMiddle);
 ringPID.pid(currentPositionRing,targetRing,kPRing,outputRing);
 thumbLeftPID.pid(currentPositionThumbLeft,targetThumbLeft,kPThumbLeft,outputLeft);
 thumbTopPID.pid(currentPositionThumbTop,targetThumbTop,kPThumbTop,outputTop);
}

/* The function for the self balancing mode */
void selfBalance()
{
  // Set limits to angles
  if(yAngle > 360){yAngle = yAngle %360;}
  if(zAngle > 360){zAngle = zAngle %360;}

  // Get current orientation readings
  int currentPositionWrist = yAngle;
  int currentPositionForearm = zAngle;

  // Set target values to initial positions
  int targetWrist = yAngleInit;
  int targetForearm = zAngleInit;

  // Call the PID library
  wristPID.pid(currentPositionWrist,targetWrist,kPWrist,outputWrist);
  forearmPID.pid(currentPositionForearm, targetForearm, kPForearm, outputForearm);
}

/* Reads the IMU */
void IMU_read()
{
  // Read angles
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  xAngle = euler.x();
  yAngle = euler.y();
  zAngle = euler.z();

  // Print angles
  printAngles();
}

/* Initializes pinModes */
void pinModeInitialize()
{
  // Set Potentiometer Pins to Inputs
  pinMode(indexPotPin, INPUT);
  pinMode(middlePotPin, INPUT);
  pinMode(ringPotPin, INPUT);
  pinMode(thumbLeftPotPin, INPUT);
  pinMode(thumbTopPotPin, INPUT);
  pinMode(indexFSRpinTop, INPUT);
  pinMode(middleFSRpinTop, INPUT);
  pinMode(ringFSRpinTop, INPUT);
  pinMode(thumbFSRpinTop, INPUT);
  pinMode(indexFSRpinBottom, INPUT);
  pinMode(middleFSRpinBottom, INPUT);
  pinMode(ringFSRpinBottom, INPUT);
  pinMode(thumbFSRpinBottom, INPUT);
}

/* Checks if IMU is connected via SCL && SDA */
void IMU_check()
{
  // Check if the IMU is connected
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
   Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
   while(1);
  }
}

/* Sets up Timer2 */
void timer2_setup()
{
  TCCR2B &= ~(1 << CS22); // Set prescaler as n = 64
  TCCR2B |= (1 << CS21);
  TCCR2B |= (1 << CS20);

  TCCR2B |= (1 << WGM22);
  TIMSK2 |= (1 << OCIE2A); // Output compare mode
  OCR2A = 250;             // Frequency = f_clock/[2*n(1+OCR2A)] ~ 1 kHz ocr2a = 250
}

/* Sets up Timer0 */
void timer0_setup()
{
  TCCR0B |= (1 << CS02); // Set prescaler as n = 256
  TCCR0B &= ~(1 << CS01);
  TCCR0B &= ~(1 << CS00);

  TCCR0B |= (1 << WGM02);
  TIMSK0 |= (1 << OCIE0A); // Output compare mode
  OCR0A = 125;             // Frequency = f_clock/[2*n(1+OCR2A)]  ~500 Hz ocr0a = 125 456hz imu
}

/* Prints Initial Angles */
void printInitialAngles()
{
  Serial.print("xAngle Initial = ");
  Serial.print(xAngleInit);
  Serial.print("yAngle Init = ");
  Serial.print(yAngleInit);
  Serial.print("zAngle Init = ");
  Serial.println(zAngleInit);
}

/* Prints Angles */
void printAngles()
{
  Serial.print("xAngle = ");
  Serial.print(xAngle);
  Serial.print("yAngle = ");
  Serial.print(yAngle);
  Serial.print("zAngle = ");
  Serial.println(zAngle);
}

/* To check if the interrupts are running at the rate wanted */
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

/* Locks the hand for grasp */
void lockJoints(){
  Serial.println("Lock Joints");
  MODE = GRASP;
}

/* Releases the grasp */
void releaseGrasp(){
  Serial.println("Release Grasp");
  MODE = RELEASEGRASP;
}
