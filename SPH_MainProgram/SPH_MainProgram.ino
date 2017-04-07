/*
  Main Program for Dexter: An Assistive Intelligence for Transradial Amputees

  Batu Sipka
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
#define kPIndex 4.5
#define kPMiddle 4.5
#define kPRing 4.5
#define kPThumbLeft 4.5
#define kPThumbTop 4.5

// Potentiometer Pins
#define indexPotPin 0
#define middlePotPin 1
#define ringPotPin 2
#define thumbLeftPotPin 3
#define thumbTopPotPin 4

// Force Sensor Pins - don't think we are gonna use interrupts
//change them if you are using digitalPinToInterrupt()
#define indexFSRpin 2 //21
#define middleFSRpin 3 //20
#define ringFSRpin 4 //19
#define thumbFSRpin 5 //18

// Touch Sensor Pins
#define touchOne 0 //digital Pin 2
#define touchTwo 1 // digital Pin 3

// Closed and opened grasp values
#define TP_INDEX_CLOSED 50
#define TP_INDEX_OPENED 50
#define TP_MIDDLE_CLOSED 150
#define TP_MIDDLE_OPENED 150
#define TP_RING_CLOSED 150
#define TP_RING_OPENED 150
#define TP_THUMBLEFT_CLOSED 150
#define TP_THUMBLEFT_OPENED 150
#define TP_THUMBTOP_CLOSED 150
#define TP_THUMBTOP_OPENED 150

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
int interruptCounterTimer0 = 0;
int interruptCounterTimer2 = 0;
int controlLoopCounterIMU = 0;
int controlLoopCounterPID = 0;

void setup()
{
  // Initialize the Serial Monitor at 9600 Baud Rate
  Serial.begin(9600);

  // Set Potentiometer Pins to Inputs
  pinMode(indexPotPin, INPUT);
  pinMode(middlePotPin, INPUT);
  pinMode(ringPotPin, INPUT);
  pinMode(thumbLeftPotPin, INPUT);
  pinMode(thumbTopPotPin, INPUT);

  // Check if the IMU is connected
  IMU_check();

  // Set IMU crystal for external use
  bno.setExtCrystalUse(true);

  /* Interrupt Initializations */
  attachInterrupt(touchOne, lock, RISING);
  attachInterrupt(touchTwo, release, RISING);

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

void loop()
{

}

void grasping()
{
  // Get current position readings
  int currentPositionIndex = analogRead(indexPotPin);
  int currentPositionMiddle = analogRead(middlePotPin);
  int currentPositionRing = analogRead(ringPotPin);
  int currentPositionThumbLeft = analogRead(thumbTopPotPin);
  int currentPositionThumbTop = analogRead(thumbLeftPotPin);

// Force sensor code here
/*
  if(middleTouched > 0){
   targetMiddle = currentPositionMiddle;
   middleTouched = 0;
   Serial.print("MIDDLE TOUCHEDD!!!!!!!!!!!!!!!!!");
  }
  if(indexTouched > 0){
   targetIndex = currentPositionIndex;
   indexTouched = 0;
   Serial.print("INDEX TOUCHEDD!!!!!!!!!!!!!!!!!");
 }*/

 // Set Targets - Probably going to be max values Closed
 indexPID.pid(currentPositionIndex,TP_INDEX_CLOSED,kPIndex,outputIndex);
 middlePID.pid(currentPositionMiddle,TP_MIDDLE_CLOSED,kPMiddle,outputMiddle);
 ringPID.pid(currentPositionRing,TP_RING_CLOSED,kPRing,outputRing);
 thumbLeftPID.pid(currentPositionThumbLeft,TP_THUMBLEFT_CLOSED,kPThumbLeft,outputLeft);
 thumbTopPID.pid(currentPositionThumbTop,TP_THUMBTOP_CLOSED,kPThumbTop,outputTop);
}

void selfBalance()
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

  // Call the PID library
  wristPID.pid(currentPositionWrist,targetY,kPWrist,outputY);
  forearmPID.pid(currentPositionForearm, targetZ, kPForearm, outputZ);
}

void IMU_read()
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

void lock(){

}

void release(){

}
