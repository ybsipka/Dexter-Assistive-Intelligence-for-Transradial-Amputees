/* Custom P-Control Loop
 * 1/21/2017
 * Author: Batu Sipka
 */
/**********Pins*******/
#define AIN1 4
#define AIN2 5
#define PWMA 3

/**************** VARIABLES *******************/
#define kP 0.3
#define ERROR_MARGIN 5
#define TARGET_POSITION 1000

int targetPosition, currentPosition, error, pwm = 0;

//void clockwise():
//void c_clockwise():

void setup() {
  Serial.begin(9600);
  targetPosition = TARGET_POSITION;
}

void loop() {
  while(true){
    currentPosition = analogRead(0);
    error = targetPosition - currentPosition;
   
    pwm = kP * error;

    //check if at boundaries
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
    if(abs(error) < ERROR_MARGIN){ //within margin
      analogWrite(PWMA,0);
    }
  }
}

void clockwise(){
  digitalWrite(AIN1, HIGH);  
  digitalWrite(AIN2, LOW);
}

void c_clockwise(){
  digitalWrite(AIN1, LOW);  
  digitalWrite(AIN2, HIGH);
}
