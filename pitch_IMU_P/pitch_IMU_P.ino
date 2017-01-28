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
#define kP 0.2
#define ERROR_MARGIN 10
#define TARGET_POSITION 500

int t_pos, c_pos,error;
int pwm = 0;

char buffer[MAX_OUT_CHARS + 1];


// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  t_pos = TARGET_POSITION;
}

// the loop routine runs over and over again forever:
void loop() {
  while(true){
  // read the input on analog pin 0:
  int c_pos = analogRead(A0);
  // print out the value you read:
  Serial.println(c_pos);
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
    sprintf(buffer, "Output= %d",pwm);
    Serial.println(buffer); 
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
    sprintf(buffer, "Output= %d",error);
    Serial.println(buffer); 
    
}
void clockwise(){
  digitalWrite(AIN1, HIGH);  
  digitalWrite(AIN2, LOW);
  
}

void c_clockwise(){
  digitalWrite(AIN1, LOW);  
  digitalWrite(AIN2, HIGH);
}
