
#define CCW 0
#define CW 1

int motorPin1 = 8;
int motorPin2 = 9;
int motorPin3 = 10;
int motorPin4 = 11;
int motorPin5 = 4;
int motorPin6 = 5;
int delayTime = 100;  /* In milliseconds, note that 100ms is about max for demo1 motor */ 
int dir = CCW;        /* Ideally must be set to either CCW or CW */

void setup() {
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(motorPin3, OUTPUT);
  pinMode(motorPin4, OUTPUT);
  pinMode(motorPin5, OUTPUT);
  pinMode(motorPin6, OUTPUT);
}

void loop() {

  if( dir == CCW ){
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
    digitalWrite(motorPin3, LOW);
    digitalWrite(motorPin4, LOW);
    digitalWrite(motorPin5, LOW);
    digitalWrite(motorPin6, LOW);
    delay(delayTime);
  
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, LOW);
    digitalWrite(motorPin3, LOW);
    digitalWrite(motorPin4, LOW);
    digitalWrite(motorPin5, HIGH);
    digitalWrite(motorPin6, LOW);
    delay(delayTime);
    
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, LOW);
    digitalWrite(motorPin3, HIGH);
    digitalWrite(motorPin4, LOW);
    digitalWrite(motorPin5, LOW);
    digitalWrite(motorPin6, LOW);
    delay(delayTime);
  }
    
  else{
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
    digitalWrite(motorPin3, LOW);
    digitalWrite(motorPin4, LOW);
    digitalWrite(motorPin5, LOW);
    digitalWrite(motorPin6, LOW);
    delay(delayTime);
  
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, LOW);
    digitalWrite(motorPin3, HIGH);
    digitalWrite(motorPin4, LOW);
    digitalWrite(motorPin5, LOW);
    digitalWrite(motorPin6, LOW);
    delay(delayTime);
    
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, LOW);
    digitalWrite(motorPin3, LOW);
    digitalWrite(motorPin4, LOW);
    digitalWrite(motorPin5, HIGH);
    digitalWrite(motorPin6, LOW);
    delay(delayTime);
  }
}
