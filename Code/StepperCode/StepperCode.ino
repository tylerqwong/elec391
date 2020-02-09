
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
int motor_state = 0;

void setup() {

  noInterrupts(); //disable all interrupts

// initialize timer1

  TCCR1A = 0;
  TCCR1B = 0;
  
  TCNT1 = 34286; //preload timer 65536-16MHz/256/2Hz
  TCCR1B |= (1 << CS12);  //256 prescaler
  TIMSK1 |= (1 << TOIE1);   //enable timer overflow interrupt
  interrupts(); //enable all interrupts
  
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(motorPin3, OUTPUT);
  pinMode(motorPin4, OUTPUT);
  pinMode(motorPin5, OUTPUT);
  pinMode(motorPin6, OUTPUT);

  // motor pins 2,4,6 are always low
  digitalWrite(motorPin2, LOW);
  digitalWrite(motorPin4, LOW);
  digitalWrite(motorPin6, LOW);
}

ISR(TIMER1_OVF_vect) //interrupts service routine
{
  TCNT1 = 62411;
  if (motor_state == 0)
    motor_state = 1;
  else if (motor_state == 1)
    motor_state = 2;
  else 
    motor_state = 0;
}
    
   


void loop() {



  if(dir == CCW)
  {
      if (motor_state == 0) 
      {
        digitalWrite(motorPin1, HIGH);
        digitalWrite(motorPin3, LOW);
        digitalWrite(motorPin5, LOW);
      }
      else if (motor_state == 1) 
      {
        digitalWrite(motorPin1, LOW);
        digitalWrite(motorPin3, HIGH);
        digitalWrite(motorPin5, LOW);
      }
       
      else if (motor_state == 2) 
      {
        digitalWrite(motorPin1, LOW);
        digitalWrite(motorPin3, LOW);
        digitalWrite(motorPin5, HIGH);
      }
      else 
      {
        digitalWrite(motorPin1, LOW);
        digitalWrite(motorPin3, LOW);
        digitalWrite(motorPin5, LOW);
      }
   }
  else 
  {
    if (motor_state == 0) 
    {
      digitalWrite(motorPin1, HIGH);
      digitalWrite(motorPin3, LOW);
      digitalWrite(motorPin5, LOW);
    }
    else if (motor_state == 1) 
    {
      digitalWrite(motorPin1, LOW);
      digitalWrite(motorPin3, LOW);
      digitalWrite(motorPin5, HIGH);
    }
     
    else if (motor_state == 2) 
    {
      digitalWrite(motorPin1, LOW);
      digitalWrite(motorPin3, HIGH);
      digitalWrite(motorPin5, LOW);
    }
    else 
    {
      digitalWrite(motorPin1, LOW);
      digitalWrite(motorPin3, LOW);
      digitalWrite(motorPin5, LOW);
    }  
  }
}
