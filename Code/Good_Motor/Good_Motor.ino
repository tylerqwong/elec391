
#define CCW 0
#define CW 1

int motorPin1 = 11;
int motorPin2 = 10;
int motorPin3 = 9;
int motorPin4 = 8;



int dir;        /* Ideally must be set to either CCW or CW */
int motor_state = 0;

int opticalPin = 2;
int Read = 0;
int lastRead;
double last_angle = 0;
double angle_counter = 0;
double angle_input = 0;
int opticalWindows = 50;

void setup() {

  noInterrupts(); //disable all interrupts

// initialize timer1

  TCCR1A = 0;
  TCCR1B = 0;
  
  TCNT1 = 56000; //preload timer 65536-16MHz/256/2Hz
  TCCR1B |= (1 << CS12);  //256 prescaler
  TIMSK1 |= (1 << TOIE1);   //enable timer overflow interrupt
  interrupts(); //enable all interrupts
  
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(motorPin3, OUTPUT);
  pinMode(motorPin4, OUTPUT);
  
  pinMode(opticalPin, INPUT);

  lastRead = digitalRead(opticalPin);

  Serial.begin(9600);
}

ISR(TIMER1_OVF_vect) //interrupt service routine
{
  TCNT1 = 56000;
  if (motor_state == 0)
    motor_state = 1;
  else if (motor_state == 1)
    motor_state = 2;
  else if (motor_state == 2)
    motor_state = 3;
  else 
    motor_state = 0;
}
    
   


void loop() {

  
  if (Serial.available() > 0) {
    angle_input = Serial.parseInt();   
    Serial.println(angle_input);    
  }
  

  
  if (last_angle != angle_counter)
  {
    Serial.println(angle_counter);
    last_angle = angle_counter;
  }

 
  
  Read = digitalRead(opticalPin);
  if (lastRead != Read) 
  {
    if(dir == CCW)
    {
      angle_counter = angle_counter + 3.6;
     // if (angle_counter >= 360)
       // angle_counter = angle_counter - 360;
      lastRead = Read;
    }
    else
    {
      angle_counter = angle_counter - 3.6;
     // if (angle_counter < 0)
       // angle_counter = angle_counter + 360;
      lastRead = Read;
    } 
  }

  if((angle_input - angle_counter) >= 4)
  {
      dir = CCW;

      
      if (motor_state == 0) 
      {
        digitalWrite(motorPin1, HIGH);
        digitalWrite(motorPin2, LOW);
        digitalWrite(motorPin3, LOW);
        digitalWrite(motorPin4, HIGH);
      }
      else if (motor_state == 1) 
      {
        digitalWrite(motorPin1, LOW);
        digitalWrite(motorPin2, HIGH);
        digitalWrite(motorPin3, LOW);
        digitalWrite(motorPin4, HIGH);
      }
       
      else if (motor_state == 2) 
      {
        digitalWrite(motorPin1, LOW);
        digitalWrite(motorPin2, HIGH);
        digitalWrite(motorPin3, HIGH);
        digitalWrite(motorPin4, LOW);
      }
      else if (motor_state == 3)
      {
        digitalWrite(motorPin1, HIGH);
        digitalWrite(motorPin2, LOW);
        digitalWrite(motorPin3, HIGH);
        digitalWrite(motorPin4, LOW);
      }
    else 
    {
      digitalWrite(motorPin1, LOW);
      digitalWrite(motorPin2, LOW);
      digitalWrite(motorPin3, LOW);
      digitalWrite(motorPin4, LOW);
    } 
    
   }

   
  else if ((angle_input - angle_counter) <= -4)
  {

    dir = CW;

    

    if (motor_state == 0) 
    {
      digitalWrite(motorPin1, HIGH);
      digitalWrite(motorPin2, LOW);
      digitalWrite(motorPin3, LOW);
      digitalWrite(motorPin4, HIGH);
    }
    else if (motor_state == 1) 
    {
      digitalWrite(motorPin1, HIGH);
      digitalWrite(motorPin2, LOW);
      digitalWrite(motorPin3, HIGH);
      digitalWrite(motorPin4, LOW);
    }
     
    else if (motor_state == 2) 
    {
      digitalWrite(motorPin1, LOW);
      digitalWrite(motorPin2, HIGH);
      digitalWrite(motorPin3, HIGH);
      digitalWrite(motorPin4, LOW);
    }
    else if (motor_state == 3)
    {
      digitalWrite(motorPin1, LOW);
      digitalWrite(motorPin2, HIGH);
      digitalWrite(motorPin3, LOW);
      digitalWrite(motorPin4, HIGH);
    }  
    else 
    {
      digitalWrite(motorPin1, LOW);
      digitalWrite(motorPin2, LOW);
      digitalWrite(motorPin3, LOW);
      digitalWrite(motorPin4, LOW);
    } 
    
  
  }

  else 
    {
      digitalWrite(motorPin1, LOW);
      digitalWrite(motorPin2, LOW);
      digitalWrite(motorPin3, LOW);
      digitalWrite(motorPin4, LOW);
    }
  

}
