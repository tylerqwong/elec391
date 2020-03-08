#define CW 0
#define CCW 1
#define hold_phase 2

int dir;

//Input sensors of encoders
int A = 3;
int B = 2;

//ISR Variables
int curA;
int curB;
int oldA = 0;
int oldB = 0;

volatile int counter = 0;
volatile int rotation_counter = 0;

//Motor state variables
int motor_state = 0;
int motor_cal_err = 0;

//Motor Logic
int motor1_1 = 8;
int motor1_2 = 9;
int motor1_3 = 10;
int motor2_1;
int motor2_2;
int motor2_3;
//Enable pin for motor 2, used for PWM  
int pwm_pin = 6;
//motor pin 1 hotfix
int motorpin1 = 7;

//Logic Power Supply
int Logic_power = 13;

//Motor calibration position
//CW
int CW_state_12;
int CW_state_11;
int CW_state_10;
int CW_state_9;
int CW_state_8;
int CW_state_7;
int CW_state_6;
int CW_state_5;
int CW_state_4;
int CW_state_3;
int CW_state_2;
int CW_state_1;
//CCW
int CCW_state_1;
int CCW_state_2;
int CCW_state_3;
int CCW_state_4;
int CCW_state_5;
int CCW_state_6;
int CCW_state_7;
int CCW_state_8;
int CCW_state_9;
int CCW_state_10;
int CCW_state_11;
int CCW_state_12;

//PID Variables and Constants
int run_pid;
int error;
int prev_error = 0;
int PID_p;
int PID_i;
int PID_d;
int PID_total;
float kp = 1;
float ki = 1;
float kd = 1;
float period = 0.01;

//Input Setpoints
int rotation_counter_setpoint = -1000;
int counter_setpoint;
int setpoint;

void setup() {
  /*
  //Initialize timer1
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 64911; //preload timer 65536-16MHz/256/20Hz
  TCCR1B |= (1 << CS12);  //256 prescaler
  TIMSK1 |= (1 << TOIE1);   //enable timer overflow interrupt
  interrupts();
*/

  //Set encoder pins to input
  pinMode(A, INPUT);
  pinMode(B, INPUT);

  //Set Enable to on
  pinMode(pwm_pin, OUTPUT);
  analogWrite(pwm_pin, 255);
  
  
  //Logic Power Supply to PCB
  pinMode(Logic_power, OUTPUT);
  digitalWrite(Logic_power, HIGH);

  //Hotfix (yellow wire to PLD)
  pinMode(motorpin1, OUTPUT);
  digitalWrite(motorpin1, LOW);

  //Motor Logic Pins (J1)
  pinMode(motor1_1, OUTPUT);
  pinMode(motor1_2, OUTPUT);
  pinMode(motor1_3, OUTPUT);
  pinMode(motor2_1, OUTPUT);
  pinMode(motor2_2, OUTPUT);
  pinMode(motor2_3, OUTPUT);

  //Turn Motor 2 Off
  digitalWrite(motor2_1, LOW);
  digitalWrite(motor2_2, LOW);
  digitalWrite(motor2_3, LOW);
 

  Serial.begin(9600);

  //Encoder ISR reading for both rising and falling edges
  attachInterrupt(digitalPinToInterrupt(A), isrA, CHANGE); 
  attachInterrupt(digitalPinToInterrupt(B), isrA, CHANGE); 
  
  //Calibrate Motor
  while(1){
  motor_calibration();
  motor_calibration_test();
  if(motor_cal_err == 0)
    break;
  }
  rotation_counter = 0;
  //setpoint = rotation_counter_setpoint*400 + CW_state_1/*(counter_setpoint)*/;
}



void loop() {  
  //PID control
  //if(run_pid == 1)
  //  pid();
  dir = CW;
  //Motor control and driver functions
  motor_control();
  motor_driver();
}

/*-------------------------------Interrupt Service Routines------------------------------------*/

/*
ISR(TIMER1_OVF_vect) //interrupt service routine for timer 1
{
  run_pid = 1; //flag to start PID loop
  TCNT1 = 64911; //Timer runs between 64911 and 65536 for 100Hz PID refresh   
}
*/

void isrA() {
  curA = digitalRead(A);
  curB = digitalRead(B);
  //add 1 to count for CW or -1 for CCW
    if(curA && !curB && !oldA && !oldB) //CW
    {
      counter++;
    }
    else if(curA && curB && oldA && !oldB) //CW
    {
      counter++;
    }
    else if(!curA && curB && oldA && oldB) //CW
    {
      counter++;
    }
    else if(!curA && !curB && !oldA && oldB) //CW
    {
      counter++;
    }
    else if(!curA && !curB && oldA && !oldB) //CCW
    {
      counter--;
    }
    else if(!curA && curB && !oldA && !oldB) //CCW
    {
      counter--;
    }
    else if(curA && curB && !oldA && oldB) //CCW
    {
      counter--;
    }
    else if(curA && !curB && oldA && oldB) //CCW
    {
      counter--;
    }
    else
      counter = counter;

   if (counter >= 400)
   {
    counter = counter - 400;
    rotation_counter++;
   }

   if (counter < 0)
   {
    counter = counter + 400;
    rotation_counter--;
   }

    oldA = curA;
    oldB = curB;
}

/*---------------------------Control Algorithms---------------------------*/

void pid() {
  error = setpoint - (rotation_counter*400 + counter);
  
  PID_p = kp * error;
 /*
  PID_i = PID_i + (ki * period);

  float difference = error - prev_error;
  PID_d = kd * ((difference)/period);
    
  PID_total = PID_p + PID_i + PID_d;
  */
  PID_total = PID_p;
  
  if(PID_total > 0)
    dir = CW;
  else
    dir = CCW;

  PID_total = abs(PID_total);

  if (PID_total > 400)
    analogWrite(pwm_pin, 255);
  else 
  {
    PID_total = map(PID_total,0,400,255,255);
    analogWrite(pwm_pin, PID_total);
  }
  
 // prev_error = error;  
 // run_pid = 0;
}

void motor_driver() {
 //Motor controller CCW
    
    if(motor_state == 0) {
      digitalWrite(motor1_1, LOW);
      digitalWrite(motor1_2, LOW);
      digitalWrite(motor1_3, LOW);
      digitalWrite(motorpin1, LOW);
    }
    else if ((motor_state == 1)||(motor_state == 7)) {
      digitalWrite(motor1_1, LOW);
      digitalWrite(motor1_2, LOW);
      digitalWrite(motor1_3, HIGH);
      digitalWrite(motorpin1, HIGH);
    }    
    else if ((motor_state == 2)||(motor_state == 8)) {
      digitalWrite(motor1_1, LOW);
      digitalWrite(motor1_2, HIGH);
      digitalWrite(motor1_3, LOW);
      digitalWrite(motorpin1, LOW);
    }
    else if ((motor_state == 3)||(motor_state == 9)) {
      digitalWrite(motor1_1, LOW);
      digitalWrite(motor1_2, HIGH);
      digitalWrite(motor1_3, HIGH);
      digitalWrite(motorpin1, LOW);
    }
    else if ((motor_state == 4)||(motor_state == 10)) {
      digitalWrite(motor1_1, HIGH);
      digitalWrite(motor1_2, LOW);
      digitalWrite(motor1_3, LOW);
      digitalWrite(motorpin1, LOW);
    }
    else if ((motor_state == 5)||(motor_state == 11)) {
      digitalWrite(motor1_1, HIGH);
      digitalWrite(motor1_2, LOW);
      digitalWrite(motor1_3, HIGH);
      digitalWrite(motorpin1, LOW);
    }
    else if ((motor_state == 6)||(motor_state == 12)) {
      digitalWrite(motor1_1, HIGH);
      digitalWrite(motor1_2, HIGH);
      digitalWrite(motor1_3, LOW);
      digitalWrite(motorpin1, HIGH);
    }
    else {
      digitalWrite(motor1_1, LOW);
      digitalWrite(motor1_2, LOW);
      digitalWrite(motor1_3, LOW);
      digitalWrite(motorpin1, LOW);
    }

}

void motor_control() {
  //CW Direction
  if(dir == CW) 
  {
    if ((counter < (CW_state_1 + 5)) && (counter > (CW_state_1 - 5)))
      motor_state = 12; 
      
    else if ((counter < (CW_state_12 + 5)) && (counter > (CW_state_12 - 5)))
      motor_state = 11;
      
    else if ((counter < (CW_state_11 + 5)) && (counter > (CW_state_11 - 5)))
      motor_state = 10;
 
    else if ((counter < (CW_state_10 + 5)) && (counter > (CW_state_10 - 5)))
      motor_state = 9;
    
    else if ((counter < (CW_state_9 + 5)) && (counter > (CW_state_9 - 5)))
      motor_state = 8;  
      
    else if ((counter < (CW_state_8 + 5)) && (counter > (CW_state_8 - 5)))
      motor_state = 7;
      
    else if ((counter < (CW_state_7 + 5)) && (counter > (CW_state_7 - 5)))
      motor_state = 6;
      
    else if ((counter < (CW_state_6 + 5)) && (counter > (CW_state_6 - 5)))
      motor_state = 5;
      
    else if ((counter < (CW_state_5 + 5)) && (counter > (CW_state_5 - 5)))
      motor_state = 4;
    
    else if ((counter < (CW_state_4 + 5)) && (counter > (CW_state_4 - 5)))
      motor_state = 3;
    
    else if ((counter < (CW_state_3 + 5)) && (counter > (CW_state_3 - 5)))
      motor_state = 2;
   
    else if ((counter < (CW_state_2 + 5)) && (counter > (CW_state_2 - 5)))
      motor_state = 1;
   
    else
      motor_state = 0; 
  }
  
  else if(dir == CCW) 
  {
    if ((counter < (CCW_state_12 + 5)) && (counter > (CCW_state_12 - 5)))
      motor_state = 1;
   
    else if ((counter < (CCW_state_1 + 5)) && (counter > (CCW_state_1 - 5)))
      motor_state = 2;
    
    else if ((counter < (CCW_state_2 + 5)) && (counter > (CCW_state_2 - 5)))
      motor_state = 3;
    
    else if ((counter < (CCW_state_3 + 5)) && (counter > (CCW_state_3 - 5)))
      motor_state = 4;
    
    else if ((counter < (CCW_state_4 + 5)) && (counter > (CCW_state_4 - 5)))
      motor_state = 5;
    
    else if ((counter < (CCW_state_5 + 5)) && (counter > (CCW_state_5 - 5)))
      motor_state = 6;
    
    else if ((counter < (CCW_state_6 + 5)) && (counter > (CCW_state_6 - 5)))
      motor_state = 7;
    
    else if ((counter < (CCW_state_7 + 5)) && (counter > (CCW_state_8 - 5)))
      motor_state = 8;
    
    else if ((counter < (CCW_state_8 + 5)) && (counter > (CCW_state_8 - 5)))
      motor_state = 9;
    
    else if ((counter < (CCW_state_9 + 5)) && (counter > (CCW_state_9 - 5)))
      motor_state = 10;

    else if ((counter < (CCW_state_10 + 5)) && (counter > (CCW_state_10 - 5)))
      motor_state = 11;
    
    else if ((counter < (CCW_state_11 + 5)) && (counter > (CCW_state_11 - 5)))
      motor_state = 12;
    
    else
      motor_state = 0;
  }
  
  else
    motor_state = 0;
}


/*---------------------------------------------Calibration Setup--------------------------------------------*/

void motor_calibration() {
  Serial.println("Press 1 to calibrate motor");
  while(1){
    if(Serial.parseInt() == 1){
      Serial.println("Calibrating motor...");
      break;
    }
   } 
   
//Calibrate by recording each state window (CW)
   Serial.println("Calibrating motor CW");
   motor_state = 1;
   motor_driver();
   delay(1000);
   counter = 0;
   CW_state_1 = counter;
   Serial.print("CW_state_1: ");
   Serial.println(CW_state_1);
   delay(500);
   motor_state = 12;
   motor_driver();
   delay(1000);
   CW_state_12 = counter;
   Serial.print("CW_state_12: ");
   Serial.println(CW_state_12);
   delay(500);
   motor_state = 11;
   motor_driver();
   delay(1000);
   CW_state_11 = counter;
   Serial.print("CW_state_11: ");
   Serial.println(CW_state_11);
   delay(500);
   motor_state = 10;
   motor_driver();
   delay(1000);
   CW_state_10 = counter;
   Serial.print("CW_state_10: ");
   Serial.println(CW_state_10);
   delay(500);
   motor_state = 9;
   motor_driver();
   delay(1000);
   CW_state_9 = counter;
   Serial.print("CW_state_9: ");
   Serial.println(CW_state_9);
   delay(500);
   motor_state = 8;
   motor_driver();
   delay(1000);
   CW_state_8 = counter;
   Serial.print("CW_state_8: ");
   Serial.println(CW_state_8);
   delay(500);
   motor_state = 7;
   motor_driver();
   delay(1000);
   CW_state_7 = counter;
   Serial.print("CW_state_7: ");
   Serial.println(CW_state_7);
   delay(500);
   motor_state = 6;
   motor_driver();
   delay(1000);
   CW_state_6 = counter;
   Serial.print("CW_state_6: ");
   Serial.println(CW_state_6);
   delay(500);
   motor_state = 5;
   motor_driver();
   delay(1000);
   CW_state_5 = counter;
   Serial.print("CW_state_5: ");
   Serial.println(CW_state_5);
   delay(500);
   motor_state = 4;
   motor_driver();
   delay(1000);
   CW_state_4 = counter;
   Serial.print("CW_state_4: ");
   Serial.println(CW_state_4);
   delay(500);
   motor_state = 3;
   motor_driver();
   delay(1000);
   CW_state_3 = counter;
   Serial.print("CW_state_3: ");
   Serial.println(CW_state_3);
   delay(500);
   motor_state = 2;
   motor_driver();
   delay(1000);
   CW_state_2 = counter;
   Serial.print("CW_state_2: ");
   Serial.println(CW_state_2);
   Serial.println("Motor calibrated CW");
   delay(500);

//Calibrate by recording each state window (CCW)
   Serial.println("Calibrating motor CCW");
   motor_state = 1;
   motor_driver();
   delay(1000);
   CCW_state_1 = counter;
   Serial.print("CCW_state_1: ");
   Serial.println(CCW_state_1);
   delay(500);
   motor_state = 2;
   motor_driver();
   delay(1000);
   CCW_state_2 = counter;
   Serial.print("CCW_state_2: ");
   Serial.println(CCW_state_2);
   delay(500);
   motor_state = 3;
   motor_driver();
   delay(1000);
   CCW_state_3 = counter;
   Serial.print("CCW_state_3: ");
   Serial.println(CCW_state_3);
   delay(500);
   motor_state = 4;
   motor_driver();
   delay(1000);
   CCW_state_4 = counter;
   Serial.print("CCW_state_4: ");
   Serial.println(CCW_state_4);
   delay(500);
   motor_state = 5;
   motor_driver();
   delay(1000);
   CCW_state_5 = counter;
   Serial.print("CCW_state_5: ");
   Serial.println(CCW_state_5);
   delay(500);
   motor_state = 6;
   motor_driver();
   delay(1000);
   CCW_state_6 = counter;
   Serial.print("CCW_state_6: ");
   Serial.println(CCW_state_6);
   delay(500);
   motor_state = 7;
   motor_driver();
   delay(1000);
   CCW_state_7 = counter;
   Serial.print("CCW_state_7: ");
   Serial.println(CCW_state_7);
   delay(500);
   motor_state = 8;
   motor_driver();
   delay(1000);
   CCW_state_8 = counter;
   Serial.print("CCW_state_8: ");
   Serial.println(CCW_state_8);
   delay(500);
   motor_state = 9;
   motor_driver();
   delay(1000);
   CCW_state_9 = counter;
   Serial.print("CCW_state_9: ");
   Serial.println(CCW_state_9);
   delay(500);
   motor_state = 10;
   motor_driver();
   delay(1000);
   CCW_state_10 = counter;
   Serial.print("CCW_state_10: ");
   Serial.println(CCW_state_10);
   delay(500);
   motor_state = 11;
   motor_driver();
   delay(1000);
   CCW_state_11 = counter;
   Serial.print("CCW_state_11: ");
   Serial.println(CCW_state_11);
   delay(500);
   motor_state = 12;
   motor_driver();
   delay(1000);
   CCW_state_12 = counter;
   Serial.print("CCW_state_12: ");
   Serial.println(CCW_state_12);
   delay(500);
   motor_state = 1;
   motor_driver();
   delay(1000);
   Serial.println("Motor calibrated");  
}
void motor_calibration_test() {
//Test CW
   motor_state = 1;
   motor_driver();
   delay(1000);
   Serial.print("state_1: ");
   Serial.println(counter);
   if(CW_state_1 > 390)
    CW_state_1 = CW_state_1 - 400;
   if(counter > 390)
    counter = counter - 400;
   if ((counter > (CW_state_1 + 5)) || (counter < (CW_state_1 - 5)))
    motor_cal_err = 1;

   motor_state = 12;
   motor_driver();
   delay(1000);
   Serial.print("state_12: ");
   Serial.println(counter);
   if ((counter > (CW_state_12+5)) || (counter < (CW_state_12-5)))
    motor_cal_err = 1;

   motor_state = 11;
   motor_driver();
   delay(1000);
   Serial.print("state_11: ");
   Serial.println(counter);
   if ((counter > (CW_state_11+5)) || (counter < (CW_state_11-5)))
    motor_cal_err = 1;

   motor_state = 10;
   motor_driver();
   delay(1000);
   Serial.print("state_10: ");
   Serial.println(counter);
   if ((counter > (CW_state_10+5)) || (counter < (CW_state_10-5)))
    motor_cal_err = 1;

   motor_state = 9;
   motor_driver();
   delay(1000);
   Serial.print("state_9: ");
   Serial.println(counter);
   if ((counter > (CW_state_9+5)) || (counter < (CW_state_9-5)))
    motor_cal_err = 1;

   motor_state = 8;
   motor_driver();
   delay(1000);
   Serial.print("state_8: ");
   Serial.println(counter);
   if ((counter > (CW_state_8+5)) || (counter < (CW_state_8-5)))
    motor_cal_err = 1;

   motor_state = 7;
   motor_driver();
   delay(1000);
   Serial.print("state_7: ");
   Serial.println(counter);
   if ((counter > (CW_state_7+5)) || (counter < (CW_state_7-5)))
    motor_cal_err = 1; 
    
   motor_state = 6;
   motor_driver();
   delay(1000);
   Serial.print("state_6: ");
   Serial.println(counter);
   if ((counter > (CW_state_6+5)) || (counter < (CW_state_6-5)))
    motor_cal_err = 1;
    
   motor_state = 5;
   motor_driver();
   delay(1000);
   Serial.print("state_5: ");
   Serial.println(counter);
   if ((counter > (CW_state_5+5)) || (counter < (CW_state_5-5)))
    motor_cal_err = 1;
    
   motor_state = 4;
   motor_driver();
   delay(1000);
   Serial.print("state_4: ");
   Serial.println(counter);
   if ((counter > (CW_state_4+5)) || (counter < (CW_state_4-5)))
    motor_cal_err = 1;
    
   motor_state = 3;
   motor_driver();
   delay(1000);
   Serial.print("state_3: ");
   Serial.println(counter);
   if ((counter > (CW_state_3+5)) || (counter < (CW_state_3-5)))
    motor_cal_err = 1;
    
   motor_state = 2;
   motor_driver();
   delay(1000);
   Serial.print("state_2: ");
   Serial.println(counter);
   if ((counter > (CW_state_2+5)) || (counter < (CW_state_2-5)))
    motor_cal_err = 1;
  if(motor_cal_err == 0)
   Serial.println("CW Calibration Test Passed."); 
  else {
    Serial.println("CW Calibration Test Failed."); 
    return;
  }

//Test CCW
   motor_state = 1;
   motor_driver();
   delay(1000);
   Serial.print("state_1: ");
   Serial.println(counter);
   if(CCW_state_1 > 390)
    CCW_state_1 = CCW_state_1 - 400;
   if(counter > 390)
    counter = counter - 400;
   if ((counter > (CCW_state_1+5)) || (counter < (CCW_state_1-5)))
    motor_cal_err = 1;

   motor_state = 2;
   motor_driver();
   delay(1000);
   Serial.print("state_2: ");
   Serial.println(counter);
   if ((counter > (CCW_state_2+5)) || (counter < (CCW_state_2-5)))
    motor_cal_err = 1;

   motor_state = 3;
   motor_driver();
   delay(1000);
   Serial.print("state_3: ");
   Serial.println(counter);
   if ((counter > (CCW_state_3+5)) || (counter < (CCW_state_3-5)))
    motor_cal_err = 1;

   motor_state = 4;
   motor_driver();
   delay(1000);
   Serial.print("state_4: ");
   Serial.println(counter);
   if ((counter > (CCW_state_4+5)) || (counter < (CCW_state_4-5)))
    motor_cal_err = 1;

   motor_state = 5;
   motor_driver();
   delay(1000);
   Serial.print("state_5: ");
   Serial.println(counter);
   if ((counter > (CCW_state_5+5)) || (counter < (CCW_state_5-5)))
    motor_cal_err = 1;

   motor_state = 6;
   motor_driver();
   delay(1000);
   Serial.print("state_6: ");
   Serial.println(counter);
   if ((counter > (CCW_state_6+5)) || (counter < (CCW_state_6-5)))
    motor_cal_err = 1;

   motor_state = 7;
   motor_driver();
   delay(1000);
   Serial.print("state_7: ");
   Serial.println(counter);
   if ((counter > (CCW_state_7+5)) || (counter < (CCW_state_7-5)))
    motor_cal_err = 1; 
    
   motor_state = 8;
   motor_driver();
   delay(1000);
   Serial.print("state_8: ");
   Serial.println(counter);
   if ((counter > (CCW_state_8+5)) || (counter < (CCW_state_8-5)))
    motor_cal_err = 1;
    
   motor_state = 9;
   motor_driver();
   delay(1000);
   Serial.print("state_9: ");
   Serial.println(counter);
   if ((counter > (CCW_state_9+5)) || (counter < (CCW_state_9-5)))
    motor_cal_err = 1;
    
   motor_state = 10;
   motor_driver();
   delay(1000);
   Serial.print("state_10: ");
   Serial.println(counter);
   if ((counter > (CCW_state_10+5)) || (counter < (CCW_state_10-5)))
    motor_cal_err = 1;
    
   motor_state = 11;
   motor_driver();
   delay(1000);
   Serial.print("state_11: ");
   Serial.println(counter);
   if ((counter > (CCW_state_11+5)) || (counter < (CCW_state_11-5)))
    motor_cal_err = 1;
    
   motor_state = 12;
   motor_driver();
   delay(1000);
   Serial.print("state_12: ");
   Serial.println(counter);
   if ((counter > (CCW_state_12+5)) || (counter < (CCW_state_12-5)))
    motor_cal_err = 1;

   motor_state = 1;
   motor_driver();
   delay(1000);
   Serial.print("state_1: ");
   Serial.println(counter);
     
   if(motor_cal_err == 0)
    Serial.println("CCW Calibration Test Passed."); 
   else
    Serial.println("CCW Calibration Test Failed."); 
}
