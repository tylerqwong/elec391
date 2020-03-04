#define CCW 0
#define CW 1

int dir = CW;

//Input sensors of encoders
int A = 3;
int B = 2;

//ISR Variables
int curA;
int curB;
int oldA = 0;
int oldB = 0;

volatile int counter = 0;
volatile byte flag = 0;

//Motor state variables
int motor_state = 0;
int motor_cal_err = 0;

//Motor1 Logic
int motor1_1 = 8;
int motor1_2 = 9;
int motor1_3 = 10;

//Motor 2 Logic
int motor2_1 = 5;
int motor2_2 = 4;
int motor2_3 = 7;

//Logic Power Supply
int pin13 = 13;

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

//Enable pin for motor 2, used for PWM  
int pwm_pin = 6;

//motor pin 1 hotfix
int motorpin1 = 7;

//PID Variables and Constants
int run_pid = 0;
int counter_error;
int counter_setpoint;
int counter_prev_error = 0;

int PID_p;
int PID_i;
int PID_d;
int PID_total;
int kp = 1;
int ki;
int kd;


void setup() {
  //Initialize timer1
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 62412; //preload timer 65536-16MHz/256/20Hz
  TCCR1B |= (1 << CS12);  //256 prescaler
  TIMSK1 |= (1 << TOIE1);   //enable timer overflow interrupt
  interrupts();

  //Set encoder pins to input
  pinMode(A, INPUT);
  pinMode(B, INPUT);

  //Set Enable to on
  pinMode(pwm_pin, OUTPUT);
  analogWrite(pwm_pin, 255);
  
  
  //Logic Power Supply to PCB
  pinMode(pin13, OUTPUT);
  digitalWrite(pin13, HIGH);

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
}



void loop() {
  //Print position to serial monitor and clear the interrupt flag
 /*
  if (flag == 1) {
    Serial.print("Position: ");
    Serial.println(counter);
    flag = 0;
  }
*/
  //PID control
  if(run_pid == 1)
    pid();

  //Motor control and driver functions
  motor_control();
  motor_driver();
}

ISR(TIMER1_OVF_vect) //interrupt service routine
{
  run_pid = 1;
  TCNT1 = 62412;
}

void isrA() {
  //flag = 1;
  curA = digitalRead(A);
  curB = digitalRead(B);
  //add 1 to count for CW
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
    counter = counter - 400;

   if (counter < 0)
    counter = counter + 400;

    oldA = curA;
    oldB = curB;
}

void calc_param() {
  //DO THIS
}

void pid() {
  counter_error = counter_setpoint - counter;
  abs(counter_error);
  PID_p = kp * counter_error;
  /*
  float count_difference = counter_error - counter_prev_error;
  PID_d = kd * ((counter_error - counter_prev_error)/period);

  if (-3 < counter_error && counter_error < 3)
    PID_i = PID_i + (ki * distance_error);
  else
    PID_i = 0;
  */
  //PID_total = PID_p + PID_i + PID_d;
  PID_total = PID_p;
  PID_total = map(PID_total,0,399,0,255);
  analogWrite(pwm_pin, PID_total);

  //counter_prev_error = counter_error;
  
  //End of pid
  run_pid = 0;
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
    if ((counter < (CW_state_2 + 5)) && (counter > (CW_state_2 - 5)))
    {
      motor_state = 1;
      counter_setpoint = CW_state_1;
    } 
    else if ((counter < (CW_state_1 + 5)) && (counter > (CW_state_1 - 5)))
    {
      motor_state = 12;
      counter_setpoint = CW_state_12;
    }  
    else if ((counter < (CW_state_12 + 5)) && (counter > (CW_state_12 - 5)))
    {
      motor_state = 11;
      counter_setpoint = CW_state_11;
    }  
    else if ((counter < (CW_state_11 + 5)) && (counter > (CW_state_11 - 5)))
   {
      motor_state = 10;
      counter_setpoint = CW_state_10;
   }  
    else if ((counter < (CW_state_10 + 5)) && (counter > (CW_state_10 - 5)))
    {
      motor_state = 9;
      counter_setpoint = CW_state_9;
    }    
    else if ((counter < (CW_state_9 + 5)) && (counter > (CW_state_9 - 5)))
    {
      motor_state = 8;
      counter_setpoint = CW_state_8;
    }  
    else if ((counter < (CW_state_8 + 5)) && (counter > (CW_state_8 - 5)))
    {
      motor_state = 7;
      counter_setpoint = CW_state_7;
    }  
    else if ((counter < (CW_state_7 + 5)) && (counter > (CW_state_7 - 5)))
    {
      motor_state = 6;
      counter_setpoint = CW_state_6;
    }  
    else if ((counter < (CW_state_6 + 5)) && (counter > (CW_state_6 - 5)))
    {
      motor_state = 5;
      counter_setpoint = CW_state_5;
    }  
    else if ((counter < (CW_state_5 + 5)) && (counter > (CW_state_5 - 5)))
     {
      motor_state = 4;
      counter_setpoint = CW_state_4;
    }  
    else if ((counter < (CW_state_4 + 5)) && (counter > (CW_state_4 - 5)))
    {
      motor_state = 3;
      counter_setpoint = CW_state_3;
    }   
    else if ((counter < (CW_state_3 + 5)) && (counter > (CW_state_3 - 5)))
    {
      motor_state = 2;
      counter_setpoint = CW_state_2;
    }  
}


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
   if(motor_cal_err == 0)
    Serial.println("CCW Calibration Test Passed."); 
   else
    Serial.println("CCW Calibration Test Failed."); 
}
