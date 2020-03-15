#define CW 0
#define CCW 1

IntervalTimer pid1;
/*---------------------------------Motor One Parameters---------------------------------*/
int m1_dir;

//Input sensors of encoders
int m1_A = 0;
int m1_B = 1;

//ISR Variables
int m1_curA;
int m1_curB;
int m1_oldA = 0;
int m1_oldB = 0;

volatile int m1_counter = 0;
volatile int m1_rotation_counter = 0;

//Motor state variables
int m1_state = 0;
int m1_cal_err = 0;

//Motor Logic
int m1_1 = 17;
int m1_2 = 18;
int m1_3 = 19;
//Enable pin for motor, used for PWM  
int m1_pwm_pin = 23;

//Motor calibration position
//CW
int m1_CW_state_1;
int m1_CW_state_2;
int m1_CW_state_3;
int m1_CW_state_4;
int m1_CW_state_5;
int m1_CW_state_6;
int m1_CW_state_7;
int m1_CW_state_8;
int m1_CW_state_9;
int m1_CW_state_10;
int m1_CW_state_11;
int m1_CW_state_12;
//CCW
int m1_CCW_state_1;
int m1_CCW_state_2;
int m1_CCW_state_3;
int m1_CCW_state_4;
int m1_CCW_state_5;
int m1_CCW_state_6;
int m1_CCW_state_7;
int m1_CCW_state_8;
int m1_CCW_state_9;
int m1_CCW_state_10;
int m1_CCW_state_11;
int m1_CCW_state_12;


//PID Variables and Constants
int m1_run_pid;
int m1_error;
int m1_prev_error = 0;
int m1_PID_p;
int m1_PID_i;
int m1_PID_d;
int m1_PID_total;
float m1_kp = 1;
float m1_ki = 1;
float m1_kd = 1;
float m1_period = 0.0001; //10kHz

//Input Setpoints
int m1_rotation_counter_setpoint = 10;
int m1_counter_setpoint;
int m1_setpoint;



/*---------------------------------Motor Two Parameters---------------------------------*/
int m2_dir;

//Input sensors of encoders
int m2_A = 2;
int m2_B = 3;

//ISR Variables
int m2_curA;
int m2_curB;
int m2_oldA = 0;
int m2_oldB = 0;

volatile int m2_counter = 0;
volatile int m2_rotation_counter = 0;

//Motor state variables
int m2_state = 0;
int m2_cal_err = 0;

int m2_1 = 14;
int m2_2 = 15;
int m2_3 = 16;
//Enable pin for motor 2, used for PWM  
int m2_pwm_pin = 22;
//motor pin 2 hotfix (output of PLD)
int m2_hotfix = 21;

//Motor calibration position
//CW
int m2_CW_state_1;
int m2_CW_state_2;
int m2_CW_state_3;
int m2_CW_state_4;
int m2_CW_state_5;
int m2_CW_state_6;
int m2_CW_state_7;
int m2_CW_state_8;
int m2_CW_state_9;
int m2_CW_state_10;
int m2_CW_state_11;
int m2_CW_state_12;
//CCW
int m2_CCW_state_1;
int m2_CCW_state_2;
int m2_CCW_state_3;
int m2_CCW_state_4;
int m2_CCW_state_5;
int m2_CCW_state_6;
int m2_CCW_state_7;
int m2_CCW_state_8;
int m2_CCW_state_9;
int m2_CCW_state_10;
int m2_CCW_state_11;
int m2_CCW_state_12;
/*
//PID Variables and Constants
int m2_run_pid = 0;
int m2_error;
int m2_prev_error = 0;
int m2_PID_p;
int m2_PID_i;
int m2_PID_d;
int m2_PID_total;
float m2_kp = 1;
float m2_ki = 1;
float m2_kd = 1;
float m2_period = 0.0001; //10kHz

//Input Setpoints
int m2_rotation_counter_setpoint = 10;
int m2_counter_setpoint;
int m2_setpoint;
*/
void setup() {
 
  //Initialize pid1 timer for 10kHz
  pid1.begin(pid1_isr, 100);

  //Set encoder pins to input
  pinMode(m1_A, INPUT);
  pinMode(m1_B, INPUT);
  pinMode(m2_A, INPUT);
  pinMode(m2_B, INPUT);

  //Set Enable pins to on (max pwm)
  pinMode(m1_pwm_pin, OUTPUT);
  analogWrite(m1_pwm_pin, 255);
  pinMode(m2_pwm_pin, OUTPUT);
  analogWrite(m2_pwm_pin, 255);

  //Hotfix (Yellow wire from PLD output for Motor 2)
  pinMode(m2_hotfix, OUTPUT);
  digitalWrite(m2_hotfix, LOW);

  //Motor Logic Pins (J1)
  pinMode(m1_1, OUTPUT);
  pinMode(m1_2, OUTPUT);
  pinMode(m1_3, OUTPUT);
  pinMode(m2_1, OUTPUT);
  pinMode(m2_2, OUTPUT);
  pinMode(m2_3, OUTPUT); 

  Serial.begin(9600);

  //Encoder ISR reading for both rising and falling edges
  attachInterrupt(digitalPinToInterrupt(m1_A), m1_isr, CHANGE); 
  attachInterrupt(digitalPinToInterrupt(m1_B), m1_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(m2_A), m2_isr, CHANGE); 
  attachInterrupt(digitalPinToInterrupt(m2_B), m2_isr, CHANGE);  
  
  //Calibrate Motor
  while(1){
  motor_calibration();
  motor_calibration_test();
  if(m1_cal_err == 0 && m2_cal_err == 0)
    break;
  }
  m1_rotation_counter = 0;
  m2_rotation_counter = 0;
  m1_setpoint = m1_rotation_counter_setpoint*400;
  //m2_setpoint = m2_rotation_counter_setpoint*400;
}


void loop() {  
  //PID control
  if(m1_run_pid == 1)
    m1_pid();

  //if(m2_run_pid == 1)
    //m2_pid();  
  m2_dir = CCW;
  //Motor control and driver functions
  m1_control();
  m2_control();
  m1_driver();
  m2_driver();
}

/*-------------------------------Interrupt Service Routines------------------------------------*/
void pid1_isr() {
  m1_run_pid = 1;
}

void m1_isr() {
  m1_curA = digitalRead(m1_A);
  m1_curB = digitalRead(m1_B);
  //add 1 to count for CW or -1 for CCW
    if(m1_curA && !m1_curB && !m1_oldA && !m1_oldB) //CW
    {
      m1_counter++;
    }
    else if(m1_curA && m1_curB && m1_oldA && !m1_oldB) //CW
    {
      m1_counter++;
    }
    else if(!m1_curA && m1_curB && m1_oldA && m1_oldB) //CW
    {
      m1_counter++;
    }
    else if(!m1_curA && !m1_curB && !m1_oldA && m1_oldB) //CW
    {
      m1_counter++;
    }
    else if(!m1_curA && !m1_curB && m1_oldA && !m1_oldB) //CCW
    {
      m1_counter--;
    }
    else if(!m1_curA && m1_curB && !m1_oldA && !m1_oldB) //CCW
    {
      m1_counter--;
    }
    else if(m1_curA && m1_curB && !m1_oldA && m1_oldB) //CCW
    {
      m1_counter--;
    }
    else if(m1_curA && !m1_curB && m1_oldA && m1_oldB) //CCW
    {
      m1_counter--;
    }
    else
      m1_counter = m1_counter;

   if (m1_counter >= 400)
   {
    m1_counter = m1_counter - 400;
    m1_rotation_counter++;
   }

   if (m1_counter < 0)
   {
    m1_counter = m1_counter + 400;
    m1_rotation_counter--;
   }

    m1_oldA = m1_curA;
    m1_oldB = m1_curB;
}

void m2_isr() {
  m2_curA = digitalRead(m2_A);
  m2_curB = digitalRead(m2_B);
  //add 1 to count for CW or -1 for CCW
    if(m2_curA && !m2_curB && !m2_oldA && !m2_oldB) //CW
    {
      m2_counter++;
    }
    else if(m2_curA && m2_curB && m2_oldA && !m2_oldB) //CW
    {
      m2_counter++;
    }
    else if(!m2_curA && m2_curB && m2_oldA && m2_oldB) //CW
    {
      m2_counter++;
    }
    else if(!m2_curA && !m2_curB && !m2_oldA && m2_oldB) //CW
    {
      m2_counter++;
    }
    else if(!m2_curA && !m2_curB && m2_oldA && !m2_oldB) //CCW
    {
      m2_counter--;
    }
    else if(!m2_curA && m2_curB && !m2_oldA && !m2_oldB) //CCW
    {
      m2_counter--;
    }
    else if(m2_curA && m2_curB && !m2_oldA && m2_oldB) //CCW
    {
      m2_counter--;
    }
    else if(m2_curA && !m2_curB && m2_oldA && m2_oldB) //CCW
    {
      m2_counter--;
    }
    else
      m2_counter = m2_counter;

   if (m2_counter >= 400)
   {
    m2_counter = m2_counter - 400;
    m2_rotation_counter++;
   }

   if (m2_counter < 0)
   {
    m2_counter = m2_counter + 400;
    m2_rotation_counter--;
   }

    m2_oldA = m2_curA;
    m2_oldB = m2_curB;
}

/*---------------------------Control Algorithms---------------------------*/


void m1_pid() {
  m1_error = m1_setpoint - (m1_rotation_counter*400 + m1_counter);
  
  m1_PID_p = m1_kp * m1_error;
 /*
  PID_i = PID_i + ki*error*period;

  float difference = error - prev_error;
  PID_d = kd * ((difference)/period);
    
  PID_total = PID_p + PID_i + PID_d;
 */
  m1_PID_total = m1_PID_p;
  
  if(m1_PID_total > 0)
    m1_dir = CW;
  else
    m1_dir = CCW;
    
  //analogWrite(m1_pwm_pin, 255);
    
 // prev_error = error;  
  m1_run_pid = 0;
}
/*
void m2_pid() {
  error = setpoint - (rotation_counter*400 + counter);
  
  PID_p = kp * error;
 
  PID_i = PID_i + ki*error*period;

  float difference = error - prev_error;
  PID_d = kd * ((difference)/period);
    
  PID_total = PID_p + PID_i + PID_d;
   
  PID_total = PID_p + PID_i;
  
  if(PID_total > 0)
    dir = CW;
  else
    dir = CCW;

  PID_total = abs(PID_total);

  if (PID_total > 400)
    analogWrite(pwm_pin, 255);
  else 
  {
    PID_total = map(PID_total,0,400,180,255);
    analogWrite(pwm_pin, PID_total);
  }
  
 // prev_error = error;  
 // run_pid = 0;
}

*/
void m1_driver() {
    if(m1_state == 0) 
    {
      digitalWrite(m1_1, LOW);
      digitalWrite(m1_2, LOW);
      digitalWrite(m1_3, LOW);
    }
    else if ((m1_state == 1)||(m1_state == 7)) 
    {
      digitalWrite(m1_1, LOW);
      digitalWrite(m1_2, LOW);
      digitalWrite(m1_3, HIGH);
    }    
    else if ((m1_state == 2)||(m1_state == 8)) 
    {
      digitalWrite(m1_1, LOW);
      digitalWrite(m1_2, HIGH);
      digitalWrite(m1_3, LOW);
    }
    else if ((m1_state == 3)||(m1_state == 9)) 
    {
      digitalWrite(m1_1, LOW);
      digitalWrite(m1_2, HIGH);
      digitalWrite(m1_3, HIGH);
    }
    else if ((m1_state == 4)||(m1_state == 10)) 
    {
      digitalWrite(m1_1, HIGH);
      digitalWrite(m1_2, LOW);
      digitalWrite(m1_3, LOW);
    }
    else if ((m1_state == 5)||(m1_state == 11)) 
    {
      digitalWrite(m1_1, HIGH);
      digitalWrite(m1_2, LOW);
      digitalWrite(m1_3, HIGH);
    }
    else if ((m1_state == 6)||(m1_state == 12)) 
    {
      digitalWrite(m1_1, HIGH);
      digitalWrite(m1_2, HIGH);
      digitalWrite(m1_3, LOW);
    }
    else 
    {
      digitalWrite(m1_1, LOW);
      digitalWrite(m1_2, LOW);
      digitalWrite(m1_3, LOW);
    }
}

void m2_driver() {    
    if(m2_state == 0) 
    {
      digitalWrite(m2_1, LOW);
      digitalWrite(m2_2, LOW);
      digitalWrite(m2_3, LOW);
      digitalWrite(m2_hotfix, LOW);
    }
    else if ((m2_state == 1)||(m2_state == 7)) 
    {
      digitalWrite(m2_1, LOW);
      digitalWrite(m2_2, LOW);
      digitalWrite(m2_3, HIGH);
      digitalWrite(m2_hotfix, HIGH);
    }    
    else if ((m2_state == 2)||(m2_state == 8)) 
    {
      digitalWrite(m2_1, LOW);
      digitalWrite(m2_2, HIGH);
      digitalWrite(m2_3, LOW);
      digitalWrite(m2_hotfix, LOW);
    }
    else if ((m2_state == 3)||(m2_state == 9)) 
    {
      digitalWrite(m2_1, LOW);
      digitalWrite(m2_2, HIGH);
      digitalWrite(m2_3, HIGH);
      digitalWrite(m2_hotfix, LOW);
    }
    else if ((m2_state == 4)||(m2_state == 10)) 
    {
      digitalWrite(m2_1, HIGH);
      digitalWrite(m2_2, LOW);
      digitalWrite(m2_3, LOW);
      digitalWrite(m2_hotfix, LOW);
    }
    else if ((m2_state == 5)||(m2_state == 11)) 
    {
      digitalWrite(m2_1, HIGH);
      digitalWrite(m2_2, LOW);
      digitalWrite(m2_3, HIGH);
      digitalWrite(m2_hotfix, LOW);
    }
    else if ((m2_state == 6)||(m2_state == 12)) 
    {
      digitalWrite(m2_1, HIGH);
      digitalWrite(m2_2, HIGH);
      digitalWrite(m2_3, LOW);
      digitalWrite(m2_hotfix, HIGH);
    }
    else 
    {
      digitalWrite(m2_1, LOW);
      digitalWrite(m2_2, LOW);
      digitalWrite(m2_3, LOW);
      digitalWrite(m2_hotfix, LOW);
    }
}

void m1_control() {
  //CW Direction
  if(m1_dir == CW) 
  {
    if ((m1_counter < (m1_CW_state_12 + 5)) && (m1_counter > (m1_CW_state_12 - 5)))
      m1_state = 1;
       
    else if ((m1_counter < (m1_CW_state_1 + 5)) && (m1_counter > (m1_CW_state_1 - 5)))
      m1_state = 2; 
      
    else if ((m1_counter < (m1_CW_state_2 + 5)) && (m1_counter > (m1_CW_state_2 - 5)))
      m1_state = 3; 

    else if ((m1_counter < (m1_CW_state_3 + 5)) && (m1_counter > (m1_CW_state_3 - 5)))
      m1_state = 4;

    else if ((m1_counter < (m1_CW_state_4 + 5)) && (m1_counter > (m1_CW_state_4 - 5)))
      m1_state = 5;

    else if ((m1_counter < (m1_CW_state_5 + 5)) && (m1_counter > (m1_CW_state_5 - 5)))
      m1_state = 6;

    else if ((m1_counter < (m1_CW_state_6 + 5)) && (m1_counter > (m1_CW_state_6 - 5)))
      m1_state = 7;

    else if ((m1_counter < (m1_CW_state_7 + 5)) && (m1_counter > (m1_CW_state_7 - 5)))
      m1_state = 8;

    else if ((m1_counter < (m1_CW_state_8 + 5)) && (m1_counter > (m1_CW_state_8 - 5)))
      m1_state = 9;

    else if ((m1_counter < (m1_CW_state_9 + 5)) && (m1_counter > (m1_CW_state_9 - 5)))
      m1_state = 10;

    else if ((m1_counter < (m1_CW_state_10 + 5)) && (m1_counter > (m1_CW_state_10 - 5)))
      m1_state = 11;

    else if ((m1_counter < (m1_CW_state_11 + 5)) && (m1_counter > (m1_CW_state_11 - 5)))
      m1_state = 12; 
   
    else
      m1_state = 0; 
  }
  
  else if(m1_dir == CCW) 
  {
    if ((m1_counter < (m1_CCW_state_1 + 5)) && (m1_counter > (m1_CCW_state_1 - 5)))
      m1_state = 12;
      
    else if ((m1_counter < (m1_CCW_state_12 + 5)) && (m1_counter > (m1_CCW_state_12 - 5)))
      m1_state = 11;

    else if ((m1_counter < (m1_CCW_state_11 + 5)) && (m1_counter > (m1_CCW_state_11 - 5)))
      m1_state = 10;

    else if ((m1_counter < (m1_CCW_state_10 + 5)) && (m1_counter > (m1_CCW_state_10 - 5)))
      m1_state = 9;

    else if ((m1_counter < (m1_CCW_state_9 + 5)) && (m1_counter > (m1_CCW_state_9 - 5)))
      m1_state = 8;

    else if ((m1_counter < (m1_CCW_state_8 + 5)) && (m1_counter > (m1_CCW_state_8 - 5)))
      m1_state = 7;

    else if ((m1_counter < (m1_CCW_state_7 + 5)) && (m1_counter > (m1_CCW_state_7 - 5)))
      m1_state = 6;

    else if ((m1_counter < (m1_CCW_state_6 + 5)) && (m1_counter > (m1_CCW_state_6 - 5)))
      m1_state = 5;

    else if ((m1_counter < (m1_CCW_state_5 + 5)) && (m1_counter > (m1_CCW_state_5 - 5)))
      m1_state = 4;

    else if ((m1_counter < (m1_CCW_state_4 + 5)) && (m1_counter > (m1_CCW_state_4 - 5)))
      m1_state = 3;

    else if ((m1_counter < (m1_CCW_state_3 + 5)) && (m1_counter > (m1_CCW_state_3 - 5)))
      m1_state = 2;

    else if ((m1_counter < (m1_CCW_state_2 + 5)) && (m1_counter > (m1_CCW_state_2 - 5)))
      m1_state = 1;

    else
      m1_state = 0;
  }
  
  else
    m1_state = 0;
}

void m2_control() {
  //CW Direction
  if(m2_dir == CW) 
  {
    if ((m2_counter < (m2_CW_state_12 + 5)) && (m2_counter > (m2_CW_state_12 - 5)))
      m2_state = 1;
       
    else if ((m2_counter < (m2_CW_state_1 + 5)) && (m2_counter > (m2_CW_state_1 - 5)))
      m2_state = 2;
       
    else if ((m2_counter < (m2_CW_state_2 + 5)) && (m2_counter > (m2_CW_state_2 - 5)))
      m2_state = 3;
       
    else if ((m2_counter < (m2_CW_state_3 + 5)) && (m2_counter > (m2_CW_state_3 - 5)))
      m2_state = 4;
       
    else if ((m2_counter < (m2_CW_state_4 + 5)) && (m2_counter > (m2_CW_state_4 - 5)))
      m2_state = 5;
       
    else if ((m2_counter < (m2_CW_state_5 + 5)) && (m2_counter > (m2_CW_state_5 - 5)))
      m2_state = 6;
       
    else if ((m2_counter < (m2_CW_state_6 + 5)) && (m2_counter > (m2_CW_state_6 - 5)))
      m2_state = 7;
       
    else if ((m2_counter < (m2_CW_state_7 + 5)) && (m2_counter > (m2_CW_state_7 - 5)))
      m2_state = 8;
       
    else if ((m2_counter < (m2_CW_state_8 + 5)) && (m2_counter > (m2_CW_state_8 - 5)))
      m2_state = 9;
       
    else if ((m2_counter < (m2_CW_state_9 + 5)) && (m2_counter > (m2_CW_state_9 - 5)))
      m2_state = 10;
       
    else if ((m2_counter < (m2_CW_state_10 + 5)) && (m2_counter > (m2_CW_state_10 - 5)))
      m2_state = 11;
       
    else if ((m2_counter < (m2_CW_state_11 + 5)) && (m2_counter > (m2_CW_state_11 - 5)))
      m2_state = 12; 
   
    else
      m2_state = 0; 
  }
  
  else if(m2_dir == CCW) 
  {
    if ((m2_counter < (m2_CCW_state_1 + 5)) && (m2_counter > (m2_CCW_state_1 - 5)))
      m2_state = 12;

    else if ((m2_counter < (m2_CCW_state_12 + 5)) && (m2_counter > (m2_CCW_state_12 - 5)))
      m2_state = 11;

    else if ((m2_counter < (m2_CCW_state_11 + 5)) && (m2_counter > (m2_CCW_state_11 - 5)))
      m2_state = 10;

    else if ((m2_counter < (m2_CCW_state_10 + 5)) && (m2_counter > (m2_CCW_state_10 - 5)))
      m2_state = 9;

    else if ((m2_counter < (m2_CCW_state_9 + 5)) && (m2_counter > (m2_CCW_state_9 - 5)))
      m2_state = 8;

    else if ((m2_counter < (m2_CCW_state_8 + 5)) && (m2_counter > (m2_CCW_state_8 - 5)))
      m2_state = 7;

    else if ((m2_counter < (m2_CCW_state_7 + 5)) && (m2_counter > (m2_CCW_state_7 - 5)))
      m2_state = 6;

    else if ((m2_counter < (m2_CCW_state_6 + 5)) && (m2_counter > (m2_CCW_state_6 - 5)))
      m2_state = 5;

    else if ((m2_counter < (m2_CCW_state_5 + 5)) && (m2_counter > (m2_CCW_state_5 - 5)))
      m2_state = 4;

    else if ((m2_counter < (m2_CCW_state_4 + 5)) && (m2_counter > (m2_CCW_state_4 - 5)))
      m2_state = 3;

    else if ((m2_counter < (m2_CCW_state_3 + 5)) && (m2_counter > (m2_CCW_state_3 - 5)))
      m2_state = 2;

    else if ((m2_counter < (m2_CCW_state_2 + 5)) && (m2_counter > (m2_CCW_state_2 - 5)))
      m2_state = 1;

    else
      m2_state = 0;
  }
  
  else
    m2_state = 0;
}


/*---------------------------------------------Calibration Setup--------------------------------------------*/

void motor_calibration() {
  Serial.println("Press 1 to calibrate motors");
  while(1){
    if(Serial.parseInt() == 1){
      Serial.println("Calibrating motors...");
      break;
    }
   } 
   
//Calibrate by recording each state window (CW)
   Serial.println("Calibrating motors CW");
   m1_state = 1;
   m2_state = 1;
   m1_driver();
   m2_driver();
   delay(1000);
   m1_counter = 0;
   m2_counter = 0;
   m1_CW_state_1 = m1_counter;
   m2_CW_state_1 = m2_counter;
   Serial.print("Motor 1 Reading: ");
   Serial.println(m1_CW_state_1);
   Serial.print("Motor 2 Reading: ");
   Serial.println(m2_CW_state_1);
   
   m1_state = 2;
   m2_state = 2;
   m1_driver();
   m2_driver();
   delay(1000);
   m1_CW_state_2 = m1_counter;
   m2_CW_state_2 = m2_counter;
   Serial.print("Motor 1 Reading: ");
   Serial.println(m1_CW_state_2);
   Serial.print("Motor 2 Reading: ");
   Serial.println(m2_CW_state_2);
   
   m1_state = 3;
   m2_state = 3;
   m1_driver();
   m2_driver();
   delay(1000);
   m1_CW_state_3 = m1_counter;
   m2_CW_state_3 = m2_counter;
   Serial.print("Motor 1 Reading: ");
   Serial.println(m1_CW_state_3);
   Serial.print("Motor 2 Reading: ");
   Serial.println(m2_CW_state_3);
   
   m1_state = 4;
   m2_state = 4;
   m1_driver();
   m2_driver();
   delay(1000);
   m1_CW_state_4 = m1_counter;
   m2_CW_state_4 = m2_counter;
   Serial.print("Motor 1 Reading: ");
   Serial.println(m1_CW_state_4);
   Serial.print("Motor 2 Reading: ");
   Serial.println(m2_CW_state_4);
   
   m1_state = 5;
   m2_state = 5;
   m1_driver();
   m2_driver();
   delay(1000);
   m1_CW_state_5 = m1_counter;
   m2_CW_state_5 = m2_counter;
   Serial.print("Motor 1 Reading: ");
   Serial.println(m1_CW_state_5);
   Serial.print("Motor 2 Reading: ");
   Serial.println(m2_CW_state_5);
   
   m1_state = 6;
   m2_state = 6;
   m1_driver();
   m2_driver();
   delay(1000);
   m1_CW_state_6 = m1_counter;
   m2_CW_state_6 = m2_counter;
   Serial.print("Motor 1 Reading: ");
   Serial.println(m1_CW_state_6);
   Serial.print("Motor 2 Reading: ");
   Serial.println(m2_CW_state_6);
   
   m1_state = 7;
   m2_state = 7;
   m1_driver();
   m2_driver();
   delay(1000);
   m1_CW_state_7 = m1_counter;
   m2_CW_state_7 = m2_counter;
   Serial.print("Motor 1 Reading: ");
   Serial.println(m1_CW_state_7);
   Serial.print("Motor 2 Reading: ");
   Serial.println(m2_CW_state_7);
   
   m1_state = 8;
   m2_state = 8;
   m1_driver();
   m2_driver();
   delay(1000);
   m1_CW_state_8 = m1_counter;
   m2_CW_state_8 = m2_counter;
   Serial.print("Motor 1 Reading: ");
   Serial.println(m1_CW_state_8);
   Serial.print("Motor 2 Reading: ");
   Serial.println(m2_CW_state_8);
   
   m1_state = 9;
   m2_state = 9;
   m1_driver();
   m2_driver();
   delay(1000);
   m1_CW_state_9 = m1_counter;
   m2_CW_state_9 = m2_counter;
   Serial.print("Motor 1 Reading: ");
   Serial.println(m1_CW_state_9);
   Serial.print("Motor 2 Reading: ");
   Serial.println(m2_CW_state_9);
   
   m1_state = 10;
   m2_state = 10;
   m1_driver();
   m2_driver();
   delay(1000);
   m1_CW_state_10 = m1_counter;
   m2_CW_state_10 = m2_counter;
   Serial.print("Motor 1 Reading: ");
   Serial.println(m1_CW_state_10);
   Serial.print("Motor 2 Reading: ");
   Serial.println(m2_CW_state_10);
   
   m1_state = 11;
   m2_state = 11;
   m1_driver();
   m2_driver();
   delay(1000);
   m1_CW_state_11 = m1_counter;
   m2_CW_state_11 = m2_counter;
   Serial.print("Motor 1 Reading: ");
   Serial.println(m1_CW_state_11);
   Serial.print("Motor 2 Reading: ");
   Serial.println(m2_CW_state_11);
   
   m1_state = 12;
   m2_state = 12;
   m1_driver();
   m2_driver();
   delay(1000);
   m1_CW_state_12 = m1_counter;
   m2_CW_state_12 = m2_counter;
   Serial.print("Motor 1 Reading: ");
   Serial.println(m1_CW_state_12);
   Serial.print("Motor 2 Reading: ");
   Serial.println(m2_CW_state_12);
  

//Calibrate by recording each state window (CCW)
   Serial.println("Calibrating motors CCW");
   m1_state = 1;
   m2_state = 1;
   m1_driver();
   m2_driver();
   delay(1000);
   m1_CCW_state_1 = m1_counter;
   m2_CCW_state_1 = m2_counter;
   Serial.print("Motor 1 Reading: ");
   Serial.println(m1_CCW_state_1);
   Serial.print("Motor 2 Reading: ");
   Serial.println(m2_CCW_state_1);
   
   m1_state = 12;
   m2_state = 12;
   m1_driver();
   m2_driver();
   delay(1000);
   m1_CCW_state_12 = m1_counter;
   m2_CCW_state_12 = m2_counter;
   Serial.print("Motor 1 Reading: ");
   Serial.println(m1_CCW_state_12);
   Serial.print("Motor 2 Reading: ");
   Serial.println(m2_CCW_state_12);
   
   m1_state = 11;
   m2_state = 11;
   m1_driver();
   m2_driver();
   delay(1000);
   m1_CCW_state_11 = m1_counter;
   m2_CCW_state_11 = m2_counter;
   Serial.print("Motor 1 Reading: ");
   Serial.println(m1_CCW_state_11);
   Serial.print("Motor 2 Reading: ");
   Serial.println(m2_CCW_state_11);
   
   m1_state = 10;
   m2_state = 10;
   m1_driver();
   m2_driver();
   delay(1000);
   m1_CCW_state_10 = m1_counter;
   m2_CCW_state_10 = m2_counter;
   Serial.print("Motor 1 Reading: ");
   Serial.println(m1_CCW_state_10);
   Serial.print("Motor 2 Reading: ");
   Serial.println(m2_CCW_state_10);
   
   m1_state = 9;
   m2_state = 9;
   m1_driver();
   m2_driver();
   delay(1000);
   m1_CCW_state_9 = m1_counter;
   m2_CCW_state_9 = m2_counter;
   Serial.print("Motor 1 Reading: ");
   Serial.println(m1_CCW_state_9);
   Serial.print("Motor 2 Reading: ");
   Serial.println(m2_CCW_state_9);
   
   m1_state = 8;
   m2_state = 8;
   m1_driver();
   m2_driver();
   delay(1000);
   m1_CCW_state_8 = m1_counter;
   m2_CCW_state_8 = m2_counter;
   Serial.print("Motor 1 Reading: ");
   Serial.println(m1_CCW_state_8);
   Serial.print("Motor 2 Reading: ");
   Serial.println(m2_CCW_state_8);
   
   m1_state = 7;
   m2_state = 7;
   m1_driver();
   m2_driver();
   delay(1000);
   m1_CCW_state_7 = m1_counter;
   m2_CCW_state_7 = m2_counter;
   Serial.print("Motor 1 Reading: ");
   Serial.println(m1_CCW_state_7);
   Serial.print("Motor 2 Reading: ");
   Serial.println(m2_CCW_state_7);
   
   m1_state = 6;
   m2_state = 6;
   m1_driver();
   m2_driver();
   delay(1000);
   m1_CCW_state_6 = m1_counter;
   m2_CCW_state_6 = m2_counter;
   Serial.print("Motor 1 Reading: ");
   Serial.println(m1_CCW_state_6);
   Serial.print("Motor 2 Reading: ");
   Serial.println(m2_CCW_state_6);
   
   m1_state = 5;
   m2_state = 5;
   m1_driver();
   m2_driver();
   delay(1000);
   m1_CCW_state_5 = m1_counter;
   m2_CCW_state_5 = m2_counter;
   Serial.print("Motor 1 Reading: ");
   Serial.println(m1_CCW_state_5);
   Serial.print("Motor 2 Reading: ");
   Serial.println(m2_CCW_state_5);
   
   m1_state = 4;
   m2_state = 4;
   m1_driver();
   m2_driver();
   delay(1000);
   m1_CCW_state_4 = m1_counter;
   m2_CCW_state_4 = m2_counter;
   Serial.print("Motor 1 Reading: ");
   Serial.println(m1_CCW_state_4);
   Serial.print("Motor 2 Reading: ");
   Serial.println(m2_CCW_state_4);
   
   m1_state = 3;
   m2_state = 3;
   m1_driver();
   m2_driver();
   delay(1000);
   m1_CCW_state_3 = m1_counter;
   m2_CCW_state_3 = m2_counter;
   Serial.print("Motor 1 Reading: ");
   Serial.println(m1_CCW_state_3);
   Serial.print("Motor 2 Reading: ");
   Serial.println(m2_CCW_state_3);
   
   m1_state = 2;
   m2_state = 2;
   m1_driver();
   m2_driver();
   delay(1000);
   m1_CCW_state_2 = m1_counter;
   m2_CCW_state_2 = m2_counter;
   Serial.print("Motor 1 Reading: ");
   Serial.println(m1_CCW_state_2);
   Serial.print("Motor 2 Reading: ");
   Serial.println(m2_CCW_state_2);
   
   m1_state = 1;
   m2_state = 1;
   m1_driver();
   m2_driver();
   delay(1000);
   Serial.println("Motor calibrated");  
}
/*-----------------------------------------------STILL NEED TO MAKE THIS DOUBLE MOTOR-----------------------------------------*/
void motor_calibration_test() {
//Test Motor 1 CCW, Motor 2 CW
   m1_state = 1;
   m2_state = 1;
   m1_driver();
   m2_driver();
   delay(1000);
   Serial.print("Motor 1 Reading: ");
   Serial.println(m1_counter);
   Serial.print("Motor 2 Reading: ");
   Serial.println(m2_counter);
   if(m1_CW_state_1 > 390)
    m1_CW_state_1 = m1_CW_state_1 - 400;
   if(m1_counter > 390)
    m1_counter = m1_counter - 400;
   if(m2_CW_state_1 > 390)
    m2_CW_state_1 = m2_CW_state_1 - 400;
   if(m2_counter > 390)
    m2_counter = m2_counter - 400;
   if ((m1_counter > (m1_CW_state_1 + 5)) || (m1_counter < (m1_CW_state_1 - 5)))
    m1_cal_err = 1;
   if ((m2_counter > (m2_CW_state_1 + 5)) || (m2_counter < (m2_CW_state_1 - 5)))
    m2_cal_err = 1;

   m1_state = 2;
   m2_state = 2;
   m1_driver();
   m2_driver();
   delay(1000);
   Serial.print("Motor 1 Reading: ");
   Serial.println(m1_counter);
   Serial.print("Motor 2 Reading: ");
   Serial.println(m2_counter);
   if ((m1_counter > (m1_CW_state_2 + 5)) || (m1_counter < (m1_CW_state_2 - 5)))
    m1_cal_err = 1;
   if ((m2_counter > (m2_CW_state_2 + 5)) || (m2_counter < (m2_CW_state_2 - 5)))
    m2_cal_err = 1;

   m1_state = 3;
   m2_state = 3;
   m1_driver();
   m2_driver();
   delay(1000);
   Serial.print("Motor 1 Reading: ");
   Serial.println(m1_counter);
   Serial.print("Motor 2 Reading: ");
   Serial.println(m2_counter);
   if ((m1_counter > (m1_CW_state_3 + 5)) || (m1_counter < (m1_CW_state_3 - 5)))
    m1_cal_err = 1;
   if ((m2_counter > (m2_CW_state_3 + 5)) || (m2_counter < (m2_CW_state_3 - 5)))
    m2_cal_err = 1;

   m1_state = 4;
   m2_state = 4;
   m1_driver();
   m2_driver();
   delay(1000);
   Serial.print("Motor 1 Reading: ");
   Serial.println(m1_counter);
   Serial.print("Motor 2 Reading: ");
   Serial.println(m2_counter);
   if ((m1_counter > (m1_CW_state_4 + 5)) || (m1_counter < (m1_CW_state_4 - 5)))
    m1_cal_err = 1;
   if ((m2_counter > (m2_CW_state_4 + 5)) || (m2_counter < (m2_CW_state_4 - 5)))
    m2_cal_err = 1;

   m1_state = 5;
   m2_state = 5;
   m1_driver();
   m2_driver();
   delay(1000);
   Serial.print("Motor 1 Reading: ");
   Serial.println(m1_counter);
   Serial.print("Motor 2 Reading: ");
   Serial.println(m2_counter);
   if ((m1_counter > (m1_CW_state_5 + 5)) || (m1_counter < (m1_CW_state_5 - 5)))
    m1_cal_err = 1;
   if ((m2_counter > (m2_CW_state_5 + 5)) || (m2_counter < (m2_CW_state_5 - 5)))
    m2_cal_err = 1;

   m1_state = 6;
   m2_state = 6;
   m1_driver();
   m2_driver();
   delay(1000);
   Serial.print("Motor 1 Reading: ");
   Serial.println(m1_counter);
   Serial.print("Motor 2 Reading: ");
   Serial.println(m2_counter);
   if ((m1_counter > (m1_CW_state_6 + 5)) || (m1_counter < (m1_CW_state_6 - 5)))
    m1_cal_err = 1;
   if ((m2_counter > (m2_CW_state_6 + 5)) || (m2_counter < (m2_CW_state_6 - 5)))
    m2_cal_err = 1;

   m1_state = 7;
   m2_state = 7;
   m1_driver();
   m2_driver();
   delay(1000);
   Serial.print("Motor 1 Reading: ");
   Serial.println(m1_counter);
   Serial.print("Motor 2 Reading: ");
   Serial.println(m2_counter);
   if ((m1_counter > (m1_CW_state_7 + 5)) || (m1_counter < (m1_CW_state_7 - 5)))
    m1_cal_err = 1;
   if ((m2_counter > (m2_CW_state_7 + 5)) || (m2_counter < (m2_CW_state_7 - 5)))
    m2_cal_err = 1;
    
   m1_state = 8;
   m2_state = 8;
   m1_driver();
   m2_driver();
   delay(1000);
   Serial.print("Motor 1 Reading: ");
   Serial.println(m1_counter);
   Serial.print("Motor 2 Reading: ");
   Serial.println(m2_counter);
   if ((m1_counter > (m1_CW_state_8 + 5)) || (m1_counter < (m1_CW_state_8 - 5)))
    m1_cal_err = 1;
   if ((m2_counter > (m2_CW_state_8 + 5)) || (m2_counter < (m2_CW_state_8 - 5)))
    m2_cal_err = 1;
    
   m1_state = 9;
   m2_state = 9;
   m1_driver();
   m2_driver();
   delay(1000);
   Serial.print("Motor 1 Reading: ");
   Serial.println(m1_counter);
   Serial.print("Motor 2 Reading: ");
   Serial.println(m2_counter);
   if ((m1_counter > (m1_CW_state_9 + 5)) || (m1_counter < (m1_CW_state_9 - 5)))
    m1_cal_err = 1;
   if ((m2_counter > (m2_CW_state_9 + 5)) || (m2_counter < (m2_CW_state_9 - 5)))
    m2_cal_err = 1;
    
   m1_state = 10;
   m2_state = 10;
   m1_driver();
   m2_driver();
   delay(1000);
   Serial.print("Motor 1 Reading: ");
   Serial.println(m1_counter);
   Serial.print("Motor 2 Reading: ");
   Serial.println(m2_counter);
   if ((m1_counter > (m1_CW_state_10 + 5)) || (m1_counter < (m1_CW_state_10 - 5)))
    m1_cal_err = 1;
   if ((m2_counter > (m2_CW_state_10 + 5)) || (m2_counter < (m2_CW_state_10 - 5)))
    m2_cal_err = 1;
    
   m1_state = 11;
   m2_state = 11;
   m1_driver();
   m2_driver();
   delay(1000);
   Serial.print("Motor 1 Reading: ");
   Serial.println(m1_counter);
   Serial.print("Motor 2 Reading: ");
   Serial.println(m2_counter);
   if ((m1_counter > (m1_CW_state_11 + 5)) || (m1_counter < (m1_CW_state_11 - 5)))
    m1_cal_err = 1;
   if ((m2_counter > (m2_CW_state_11 + 5)) || (m2_counter < (m2_CW_state_11 - 5)))
    m2_cal_err = 1;
    
   m1_state = 12;
   m2_state = 12;
   m1_driver();
   m2_driver();
   delay(1000);
   Serial.print("Motor 1 Reading: ");
   Serial.println(m1_counter);
   Serial.print("Motor 2 Reading: ");
   Serial.println(m2_counter);
   if ((m1_counter > (m1_CW_state_12 + 5)) || (m1_counter < (m1_CW_state_12 - 5)))
    m1_cal_err = 1;
   if ((m2_counter > (m2_CW_state_12 + 5)) || (m2_counter < (m2_CW_state_12 - 5)))
    m2_cal_err = 1;
    
  if(m1_cal_err == 0 && m2_cal_err == 0)
   Serial.println("CW Calibration Test Passed."); 
  else if (m1_cal_err)
  {
    Serial.println("Motor One CW Calibration Test Failed."); 
    return;
  }
  else
  {
    Serial.println("Motor Two CW Calibration Test Failed."); 
    return;
  }

//Test CCW
   m1_state = 1;
   m2_state = 1;
   m1_driver();
   m2_driver();
   delay(1000);
   Serial.print("Motor 1 Reading: ");
   Serial.println(m1_counter);
   Serial.print("Motor 2 Reading: ");
   Serial.println(m2_counter);
   if(m1_CCW_state_1 > 390)
    m1_CCW_state_1 = m1_CCW_state_1 - 400;
   if(m1_counter > 390)
    m1_counter = m1_counter - 400;
   if(m2_CCW_state_1 > 390)
    m2_CCW_state_1 = m2_CCW_state_1 - 400;
   if(m2_counter > 390)
    m2_counter = m2_counter - 400;
    
   if ((m1_counter > (m1_CCW_state_1+5)) || (m1_counter < (m1_CCW_state_1-5)))
    m1_cal_err = 1;
   if ((m2_counter > (m2_CCW_state_1+5)) || (m2_counter < (m2_CCW_state_1-5)))
    m2_cal_err = 1;

   m1_state = 12;
   m2_state = 12;
   m1_driver();
   m2_driver();
   delay(1000);
   Serial.print("Motor 1 Reading: ");
   Serial.println(m1_counter);
   Serial.print("Motor 2 Reading: ");
   Serial.println(m2_counter);
   if ((m1_counter > (m1_CCW_state_12+5)) || (m1_counter < (m1_CCW_state_12-5)))
    m1_cal_err = 1;
   if ((m2_counter > (m2_CCW_state_12+5)) || (m2_counter < (m2_CCW_state_12-5)))
    m2_cal_err = 1;

   m1_state = 11;
   m2_state = 11;
   m1_driver();
   m2_driver();
   delay(1000);
   Serial.print("Motor 1 Reading: ");
   Serial.println(m1_counter);
   Serial.print("Motor 2 Reading: ");
   Serial.println(m2_counter);
   if ((m1_counter > (m1_CCW_state_11+5)) || (m1_counter < (m1_CCW_state_11-5)))
    m1_cal_err = 1;
   if ((m2_counter > (m2_CCW_state_11+5)) || (m2_counter < (m2_CCW_state_11-5)))
    m2_cal_err = 1;

   m1_state = 10;
   m2_state = 10;
   m1_driver();
   m2_driver();
   delay(1000);
   Serial.print("Motor 1 Reading: ");
   Serial.println(m1_counter);
   Serial.print("Motor 2 Reading: ");
   Serial.println(m2_counter);
   if ((m1_counter > (m1_CCW_state_10+5)) || (m1_counter < (m1_CCW_state_10-5)))
    m1_cal_err = 1;
   if ((m2_counter > (m2_CCW_state_10+5)) || (m2_counter < (m2_CCW_state_10-5)))
    m2_cal_err = 1;

   m1_state = 9;
   m2_state = 9;
   m1_driver();
   m2_driver();
   delay(1000);
   Serial.print("Motor 1 Reading: ");
   Serial.println(m1_counter);
   Serial.print("Motor 2 Reading: ");
   Serial.println(m2_counter);
   if ((m1_counter > (m1_CCW_state_9+5)) || (m1_counter < (m1_CCW_state_9-5)))
    m1_cal_err = 1;
   if ((m2_counter > (m2_CCW_state_9+5)) || (m2_counter < (m2_CCW_state_9-5)))
    m2_cal_err = 1;

   m1_state = 8;
   m2_state = 8;
   m1_driver();
   m2_driver();
   delay(1000);
   Serial.print("Motor 1 Reading: ");
   Serial.println(m1_counter);
   Serial.print("Motor 2 Reading: ");
   Serial.println(m2_counter);
   if ((m1_counter > (m1_CCW_state_8+5)) || (m1_counter < (m1_CCW_state_8-5)))
    m1_cal_err = 1;
   if ((m2_counter > (m2_CCW_state_8+5)) || (m2_counter < (m2_CCW_state_8-5)))
    m2_cal_err = 1;

   m1_state = 7;
   m2_state = 7;
   m1_driver();
   m2_driver();
   delay(1000);
   Serial.print("Motor 1 Reading: ");
   Serial.println(m1_counter);
   Serial.print("Motor 2 Reading: ");
   Serial.println(m2_counter);
   if ((m1_counter > (m1_CCW_state_7+5)) || (m1_counter < (m1_CCW_state_7-5)))
    m1_cal_err = 1;
   if ((m2_counter > (m2_CCW_state_7+5)) || (m2_counter < (m2_CCW_state_7-5)))
    m2_cal_err = 1;
    
   m1_state = 6;
   m2_state = 6;
   m1_driver();
   m2_driver();
   delay(1000);
   Serial.print("Motor 1 Reading: ");
   Serial.println(m1_counter);
   Serial.print("Motor 2 Reading: ");
   Serial.println(m2_counter);
   if ((m1_counter > (m1_CCW_state_6+5)) || (m1_counter < (m1_CCW_state_6-5)))
    m1_cal_err = 1;
   if ((m2_counter > (m2_CCW_state_6+5)) || (m2_counter < (m2_CCW_state_6-5)))
    m2_cal_err = 1;
    
   m1_state = 5;
   m2_state = 5;
   m1_driver();
   m2_driver();
   delay(1000);
   Serial.print("Motor 1 Reading: ");
   Serial.println(m1_counter);
   Serial.print("Motor 2 Reading: ");
   Serial.println(m2_counter);
   if ((m1_counter > (m1_CCW_state_5+5)) || (m1_counter < (m1_CCW_state_5-5)))
    m1_cal_err = 1;
   if ((m2_counter > (m2_CCW_state_5+5)) || (m2_counter < (m2_CCW_state_5-5)))
    m2_cal_err = 1;
    
   m1_state = 4;
   m2_state = 4;
   m1_driver();
   m2_driver();
   delay(1000);
   Serial.print("Motor 1 Reading: ");
   Serial.println(m1_counter);
   Serial.print("Motor 2 Reading: ");
   Serial.println(m2_counter);
   if ((m1_counter > (m1_CCW_state_4+5)) || (m1_counter < (m1_CCW_state_4-5)))
    m1_cal_err = 1;
   if ((m2_counter > (m2_CCW_state_4+5)) || (m2_counter < (m2_CCW_state_4-5)))
    m2_cal_err = 1;
    
   m1_state = 3;
   m2_state = 3;
   m1_driver();
   m2_driver();
   delay(1000);
   Serial.print("Motor 1 Reading: ");
   Serial.println(m1_counter);
   Serial.print("Motor 2 Reading: ");
   Serial.println(m2_counter);
   if ((m1_counter > (m1_CCW_state_3+5)) || (m1_counter < (m1_CCW_state_3-5)))
    m1_cal_err = 1;
   if ((m2_counter > (m2_CCW_state_3+5)) || (m2_counter < (m2_CCW_state_3-5)))
    m2_cal_err = 1;
    
   m1_state = 2;
   m2_state = 2;
   m1_driver();
   m2_driver();
   delay(1000);
   Serial.print("Motor 1 Reading: ");
   Serial.println(m1_counter);
   Serial.print("Motor 2 Reading: ");
   Serial.println(m2_counter);
   if ((m1_counter > (m1_CCW_state_2+5)) || (m1_counter < (m1_CCW_state_2-5)))
    m1_cal_err = 1;
   if ((m2_counter > (m2_CCW_state_2+5)) || (m2_counter < (m2_CCW_state_2-5)))
    m2_cal_err = 1;

   m1_state = 1;
   m2_state = 1;
   m1_driver();
   m2_driver();
   delay(1000);
     
  if(m1_cal_err == 0 && m2_cal_err == 0)
   Serial.println("CW Calibration Test Passed."); 
  else if (m1_cal_err)
  {
    Serial.println("Motor One CW Calibration Test Failed."); 
    return;
  }
  else
  {
    Serial.println("Motor Two CW Calibration Test Failed."); 
    return;
  } 
}
