/*  
  Line Following AGV Testing Program
  PT. Stechoq Robotika Indonesia
  
  Date    : 18 SEPTEMBER 2024
  Author  : Alfonsus Giovanni Mahendra Putra - Universitas Diponegoro
*/

/*USER PRIVATE INCLUDE*/
//************************************************************************************************************************************************************************************
#include "PID_driver.h"
//************************************************************************************************************************************************************************************


/*USER PRIVATE TYPEDEF*/
//************************************************************************************************************************************************************************************

// PID TYPEDEF ---------
PIDController pid_agv_f;
PIDController pid_agv_b;
// ---------------------

// SENSOR POS --
typedef enum{
  FRONT = 0x00,
  REAR  = 0x01
}LineSens_sel_t;
// -------------

// MOTOR SELECT ----
typedef enum{
  RIGHT_MOTOR = 0x01,
  LEFT_MOTOR,
  BOTH_MOTOR,
}Motor_sel_t;
// -----------------

// RUNNING DIR --
typedef enum{ 
  FORWARD = 0x01,
  BACKWARD,
  ROTATE_LEFT,
  ROTATE_RIGHT,
  ROTATE_BREAK,
  FORWARD_BREAK,
  BACKWARD_BREAK,
}Run_Dir_t;
// --------------

//************************************************************************************************************************************************************************************


/*USER PRIVATE DEFINE*/
//************************************************************************************************************************************************************************************

// PID SET ----------------
#define SAMPLE_TIME_S 0.01f
// ------------------------

// SPEAKER PIN -------
#define SPEAKER_PIN 46
// -------------------

// START BTN PIN ---
#define START_BTN 52
// -----------------

// MOTOR PIN --------
#define ENA_L      5
#define PWM_L      6
#define DIR_L      7
#define ENA_R      8
#define PWM_R      9
#define DIR_R      10
#define RUN_LR     30
#define BASE_SPD  250
// ------------------

// SENS COUNT ------
#define SENS_NUM  16
// -----------------

// TESTING SELECT ----
//#define LINESENS_TEST
//#define MOTOR_TEST
#define PID_TEST
// -------------------

// ALGORITHM SEL --------------
#define USE_PLXDAQ
#define USE_EARLY_CHECK
#define USE_OLD_SENS_COORDINATE
//#define USE_NEW_SENS_COORDINATE
// ----------------------------

//************************************************************************************************************************************************************************************


/*USER PRIVATE VARIABLE*/
//************************************************************************************************************************************************************************************
const uint8_t
sens_pin[2][SENS_NUM] = {
  {53, 51, 49, 47, 45, 43, 41, 39, 37, 35, 33, 31, 29, 27, 25, 23},
  {38, 36, 40, 34, 42, 32, 44, 30, 46, 28, 48, 26, 50, 24, 52, 22}
};

uint8_t
interval_check = 150,
line_false_cnt;

uint16_t 
sens_value,
sens_data;

int8_t
R_speed,
L_speed;

int16_t
R_line_pos,
L_line_pos,
line_pos;

float
pid_val;

unsigned long
tick_now,
last_tickA,
last_tickB;

bool
no_line = false,
line_detected = false,
start = false,
running = false;
//************************************************************************************************************************************************************************************


/*USER PRIVATE FUNCTION*/
//************************************************************************************************************************************************************************************
void check_motor(Motor_sel_t motor_sel, Run_Dir_t direction, uint8_t speed);
void check_sensor(LineSens_sel_t sensor_sel);

void read_sens(LineSens_sel_t sensor_sel);
void calculate_pid(LineSens_sel_t sensor_sel);
void agv_run(Run_Dir_t direction, uint8_t speed);
//************************************************************************************************************************************************************************************


/*VOID SETUP*/
//************************************************************************************************************************************************************************************
void setup(){
  // BAUD RATE SETUP --
  Serial.begin(115200);
  // ------------------

  // SETUP PLX DAQ ------------------------------------
  #ifdef USE_PLXDAQ
  Serial.println("CLEARDATA"); 
  Serial.println("LABEL,Time,Millis,Response,Set Point");
  #endif 
  // --------------------------------------------------

  // BUTTON & SPEAKER SETUP -------
  pinMode(START_BTN, INPUT_PULLUP);
  pinMode(SPEAKER_PIN, OUTPUT);
  digitalWrite(SPEAKER_PIN, LOW);
  // ------------------------------

  // SENSOR PIN SETUP ----------------------
  for(int i=0; i<1; i++){
    for(int j=0; j<SENS_NUM; j++){
      pinMode(sens_pin[i][j], INPUT_PULLUP);
    }
  }
  // ---------------------------------------

  // MOTOR PIN SETUP -----
  pinMode(ENA_R, OUTPUT);
  pinMode(PWM_R, OUTPUT);
  pinMode(DIR_R, OUTPUT);

  pinMode(ENA_L, OUTPUT);
  pinMode(PWM_L, OUTPUT);
  pinMode(DIR_L, OUTPUT);

  pinMode(RUN_LR, OUTPUT);
  // ---------------------

  // PID SETUP -----------------------
  pid_agv_f.Kp        = 1.4;     
  pid_agv_f.Ki        = 0.000025;       
  pid_agv_f.Kd        = 7.25; 		
  pid_agv_f.tau       = 0.01;
	pid_agv_f.limMax    = 125;     
  pid_agv_f.limMin    = -125;     
  pid_agv_f.limMaxInt = 5.0; 	   
  pid_agv_f.limMinInt = -5.0;
	pid_agv_f.T_sample  = SAMPLE_TIME_S;
  PIDController_Init(&pid_agv_f);

  pid_agv_b.Kp        = 0.0;     
  pid_agv_b.Ki        = 0.0;       
  pid_agv_b.Kd        = 0.0; 		
  pid_agv_b.tau       = 0.02;
	pid_agv_b.limMax    = 100;     
  pid_agv_b.limMin    = -100;     
  pid_agv_b.limMaxInt = 5.0; 	   
  pid_agv_b.limMinInt = -5.0;
	pid_agv_b.T_sample  = SAMPLE_TIME_S;
  PIDController_Init(&pid_agv_b);
  // ---------------------------------

  // EARLY CHECK SETUP -----------------------------
  #ifdef USE_EARLY_CHECK
  while(1){
    digitalWrite(RUN_LR, HIGH);
    delay(20);
    digitalWrite(ENA_R, LOW);
    digitalWrite(ENA_L, LOW);
    delay(20);
    analogWrite(PWM_R, 0);
    analogWrite(PWM_L, 0);
    delay(20);

    start = !digitalRead(START_BTN);
    if(start){
      read_sens(FRONT);
      if(line_detected) break;
      else{
        check_motor(BOTH_MOTOR, ROTATE_LEFT, 50);
        delay(2600);
        check_motor(BOTH_MOTOR, ROTATE_BREAK, 0);
        delay(1000);
        break;
      }
    }
  }

  #else
  while(1){
    start = !digitalRead(START_BTN);
    if(start){
      delay(1000);
      break;
    }
  }
  #endif
  // ---------------------------------------------
}
//************************************************************************************************************************************************************************************


/*VOID LOOP*/
//************************************************************************************************************************************************************************************
void loop(){
  #ifdef LINESENS_TEST
  digitalWrite(RUN_LR, HIGH);
  digitalWrite(ENA_R, LOW);
  digitalWrite(ENA_L, LOW);
  check_sensor(FRONT);
  #endif

  #ifdef MOTOR_TEST
  digitalWrite(SPEAKER_PIN, HIGH);
  check_motor(BOTH_MOTOR, FORWARD, 50);
  #endif 

  #ifdef PID_TEST
  digitalWrite(SPEAKER_PIN, HIGH);
  agv_run(FORWARD, 150);
  
  if(millis()-last_tickA > interval_check && !line_detected){
    line_false_cnt++;
    last_tickA = millis();
  }
  else if (line_detected) line_false_cnt = 0;

  if(line_false_cnt >= 5) no_line = true;

  if(no_line){
    check_motor(BOTH_MOTOR, FORWARD_BREAK, 0);
    delay(1500);

    while(1){
      check_motor(BOTH_MOTOR, ROTATE_LEFT, 50);
      delay(2600);
      check_motor(BOTH_MOTOR, ROTATE_BREAK, 0);
      delay(1000);
      line_pos = 0;
      no_line = false;
      break;
    }
  }

  if(millis()-last_tickB > 25){
    Serial.print("DATA,TIME,");
    Serial.print(millis());
    Serial.print(" ,");
    Serial.println(line_pos);

    last_tickB = millis();
  }
  #endif
}
//************************************************************************************************************************************************************************************


/*--- READ SENSOR FUNCTION ---*/
//************************************************************************************************************************************************************************************
void read_sens(LineSens_sel_t sensor_sel){
  #ifdef USE_OLD_SENS_COORDINATE
  if(sensor_sel == FRONT){
    for(int i=0; i<SENS_NUM; i++){
      sens_value = !digitalRead(sens_pin[FRONT][i]);
      bitWrite(sens_data, i, sens_value);
    }
  }

  else if(sensor_sel == REAR){
    // ONGOING
  }

  if     (sens_data == 0b1000000000000000) line_pos = -24;
  else if(sens_data == 0b1100000000000000) line_pos = -21;
  else if(sens_data == 0b1110000000000000) line_pos = -19;
  else if(sens_data == 0b1111000000000000) line_pos = -16;
  else if(sens_data == 0b1111100000000000) line_pos = -14;
  else if(sens_data == 0b0111100000000000) line_pos = -12;
  else if(sens_data == 0b0111110000000000) line_pos = -10;
  else if(sens_data == 0b0011110000000000) line_pos = -8;
  else if(sens_data == 0b0001110000000000) line_pos = -7;
  else if(sens_data == 0b0001111000000000) line_pos = -6;
  else if(sens_data == 0b0000111000000000) line_pos = -5;
  else if(sens_data == 0b0000111100000000) line_pos = -4;
  else if(sens_data == 0b0000011100000000) line_pos = -3;
  else if(sens_data == 0b0000011110000000) line_pos = -2;
  else if(sens_data == 0b0000001110000000) line_pos = -1;
  else if(sens_data == 0b0000001111000000) line_pos = 0;
  else if(sens_data == 0b0000000111000000) line_pos = 1;
  else if(sens_data == 0b0000000111100000) line_pos = 2;
  else if(sens_data == 0b0000000011100000) line_pos = 3;
  else if(sens_data == 0b0000000011110000) line_pos = 4;
  else if(sens_data == 0b0000000001110000) line_pos = 5;
  else if(sens_data == 0b0000000001111000) line_pos = 6;
  else if(sens_data == 0b0000000000111000) line_pos = 7;
  else if(sens_data == 0b0000000000111100) line_pos = 8;
  else if(sens_data == 0b0000000000111110) line_pos = 10;
  else if(sens_data == 0b0000000000011110) line_pos = 12;
  else if(sens_data == 0b0000000000011111) line_pos = 14;
  else if(sens_data == 0b0000000000001111) line_pos = 16;
  else if(sens_data == 0b0000000000000111) line_pos = 19;
  else if(sens_data == 0b0000000000000011) line_pos = 21;
  else if(sens_data == 0b0000000000000001) line_pos = 25;
  #endif

  #ifdef USE_NEW_SENS_COORDINATE
  L_line_pos = 0;
  R_line_pos = 0;

  for(int i=0; i<8; i++){
    sens_value = !digitalRead(sens_pin[FRONT][i]);
    bitWrite(sens_data, 7-i, sens_value);
  }

  for(int i=8; i<SENS_NUM; i++){
    sens_value = !digitalRead(sens_pin[FRONT][i]);
    bitWrite(sens_data, i, sens_value);
  }

  L_line_pos = (sens_data >> 8)*(-1);
  R_line_pos = sens_data & 0xFF;
  line_pos = R_line_pos + L_line_pos;
  #endif

  if(sens_data == 0b0000000000000000) line_detected = false;
  else line_detected = true;
}
//************************************************************************************************************************************************************************************


/*--- CALCULATE PID FUNCTION ---*/
//************************************************************************************************************************************************************************************
void calculate_pid(LineSens_sel_t sensor_sel){
  read_sens(sensor_sel);

  if(sensor_sel == FRONT){
    PIDController_Update(&pid_agv_f, 0, (float)line_pos);
    pid_val = pid_agv_f.out;
  }

  else{
    PIDController_Update(&pid_agv_b, 0, (float)line_pos);
    pid_val = pid_agv_b.out;
  }
}
//************************************************************************************************************************************************************************************


/*--- AGV RUNNING FUNCTION ---*/
//************************************************************************************************************************************************************************************
void agv_run(Run_Dir_t direction, uint8_t speed){
  digitalWrite(RUN_LR, LOW);
  digitalWrite(ENA_R, LOW);
  digitalWrite(ENA_L, LOW);

  if(direction == FORWARD){
    calculate_pid(FRONT);
    R_speed = speed + pid_val;
    L_speed = speed - pid_val;

    digitalWrite(DIR_R, LOW);
    digitalWrite(DIR_L, HIGH);
  }

  else if(direction == BACKWARD){
    calculate_pid(REAR);
    R_speed = speed - pid_val;
    L_speed = speed + pid_val;

    digitalWrite(DIR_R, HIGH);
    digitalWrite(DIR_L, LOW);
  }

  constrain(R_speed, 0, speed);
  constrain(L_speed, 0, speed);

  analogWrite(PWM_R, R_speed);
  analogWrite(PWM_L, L_speed);
}
//************************************************************************************************************************************************************************************


/*--- CHECK SENSOR FUNCTION ---*/
//************************************************************************************************************************************************************************************
void check_sensor(LineSens_sel_t sensor_sel){
  calculate_pid(sensor_sel);
  Serial.print("Sens Data: 0b");

  for(int i=SENS_NUM-1; i>=0; i--){
    Serial.print(bitRead(sens_data, i));
  }
  Serial.print("    ");
  Serial.print("Line State: ");
  Serial.print(line_detected);
  
  Serial.print("    ");
  Serial.print("Right Position: ");
  Serial.print(R_line_pos);
  
  Serial.print("    ");
  Serial.print("Left Position: ");
  Serial.print(L_line_pos);
  
  Serial.print("    ");
  Serial.print("Line Position: ");
  Serial.print(line_pos);

  Serial.print("    ");
  Serial.print("PID: ");
  Serial.println(pid_val);
}
//************************************************************************************************************************************************************************************


/*--- MOTOR CHECK FUNCTION ---*/
//************************************************************************************************************************************************************************************
void check_motor(Motor_sel_t motor_sel, Run_Dir_t direction, uint8_t speed){
  digitalWrite(RUN_LR, LOW);
  
  if(motor_sel == RIGHT_MOTOR){
    digitalWrite(ENA_R, HIGH);
    digitalWrite(ENA_L, LOW);

    if(direction == FORWARD) digitalWrite(DIR_R, LOW);
    else digitalWrite(DIR_R, HIGH);

    analogWrite(PWM_R, speed);
  }

  else if(motor_sel == LEFT_MOTOR){
    digitalWrite(ENA_R, LOW);
    digitalWrite(ENA_L, HIGH);

    if(direction == FORWARD) digitalWrite(DIR_L, HIGH);
    else digitalWrite(DIR_L, LOW);

    analogWrite(PWM_L, speed);
  }

  else{
    digitalWrite(ENA_R, LOW);
    digitalWrite(ENA_L, LOW);
    
    if(direction == FORWARD){
      digitalWrite(DIR_R, LOW);
      digitalWrite(DIR_L, HIGH);
      analogWrite(PWM_R, speed);
      analogWrite(PWM_L, speed);
    }

    else if(direction == BACKWARD){
      digitalWrite(DIR_R, HIGH);
      digitalWrite(DIR_L, LOW);
      analogWrite(PWM_R, speed);
      analogWrite(PWM_L, speed);
    }

    else if(direction == ROTATE_LEFT){
      digitalWrite(DIR_R, LOW);
      digitalWrite(DIR_L, LOW);
      analogWrite(PWM_R, speed);
      analogWrite(PWM_L, speed);
    }

    else if(direction == ROTATE_RIGHT){
      digitalWrite(DIR_R, HIGH);
      digitalWrite(DIR_L, HIGH);
      analogWrite(PWM_R, speed);
      analogWrite(PWM_L, speed);
    }
    
    else if(direction == ROTATE_BREAK){
      digitalWrite(RUN_LR, HIGH);
    }

    else if(direction == FORWARD_BREAK){
     digitalWrite(RUN_LR, HIGH);
    }
  }
}
//************************************************************************************************************************************************************************************
