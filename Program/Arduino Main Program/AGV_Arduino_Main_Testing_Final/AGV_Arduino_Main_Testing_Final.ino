/*
  Automated Guided Vehicle - Arduino Main Program
  PT. Stechoq Robotika Indonesia

  Data    : 18 Oktober 2024
  Author  : Alfonsus Giovanni Mahendra Puta - Universitas Diponegoro
*/

//#define USE_NEW_PINOUT
#define USE_OLD_PINOUT

/*User Private Include*/
#include "PID_driver.h"
#include "CustomSerial.h"
#include "Wire.h"
#include "Adafruit_PN532.h"

/*User Private Define*/
#define SENS_SWITCH 2

#define ENA_L       5    // Left Motor Run and Brake Pin (EL) <-PCB_PIN
#define PWM_L       6    // Left Motor Speed PWM (VR) <- PCB PIN
#define DIR_L       7    // Left Motor Direction (Z/F) <- PCB PIN
#define ENA_R       8    // Right Motor Run and Brake Pin (EL) <-PCB_PIN
#define PWM_R       9    // Right Motor Speed PWM (VR) <- PCB PIN
#define DIR_R       10   // Right Motor Direction (Z/F) <- PCB PIN

#ifdef USE_NEW_PINOUT
  #define SIG_LR      11   // Left Right Motor Start Stop (SIGNAL) <-PCB PIN
#endif

#ifdef USE_OLD_PINOUT
  #define SIG_LR      30   // Left Right Motor Start Stop (SIGNAL) <-PCB PIN
#endif

#ifdef USE_NEW_PINOUT
  #define PROX_FRONT  12
  #define PROX_REAR   13
#endif

#ifdef USE_OLD_PINOUT
  #define PROX_FRONT  A0  //A9
  #define PROX_REAR   A1 //A10
#endif

#define IRQ_NFC1    14
#define IRQ_NFC2    16
#define VIR_VCC1    15
#define VIR_VCC2    17

#define SERVO_LOCK  38
#define LS1         39
#define LS2         40

#ifdef USE_NEW_PINOUT
  #define START_BTN       41
  #define SPEAKER_PIN     42
  #define PILOTLAMP_PIN   43
#endif

#ifdef USE_OLD_PINOUT
  #define START_BTN       52
  #define SPEAKER_PIN     46
  #define PILOTLAMP_PIN   46
#endif

#define SENS_NUM  16
#define DATA_AUTH_HEADER    0xFF

/*User Private Typedef*/

/*User Private Variable*/
const uint8_t
sens_pin[SENS_NUM] = {53, 51, 49, 47, 45, 43, 41, 39, 37, 35, 33, 31, 29, 27, 25, 23};

const int32_t
sens_weight[SENS_NUM] = {0, 7.5, 12.5, 17.5, 22.5, 27.5, 32.5, 37.5, 42.5, 47.5, 52.5, 57.5, 62.5, 67.5, 72.5, 80};

const int
gridWidth = 8,
gridHeight = 8,
cellSize = 10;

uint8_t
line_false_interval = 150,
regenerative_interval = 5,
nfc_read_interval = 100;

uint8_t
NFC_Timeout = 25,
brake_val = 10,
decrease_speedL,
decrease_speedR;

uint8_t
speed_cnt,
brake_cnt,
sens_cnt,
turn_timer_cnt,
cross_timer_cnt,
line_false_cnt,
prev_sel_sens;

uint8_t
head_dir,
step_stack,
stack_size,
free_nodes_count,
turning_decision,
path_step[gridWidth*gridHeight];

uint16_t
sens_value,
sens_data;

int16_t
L_speed,
R_speed;

int
total_weight,
sens_count;

float
line_pos,
pid_val;

unsigned long
prev_dummytick,   // -> Dummy Testing Tick
prev_tickComtx,   // -> Timer Serial Com TX
prev_tickComrx,   // -> Timer Serial Com RX
prev_turningTick, // -> Turning Reset Tick
prev_tickA,       // -> Timer Check Garis
prev_tickB,       // -> Timer Regenerative
prev_tickC,       // -> Timer Pembacaan NFC
prev_tickD,       // -> Timer Validasi Reset NFC
prev_tickE,       // -> Timer Validasi Pmebacaan NFC Turn Sign
prev_tickF;       // -> Timer Validasi Pmebacaan NFC Cross Sign

bool
arrived_at_destination = false,
btn_pressed = false,
config_done = false,
start = false,
home = true,
running = false,
no_line = false,
line_detected = false,
front_state = false,
rear_state = false,
path_planned = false;

String
Tag_Start,
Tag_Destination;

int grid[gridHeight][gridWidth] = {
  {0, 1, 1, 1, 1, 1, 1, 0},
  {0, 1, 1, 1, 1, 1, 1, 0},
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 1, 0, 1, 1, 0, 1, 0},
  {0, 1, 0, 1, 1, 0, 1, 0},
  {1, 1, 0, 1, 1, 0, 1, 1},
  {1, 1, 0, 1, 1, 0, 1, 1},
  {1, 1, 0, 0, 0, 0, 1, 1},
};

int 
openSetSize = 0,
openSetF[gridWidth * gridHeight];

/*User Private Typedef*/
struct Point {
    int x, y;
    Point(int x = 0, int y = 0) : x(x), y(y) {}
};

Point openSet[gridWidth * gridHeight];          // Open all available nodes
Point cameFrom[gridWidth][gridHeight];          // Save current pos to nodes
Point traceBack[gridWidth * gridHeight];        // Traceback nodes pattern from goal to start
Point invert_traceBack[gridWidth * gridHeight]; // Invert traceback value
Point nodes_checked[gridWidth * gridHeight];

typedef enum{
  NONE_TURN,
  STRAIGHT,
  TURN_LEFT,
  TURN_RIGHT,
  TURN_STOP
}Decision_t;

typedef enum{
  HEAD_XP = 0x01,
  HEAD_XN,
  HEAD_YP,
  HEAD_YN
}Head_Dir_t;

typedef enum{
  SUCCESS = 0x01,
  UID_READ_ERROR,
  BLOCK_AUTH_ERROR,
  DATA_READ_ERROR, 
  DATA_WRITE_ERROR,
  TAG_READ_ERROR,
  TAG_WRITE_ERROR,
  UNREGISTERED_TAG,
}NFC_Status_t;

typedef enum{
  FRONT_NFC = 0x01,
  REAR_NFC
}NFC_Select_t;

typedef struct{
  uint8_t 
  stored_data[16],
  uid[4],
  new_uid[13],
  tagType,
  tagPosX,
  tagPosY;

  uint16_t
  tagTypeValue,
  tagNum;
}Tag_Data_t;

typedef enum{
  FRONT = 0x00,
  REAR  = 0x01
}Sens_sel_t;

Tag_Data_t NFC_F;
Tag_Data_t NFC_R;

Param_t parameter;

PIDController pid_agv_f;  // -> PID Jalan Maju
PIDController pid_agv_b;  // -> PID Jalan Mundur

Adafruit_PN532 nfc(IRQ_NFC1, 3);

/*User Private Function Declaration*/
void Read_Sens(Sens_sel_t sensor_sel);
void Calc_PID(Sens_sel_t sensor_sel);
void Line_Check(void);
bool Line_Search(Sens_sel_t sensor_sel);
void Motor_Handler_LF(uint8_t direction, uint8_t accel, uint8_t brake, uint8_t speed);
void Run_AGV(uint8_t agv_mode);
void Destination_Handler(void);

void NFC_Handler(NFC_Select_t select);
void Turn_Check(NFC_Select_t select);
void Cross_Check(NFC_Select_t select);
void Station_Check(NFC_Select_t select);
void Nodes_check(NFC_Select_t select);

bool Run_Path_Planning(void);

bool Read_Proximity(Sens_sel_t sensor_sel);

float Read_Voltage(void);
float Read_Current(void);
float Check_Battery_Cappacity(void);

NFC_Status_t NFC_readData(Tag_Data_t *nfc);
NFC_Status_t NFC_readTag(Tag_Data_t *nfc);

Point Return_OpenSet(void);
int Calc_Heuristic(Point a, Point b);
void Insert_OpenSet(Point p, int f);
bool Run_AStar(Point start, Point goal);
void Reconstruct_Path(Point start, Point goal, char displayGrid[gridWidth][gridHeight]);
void Reconstruct_PathDir(void);

/*User Main Program*/
void setup(){
  // Communication Setup
  Serial.begin(1000000);
  Serial.setTimeout(100);

  // Sensor Pin Setup
  for(int i=0; i<SENS_NUM; i++){
    pinMode(sens_pin[i], INPUT_PULLUP);
  }

  pinMode(PROX_FRONT, INPUT);
  pinMode(PROX_REAR, INPUT);

  // Motor Pin Setup
  pinMode(ENA_L, OUTPUT);
  pinMode(PWM_L, OUTPUT);
  pinMode(DIR_L, OUTPUT);
  pinMode(ENA_R, OUTPUT);
  pinMode(PWM_R, OUTPUT);
  pinMode(DIR_R, OUTPUT);
  pinMode(SIG_LR, OUTPUT);
  
  digitalWrite(SIG_LR, HIGH);
  delay(20);
  digitalWrite(ENA_R, LOW);
  digitalWrite(ENA_L, LOW);
  delay(20);
  analogWrite(PWM_R, 0);
  analogWrite(PWM_L, 0);
  delay(20);

  // Interface Pin Setup
  pinMode(START_BTN, INPUT_PULLUP);
  pinMode(SPEAKER_PIN, OUTPUT);
  pinMode(PILOTLAMP_PIN, OUTPUT);

  // Control Pin Setup
  pinMode(IRQ_NFC1, OUTPUT);
  pinMode(IRQ_NFC2, OUTPUT);
  pinMode(VIR_VCC1, OUTPUT);
  pinMode(VIR_VCC2, OUTPUT);

  #ifdef USE_NEW_PINOUT
    pinMode(SERVO_LOCK, OUTPUT);
    pinMode(LS1, INPUT_PULLUP);
    pinMode(LS2, INPUT_PULLUP);
  #endif

  pinMode(SENS_SWITCH, OUTPUT);
  digitalWrite(SENS_SWITCH, LOW);

  // Communication Setup
  parameter.Running_Mode = NOT_SET,
  parameter.Base_Speed = 80;
  parameter.Running_State = STOP;
  parameter.Running_Accel = REGENERATIVE_ACCEL;
  parameter.Running_Brake = REGENERATIVE_BRAKE;
  parameter.Current_Pos = HOME;
  parameter.SensorA_Status = NOT_DETECTED;
  parameter.SensorB_Status = NOT_DETECTED;
  parameter.Tag_sign = NONE_SIGN;

  // Pid Setup
  pid_agv_f.Kp        = 1.225; // pid_agv_f.Kp        = 0.825; 
  pid_agv_f.Ki        = 0.0005;       
  pid_agv_f.Kd        = 0.035; 		
  pid_agv_f.tau       = 0.01;
	pid_agv_f.limMax    = 100;     
  pid_agv_f.limMin    = -100;     
  pid_agv_f.limMaxInt = 5.0; 	   
  pid_agv_f.limMinInt = -5.0;
	pid_agv_f.T_sample  = 0.01;
  PIDController_Init(&pid_agv_f);

  // NFC Setup
  nfc.begin();
  nfc.SAMConfig();
}

void loop(){
  /*
  rear_state = Read_Proximity(REAR);
  Serial.print("Sensor: ");
  Serial.println(rear_state);
  */

  /*
  Motor_Handler_Lidar(FORWARD, NORMAL_ACCEL, NORMAL_BRAKE, 100);
  */

  /*
  if(!config_done){
    digitalWrite(LED_BUILTIN, LOW);
    Receive_Serial(&parameter);

    if(parameter.Running_Mode != NOT_SET && parameter.Running_State == START){
      prev_dummytick = millis();
      config_done = true;
    }
  } 

  else if(config_done){
    digitalWrite(LED_BUILTIN, HIGH);

    parameter.Current_Pos = ON_THE_WAY;
    parameter.Send_counter = 10;
    parameter.Pickup_counter = 11;
    parameter.Battery_level = 90.5;

    Transmit_Serial(&parameter);

    if(millis() - prev_dummytick > 5000){
      parameter.Running_Mode = NOT_SET;
      parameter.Running_State = STOP;
      config_done = false;
    }
  }
  */
  
  /*
  if(parameter.Running_State == START && parameter.Running_Dir == FORWARD && parameter.Start_Pos == HOME && parameter.Destination == STATION_A){
    if(millis() - prev_dummytick > 150){
      if(digitalRead(LED_BUILTIN) == LOW) digitalWrite(LED_BUILTIN, HIGH);
      else digitalWrite(LED_BUILTIN, LOW);
      prev_dummytick = millis();
    }
  }
  */

  if(!config_done){
    Receive_Serial(&parameter);
    btn_pressed = !digitalRead(START_BTN);
    
    Read_Sens(FRONT);
    NFC_readTag(&NFC_F);

    // Check if there is no line and home tag detected and running mode is not set
    if(!line_detected && NFC_F.tagType != HOME_SIGN && parameter.Running_Mode == NOT_SET){
      digitalWrite(PILOTLAMP_PIN, LOW);
    }

    // Check if line is detected but home tag is not and running mode is not set
    else if(line_detected && NFC_F.tagType != HOME_SIGN && parameter.Running_Mode == NOT_SET){
      if(millis() - prev_dummytick > 150){
        if(digitalRead(PILOTLAMP_PIN) == LOW) digitalWrite(PILOTLAMP_PIN, HIGH);
        else digitalWrite(PILOTLAMP_PIN, LOW);
        prev_dummytick = millis();
      }
    }

    // Check if line and home tag is detected but running mode is not set
    else if(line_detected && NFC_F.tagType == HOME_SIGN && parameter.Running_Mode == NOT_SET){
      if(millis() - prev_dummytick > 500){
        if(digitalRead(PILOTLAMP_PIN) == LOW) digitalWrite(PILOTLAMP_PIN, HIGH);
        else digitalWrite(PILOTLAMP_PIN, LOW);
        prev_dummytick = millis();
      }
      head_dir = HEAD_XP;
    }

    // All needed state is accomplished
    else if(line_detected && NFC_F.tagType == HOME_SIGN && parameter.Running_Mode != NOT_SET){
      digitalWrite(PILOTLAMP_PIN, HIGH);

      if(line_detected && parameter.Running_Mode != NOT_SET && parameter.Running_State == START){
        digitalWrite(PILOTLAMP_PIN, LOW);
        parameter.Running_State = START;
        parameter.Running_Dir = FORWARD;
        Serial.flush();
        delay(1000);
        config_done = true;
      }

      else if(line_detected && btn_pressed){
        parameter.Running_Mode = LF_MODE;
        parameter.Running_State = START;
        parameter.Running_Dir = FORWARD;
        
        digitalWrite(PILOTLAMP_PIN, LOW);
        delay(1000);
        parameter.Running_State = START;
        config_done = true;
      }
    }
  }

  else if(config_done){
    switch(parameter.Running_State){
      case START:
      parameter.Current_Pos = ON_THE_WAY;
      Transmit_Serial(&parameter);

      Line_Check();
      Motor_Handler_LF(FORWARD, REGENERATIVE_ACCEL, REGENERATIVE_BRAKE, 80);
      digitalWrite(PILOTLAMP_PIN, HIGH);
      Run_AGV(parameter.Running_Mode);
      break;

      case STOP:
      if(millis() > prev_dummytick > 500){
        if(digitalRead(PILOTLAMP_PIN) == LOW) digitalWrite(PILOTLAMP_PIN, HIGH);
        else digitalWrite(PILOTLAMP_PIN, LOW);
        prev_dummytick = millis();
      }

      Motor_Handler_LF(BRAKE, NORMAL_ACCEL, REGENERATIVE_BRAKE, parameter.Base_Speed);
      break;

      case PAUSE:
      if(millis() > prev_tickComrx > 100){
        Receive_Serial(&parameter);
        prev_tickComrx = millis();
      }

      if(parameter.Running_Dir == FORWARD) Calc_PID(FRONT);
      else if(parameter.Running_Dir == BACKWARD) Calc_PID(REAR);
      Motor_Handler_LF(BRAKE, NORMAL_ACCEL, REGENERATIVE_BRAKE, parameter.Base_Speed);
      break;
    }
  }
}

/*User Private Function Initialize*/
// Read Magnetic Line Sensor Function
void Read_Sens(Sens_sel_t sensor_sel){
  total_weight = 0;
  sens_count = 0;

  if(sensor_sel == FRONT && prev_sel_sens != sensor_sel){
    digitalWrite(SENS_SWITCH, LOW); 
    delay(50);
  }
  else if(sensor_sel == REAR && prev_sel_sens != sensor_sel){
    digitalWrite(SENS_SWITCH, HIGH);
    delay(50);
  }
  prev_sel_sens = sensor_sel;

  if(turning_decision == NONE_TURN){
    sens_data == 0b0000000000000000;
    for(int i=0; i<SENS_NUM; i++){
      sens_value = !digitalRead(sens_pin[i]);
      bitWrite(sens_data, i, sens_value);

      total_weight += (bitRead(sens_data, i) & 0xFF) * sens_weight[i];
      sens_count += (bitRead(sens_data, i) & 0xFF);
    }
  }

  else if(turning_decision == STRAIGHT){
    sens_data == 0b0000000000000000;
    for(int i=6; i<10; i++){
      sens_value = !digitalRead(sens_pin[i]);
      bitWrite(sens_data, i, sens_value);

      total_weight += (bitRead(sens_data, i) & 0xFF) * sens_weight[i];
      sens_count += (bitRead(sens_data, i) & 0xFF);
    }
  }

  else if(turning_decision == TURN_LEFT){
    sens_data == 0b0000000000000000;
    for(int i=10; i<SENS_NUM; i++){
      sens_value = !digitalRead(sens_pin[i]);
      bitWrite(sens_data, i, sens_value);

      total_weight += (bitRead(sens_data, i) & 0xFF) * sens_weight[i];
      sens_count += (bitRead(sens_data, i) & 0xFF);
    }
  }

  else if(turning_decision == TURN_RIGHT){
    sens_data == 0b0000000000000000;
    for(int i=0; i<6; i++){
      sens_value = !digitalRead(sens_pin[i]);
      bitWrite(sens_data, i, sens_value);

      total_weight += (bitRead(sens_data, i) & 0xFF) * sens_weight[i];
      sens_count += (bitRead(sens_data, i) & 0xFF);
    }
  }

  if(sens_data == 0b0000000000000000){
    line_pos = 0;
    line_detected = false;
  }
  else{
    line_detected = true;
    line_pos = 40 - (total_weight/sens_count);
  }
}

// PID Calculation Function
void Calc_PID(Sens_sel_t sensor_sel){
  Read_Sens(sensor_sel);

  if(sensor_sel == FRONT){
    PIDController_Update(&pid_agv_f, 0, line_pos);
    pid_val = pid_agv_f.out;
  }
  else{
    PIDController_Update(&pid_agv_b, 0, line_pos);
    pid_val = pid_agv_b.out;
  }
}

// Line Search Function
bool Line_Search(Sens_sel_t sensor_sel){
  uint8_t direction;
  if(line_pos > 15) direction = ROTATE_LEFT;
  else if(line_pos >= -15 && line_pos <= 15) direction = ROTATE_RIGHT;
  else if(line_pos <= -15) direction = ROTATE_RIGHT;

  while(1){
    Read_Sens(sensor_sel);
    Motor_Handler_LF(direction, NORMAL_ACCEL, NORMAL_BRAKE, 30);
    if(line_pos >= -20 && line_pos <= 20 && line_detected){
      Motor_Handler_LF(BRAKE, NORMAL_ACCEL, NORMAL_BRAKE, 30);
      break;
    }
  }
  return true;
}

// LINE CHECK SUBROUTINE FUNCTION
void Line_Check(void){
  if(millis()-prev_tickA > line_false_interval && !line_detected){
    line_false_cnt++;
    prev_tickA = millis();
  }
  else if (line_detected) line_false_cnt = 0;

  if(line_false_cnt >= 5) no_line = true;

  if(no_line){
    Motor_Handler_LF(BRAKE, NORMAL_ACCEL, REGENERATIVE_BRAKE, parameter.Base_Speed);
    delay(1500);
    while(1){
      if(Line_Search(FRONT) == true){
        line_false_cnt = 0;
        no_line = false;
        delay(1000);
        break;
      }
      else continue;
    }
  }
}

// Proximity Sensor Read Function
bool Read_Proximity(Sens_sel_t sensor_sel){
  bool value;
  if(sensor_sel == FRONT) value = !digitalRead(PROX_FRONT);
  else if(sensor_sel == REAR) value = !digitalRead(PROX_REAR);

  return value;
}

// LF Motor Handler Function
void Motor_Handler_LF(uint8_t direction, uint8_t accel, uint8_t brake, uint8_t speed){
  uint8_t 
  left_tolerance = 0,
  right_tolerance = 0;

  digitalWrite(SIG_LR, LOW);
  digitalWrite(ENA_R, LOW);
  digitalWrite(ENA_L, LOW);

  switch(direction){
    case FORWARD:
    Calc_PID(FRONT);
    
    digitalWrite(DIR_R, LOW);
    digitalWrite(DIR_L, HIGH);

    switch(accel){
      case NORMAL_ACCEL:
      R_speed = speed + pid_val - decrease_speedR;
      L_speed = speed - pid_val - decrease_speedL;

      if(R_speed >= speed) R_speed = speed;
      else if(R_speed <= 0) R_speed = 0;

      if(L_speed >= speed) L_speed = speed;
      else if(L_speed <= 0) L_speed = 0;

      analogWrite(PWM_R, R_speed);
      analogWrite(PWM_L, L_speed);
      break;

      case REGENERATIVE_ACCEL:
      if(millis()-prev_tickB > regenerative_interval && speed_cnt <= speed && !running){
        speed_cnt++;
        analogWrite(PWM_R, speed_cnt);
        analogWrite(PWM_L, speed_cnt);
        prev_tickB = millis();
      }
      else if(speed_cnt == speed) running = true;

      if(running){
        R_speed = speed + pid_val - decrease_speedR;
        L_speed = speed - pid_val - decrease_speedL;

        if(R_speed >= speed) R_speed = speed;
        else if(R_speed <= 0) R_speed = 0;

        if(L_speed >= speed) L_speed = speed;
        else if(L_speed <= 0) L_speed = 0;

        analogWrite(PWM_R, R_speed);
        analogWrite(PWM_L, L_speed);
        speed_cnt = 0;
      }
      break;
    }
    break;

    case BACKWARD:
    Calc_PID(REAR);
    
    digitalWrite(DIR_R, HIGH);
    digitalWrite(DIR_L, LOW);

    switch(accel){
      case NORMAL_ACCEL:
      R_speed = speed + pid_val - decrease_speedR;
      L_speed = speed - pid_val - decrease_speedL;

      constrain(R_speed, 0, speed);
      constrain(L_speed, 0, speed);
      analogWrite(PWM_R, R_speed);
      analogWrite(PWM_L, L_speed);
      break;

      case REGENERATIVE_ACCEL:
      if(millis()-prev_tickB > regenerative_interval && speed_cnt <= speed && !running){
        speed_cnt++;
        analogWrite(PWM_R, speed_cnt);
        analogWrite(PWM_L, speed_cnt);
        prev_tickB = millis();
      }
      else if(speed_cnt == speed) running = true;

      if(running){
        R_speed = speed + pid_val - right_tolerance - decrease_speedR;
        L_speed = speed - pid_val - left_tolerance - decrease_speedL;
        constrain(R_speed, 0, speed - right_tolerance);
        constrain(L_speed, 0, speed - left_tolerance);

        analogWrite(PWM_R, R_speed);
        analogWrite(PWM_L, L_speed);
        speed_cnt = 0;
      }
      break;
    }
    break;

    case LEFT:
    running = true;
    digitalWrite(ENA_R, HIGH);
    digitalWrite(ENA_L, LOW);

    digitalWrite(DIR_R, LOW);
    analogWrite(PWM_R, speed);
    break;
  
    case RIGHT:
    running = true;
    digitalWrite(ENA_R, LOW);
    digitalWrite(ENA_L, HIGH);

    digitalWrite(DIR_L, HIGH);
    analogWrite(PWM_L, speed);
    break;
  
    case ROTATE_LEFT:
    running = true;
    digitalWrite(DIR_R, LOW);
    digitalWrite(DIR_L, LOW);
    analogWrite(PWM_R, speed);
    analogWrite(PWM_L, speed);
    break;
  
    case ROTATE_RIGHT:
    running = true;
    digitalWrite(DIR_R, HIGH);
    digitalWrite(DIR_L, HIGH);
    analogWrite(PWM_R, speed);
    analogWrite(PWM_L, speed);
    break;

    case BRAKE:
    switch(brake){
      case NORMAL_BRAKE:
      digitalWrite(SIG_LR, HIGH);
      running = false;
      break;

      case REGENERATIVE_BRAKE:
      if(running){
        for(int i=speed; i>= 0; i-=brake_val){
          analogWrite(PWM_L, i);
          analogWrite(PWM_R, i);
          delay(regenerative_interval);
        }
      }
      running = false;
      break;
    }
    break;
  }
}

// Lidar Motor Handler Function
void Motor_Handler_Lidar(uint8_t direction, uint8_t accel, uint8_t brake, uint8_t speed){
  uint8_t 
  left_tolerance = 0,
  right_tolerance = 0;

  //digitalWrite(SIG_LR, LOW);
  digitalWrite(ENA_R, HIGH);
  digitalWrite(ENA_L, HIGH);

  switch(direction){
    case FORWARD:
    digitalWrite(DIR_R, HIGH);
    digitalWrite(DIR_L, HIGH);

    switch(accel){
      case NORMAL_ACCEL:
      R_speed = speed + pid_val - decrease_speedR;
      L_speed = speed - pid_val - decrease_speedL;

      if(R_speed >= speed) R_speed = speed;
      else if(R_speed <= 0) R_speed = 0;

      if(L_speed >= speed) L_speed = speed;
      else if(L_speed <= 0) L_speed = 0;

      analogWrite(PWM_R, R_speed);
      analogWrite(PWM_L, L_speed);
      break;

      case REGENERATIVE_ACCEL:
      if(millis()-prev_tickB > regenerative_interval && speed_cnt <= speed && !running){
        speed_cnt++;
        analogWrite(PWM_R, speed_cnt);
        analogWrite(PWM_L, speed_cnt);
        prev_tickB = millis();
      }
      else if(speed_cnt == speed) running = true;

      if(running){
        R_speed = speed + pid_val - decrease_speedR;
        L_speed = speed - pid_val - decrease_speedL;

        if(R_speed >= speed) R_speed = speed;
        else if(R_speed <= 0) R_speed = 0;

        if(L_speed >= speed) L_speed = speed;
        else if(L_speed <= 0) L_speed = 0;

        analogWrite(PWM_R, R_speed);
        analogWrite(PWM_L, L_speed);
        speed_cnt = 0;
      }
      break;
    }
    break;

    case BACKWARD:
    digitalWrite(DIR_R, HIGH);
    digitalWrite(DIR_L, LOW);

    switch(accel){
      case NORMAL_ACCEL:
      R_speed = speed + pid_val - decrease_speedR;
      L_speed = speed - pid_val - decrease_speedL;

      constrain(R_speed, 0, speed);
      constrain(L_speed, 0, speed);
      analogWrite(PWM_R, R_speed);
      analogWrite(PWM_L, L_speed);
      break;

      case REGENERATIVE_ACCEL:
      if(millis()-prev_tickB > regenerative_interval && speed_cnt <= speed && !running){
        speed_cnt++;
        analogWrite(PWM_R, speed_cnt);
        analogWrite(PWM_L, speed_cnt);
        prev_tickB = millis();
      }
      else if(speed_cnt == speed) running = true;

      if(running){
        R_speed = speed + pid_val - right_tolerance - decrease_speedR;
        L_speed = speed - pid_val - left_tolerance - decrease_speedL;
        constrain(R_speed, 0, speed - right_tolerance);
        constrain(L_speed, 0, speed - left_tolerance);

        analogWrite(PWM_R, R_speed);
        analogWrite(PWM_L, L_speed);
        speed_cnt = 0;
      }
      break;
    }
    break;

    case LEFT:
    running = true;
    digitalWrite(ENA_R, HIGH);
    digitalWrite(ENA_L, LOW);

    digitalWrite(DIR_R, LOW);
    analogWrite(PWM_R, speed);
    break;
  
    case RIGHT:
    running = true;
    digitalWrite(ENA_R, LOW);
    digitalWrite(ENA_L, HIGH);

    digitalWrite(DIR_L, HIGH);
    analogWrite(PWM_L, speed);
    break;
  
    case ROTATE_LEFT:
    running = true;
    digitalWrite(DIR_R, LOW);
    digitalWrite(DIR_L, LOW);
    analogWrite(PWM_R, speed);
    analogWrite(PWM_L, speed);
    break;
  
    case ROTATE_RIGHT:
    running = true;
    digitalWrite(DIR_R, HIGH);
    digitalWrite(DIR_L, HIGH);
    analogWrite(PWM_R, speed);
    analogWrite(PWM_L, speed);
    break;

    case BRAKE:
    switch(brake){
      case NORMAL_BRAKE:
      digitalWrite(SIG_LR, HIGH);
      running = false;
      break;

      case REGENERATIVE_BRAKE:
      if(running){
        for(int i=speed; i>= 0; i-=brake_val){
          analogWrite(PWM_L, i);
          analogWrite(PWM_R, i);
          delay(regenerative_interval);
        }
      }
      running = false;
      break;
    }
    break;
  }
}

// Run AGV Function
void Run_AGV(uint8_t agv_mode){
  switch(agv_mode){
    case LF_MODE:
    if(parameter.Running_Dir == FORWARD || parameter.Running_Dir == BACKWARD){
      if(parameter.Running_Dir == FORWARD){
        Calc_PID(FRONT);
        if(millis() - prev_tickC > nfc_read_interval){
          NFC_Handler(FRONT_NFC);
          prev_tickC = millis();
        }
      }
      else{
        Calc_PID(REAR);
        if(millis() - prev_tickC > nfc_read_interval){
          NFC_Handler(REAR_NFC);
          prev_tickC = millis();
        }
      }
      Line_Check();
      Motor_Handler_LF(parameter.Running_Dir, parameter.Running_Accel, parameter.Running_Brake, parameter.Base_Speed);
    }

    else{
      Motor_Handler_LF(parameter.Running_Dir, parameter.Running_Accel, parameter.Running_Brake, parameter.Base_Speed);
    }
    break;

    case LIDAR_MODE:
    break;
  }
}

// Destination To Current Position Check FUnction
void Destination_Handler(void){
  if(turning_decision == TURN_STOP){
    if(parameter.Tag_sign == HOME_SIGN){
      parameter.Current_Pos = HOME;
      Motor_Handler_LF(BRAKE, NORMAL_ACCEL, REGENERATIVE_BRAKE, parameter.Base_Speed);
      delay(1000);
      Line_Search(FRONT);
      parameter.Running_State = PAUSE;
    }

    else if(parameter.Tag_sign == CARRIER_SIGN){
      parameter.Current_Pos = ON_STATION;
      parameter.CurrentPos_Value = parameter.Tag_value;
      delay(1000);
      Motor_Handler_LF(BRAKE, NORMAL_ACCEL, REGENERATIVE_BRAKE, parameter.Base_Speed);
      delay(1000);
      Line_Search(FRONT);
      parameter.Running_State = PAUSE;
    }

    else if(parameter.Tag_sign == STATION_SIGN){
      parameter.Current_Pos = ON_STATION;
      parameter.CurrentPos_Value = parameter.Tag_value;
      Motor_Handler_LF(BRAKE, NORMAL_ACCEL, REGENERATIVE_BRAKE, parameter.Base_Speed);
      digitalWrite(SIG_LR, HIGH);
      parameter.Running_State = PAUSE;
    }
  }
}

// Turning Check Function
void Turn_Check(NFC_Select_t select){
  if(select == FRONT_NFC){
    // DETEKSI AWAL BELOKAN
    if(parameter.Tag_sign != TURN_SIGN && NFC_F.tagType == TURN_SIGN){
      decrease_speedL = NFC_F.tagTypeValue;
      decrease_speedR = NFC_F.tagTypeValue;
    }
    
    // DETEKSI AKHIR BELOKAN
    else if(parameter.Tag_sign == TURN_SIGN && NFC_F.tagType == TURN_SIGN){
      if(millis() - prev_tickE > 10 && decrease_speedR != 0 && decrease_speedL != 0){
        turn_timer_cnt++;

        if(turn_timer_cnt == 2){
          turn_timer_cnt = 0;
          decrease_speedL = 0;
          decrease_speedR = 0;
        }

        prev_tickE = millis();
      }
    }

    // BELOKAN TIDAK TERDETEKSI
    else if(NFC_F.tagType != TURN_SIGN) turn_timer_cnt = 0;

    if(millis() - prev_tickD > 1000 && NFC_F.tagType == NONE_SIGN && decrease_speedR == 0 && decrease_speedL == 0){
      parameter.Tag_sign = NONE_SIGN;
      prev_tickD = millis();
    }
  }
}

// Crossection Check Function
void Cross_Check(NFC_Select_t select){
  if(select == FRONT_NFC){
    // DETEKSI AWAL BELOKAN
    if(parameter.Tag_sign != CROSSECTION_SIGN && NFC_F.tagType == CROSSECTION_SIGN){      
      if(turning_decision == TURN_LEFT){
        decrease_speedL = 30;
        decrease_speedR = 10;
      }
      
      else if(turning_decision == TURN_RIGHT){
        decrease_speedL = 10;
        decrease_speedR = 30;
      }

      else{
        decrease_speedR = 40;
        decrease_speedL = 40;
      }
    }
    
    // DETEKSI AKHIR BELOKAN
    else if(parameter.Tag_sign == CROSSECTION_SIGN && NFC_F.tagType == CROSSECTION_SIGN){
      if(millis() - prev_tickF > 10 && decrease_speedL != 0 && decrease_speedR != 0){
        cross_timer_cnt++;

        if(cross_timer_cnt == 2){
          turning_decision = NONE_TURN;
          cross_timer_cnt = 0;
          decrease_speedR = 0;
          decrease_speedL = 0;
        }

        prev_tickF = millis();
      }
    }

    // BELOKAN TIDAK TERDETEKSI
    else if(NFC_F.tagType != CROSSECTION_SIGN){
      cross_timer_cnt = 0;
    }

    if(turning_decision == TURN_LEFT || turning_decision == TURN_RIGHT){
      if(millis() - prev_turningTick > 3000){
        turning_decision = NONE_TURN;
        prev_turningTick = millis();
      }
    }

    if(millis() - prev_tickD > 1000 && NFC_F.tagType == NONE_SIGN && decrease_speedR == 0 && decrease_speedL == 0){
      parameter.Tag_sign = NONE_SIGN;
      prev_tickD = millis();
    }
  }
}

// PATH PLANNING FUNCTION
bool Run_Path_Planning(void){
  Point start_pos(parameter.Start_coordinateX, parameter.Start_coordinateY);
  Point goal_pos(parameter.Goal_coordinateX, parameter.Goal_coordinateY);

  if(!Run_AStar(start_pos, goal_pos))
    return false;

  else
    Reconstruct_Path(start_pos, goal_pos);
    Reconstruct_PathDir();
    return true;
}

// NODES CHECK FUNCTION
void Nodes_Check(NFC_Select_t select){
  if(select == FRONT_NFC){
    for(int i=0; i<stack_size; i++){
      if(NFC_F.tagPosX == invert_traceBack[i].x && NFC_F.tagPosY == invert_traceBack[i].y && nodes_checked[i].x != invert_traceBack[i].x && nodes_checked[i].y != invert_traceBack[i].y){
        nodes_checked[i] = invert_traceBack[i];

        if(NFC_F.tagType == NONE_SIGN){
          if(path_step[i] == 'f')
            turning_decision = NONE_TURN;

          else if(path_step[i] == 'r')
            turning_decision = TURN_RIGHT;

          else if(path_step[i] == 'l')
            turning_decision = TURN_LEFT;

          else if(path_step[i] == 's')
            turning_decision = TURN_STOP;
        }
        
        else if(NFC_F.tagType == CROSSECTION_SIGN){
          if(path_step[i] == 'f')
            turning_decision = STRAIGHT;

          else if(path_step[i] == 'r')
            turning_decision = TURN_RIGHT;

          else if(path_step[i] == 'l')
            turning_decision = TURN_LEFT;

          else if(path_step[i] == 's')
            turning_decision = TURN_STOP;
        }
      }
    }
  }

  else if(select == REAR_NFC){

  }
}

// NFC Handler Function
void NFC_Handler(NFC_Select_t select){
  if(select == FRONT_NFC){
    digitalWrite(VIR_VCC1, HIGH);
    digitalWrite(VIR_VCC2, LOW);

    NFC_readTag(&NFC_F);
    Nodes_Check(FRONT_NFC);
    Turn_Check(FRONT_NFC);
    Cross_Check(FRONT_NFC);
    Destination_Handler();

    if(NFC_F.tagType != NONE_SIGN){
      parameter.Tag_sign = NFC_F.tagType;
      parameter.Tag_value = NFC_F.tagTypeValue;
      parameter.Tag_num = NFC_F.tagNum;

      parameter.Current_coordinateX = NFC_F.tagPosX;
      parameter.Current_coordinateY = NFC_F.tagPosY;
    }
    else{
      parameter.Current_coordinateX = NFC_F.tagPosX;
      parameter.Current_coordinateY = NFC_F.tagPosY;
    }
  }

  else if(select == REAR_NFC){
    digitalWrite(VIR_VCC1, LOW);
    digitalWrite(VIR_VCC2, HIGH);

    NFC_readTag(&NFC_R);
    Nodes_Check(REAR_NFC);
    Turn_Check(REAR_NFC);
    Cross_Check(REAR_NFC);
    Destination_Handler();

    if(NFC_F.tagType != NONE_SIGN){
      parameter.Tag_sign = NFC_R.tagType;
      parameter.Tag_value = NFC_R.tagTypeValue;
      parameter.Tag_num = NFC_R.tagNum;
    
      parameter.Current_coordinateX = NFC_R.tagPosX;
      parameter.Current_coordinateY = NFC_R.tagPosY;
    }
    else{
      parameter.Current_coordinateX = NFC_R.tagPosX;
      parameter.Current_coordinateY = NFC_R.tagPosY;
    }
  }
}

// NFC Read Data Function
NFC_Status_t NFC_readData(Tag_Data_t *nfc_data){
  uint8_t
  get_uid[7],
  get_data[6],
  keyA[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
  uid_Length,
  read_success;

  read_success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, get_uid, &uid_Length, NFC_Timeout);

  if(read_success && uid_Length == 4){
    for(int i=0; i<uid_Length; i++) nfc_data->uid[i] = get_uid[i];
    read_success = nfc.mifareclassic_AuthenticateBlock(get_uid, uid_Length, 4, 0, keyA);
    
    if(read_success){

      uint8_t data[16];

      read_success = nfc.mifareclassic_ReadDataBlock(4, data);

      if(read_success){
        for(int i=0; i<sizeof(data); i++){
          nfc_data->stored_data[i] = data[i];
        }
        return SUCCESS;
      }
      else return DATA_READ_ERROR;
    }
    else return BLOCK_AUTH_ERROR;
  }
  else return UID_READ_ERROR;
}

// NFC Tag Read Function
NFC_Status_t NFC_readTag(Tag_Data_t *nfc_data){
  if(NFC_readData(nfc_data) == SUCCESS){
    if(nfc_data->stored_data[0] == DATA_AUTH_HEADER && nfc_data->stored_data[1] == DATA_AUTH_HEADER){
      nfc_data->tagType = nfc_data->stored_data[2];
      nfc_data->tagTypeValue = (nfc_data->stored_data[3] >> 8) | nfc_data->stored_data[4];
      nfc_data->tagNum = (nfc_data->stored_data[5] >> 8) | nfc_data->stored_data[6];
      nfc_data->tagPosX = nfc_data->stored_data[7];
      nfc_data->tagPosY = nfc_data->stored_data[8];

      for(int i=0; i<sizeof(nfc_data->new_uid); i++){
        if(i<4) nfc_data->new_uid[i] = nfc_data->uid[i];
        else nfc_data->new_uid[i] = nfc_data->stored_data[i-4];
      }
      return SUCCESS;
    }
    else{
      nfc_data->tagType = NONE_SIGN;
      nfc_data->tagTypeValue = 0;
      nfc_data->tagNum = 0;
      return UNREGISTERED_TAG;
    }
  } 
  else{
    nfc_data->tagType = NONE_SIGN;
    nfc_data->tagTypeValue = 0;
    nfc_data->tagNum = 0;
    return TAG_READ_ERROR;
  }
}

// Removes And Returns Cell With Lowest 'F-Cost' from openset struct
Point Return_OpenSet(void){
  Point p = openSet[0];
  for (int i = 1; i < openSetSize; i++) {
      openSet[i - 1] = openSet[i];
      openSetF[i - 1] = openSetF[i];
  }
  openSetSize--;
  return p;
}

// Calculate Calc_Heuristic Value Function
int Calc_Heuristic(Point a, Point b){
  return abs(a.x - b.x) + abs(a.y - b.y);
}

// New Cell Point Insert Function
void Insert_OpenSet(Point p, int f){
int i = openSetSize - 1;
  while (i >= 0 && openSetF[i] > f) {
      openSet[i + 1] = openSet[i];
      openSetF[i + 1] = openSetF[i];
      i--;
  }
  openSet[i + 1] = p;
  openSetF[i + 1] = f;
  openSetSize++;
}
 
// A-Star Algorithm Function
bool Run_AStar(Point start, Point goal){
  int gScore[gridWidth][gridHeight];
  for (int i = 0; i < gridWidth; i++)
      for (int j = 0; j < gridHeight; j++)
          gScore[i][j] = INT8_MAX;

  gScore[start.x][start.y] = 0;
  Insert_OpenSet(start, Calc_Heuristic(start, goal));

  while (openSetSize > 0) {
    Point current = Return_OpenSet();

    if (current.x == goal.x && current.y == goal.y)
      return true;

    const int dx[] = {0, 1, 0, -1};
    const int dy[] = {1, 0, -1, 0};

    for (int i = 0; i < 4; i++) {
      Point neighbor(current.x + dx[i], current.y + dy[i]);

      if (neighbor.x < 0 || neighbor.x >= gridWidth || neighbor.y < 0 || neighbor.y >= gridHeight)
          continue;
      if (grid[neighbor.x][neighbor.y] == 1)
          continue;

      int tentative_gScore = gScore[current.x][current.y] + cellSize;
      if (tentative_gScore < gScore[neighbor.x][neighbor.y]) {
        gScore[neighbor.x][neighbor.y] = tentative_gScore;
        int fScore = tentative_gScore + Calc_Heuristic(neighbor, goal);
        Insert_OpenSet(neighbor, fScore);
        cameFrom[neighbor.x][neighbor.y] = current;
      }
    }
  }
  return false;
}

// RECONSTRUCT PATH FUNCTION
void Reconstruct_Path(Point start, Point goal){
  Point current = goal;

  while (!(current.x == start.x && current.y == start.y)) {
    traceBack[step_stack] = current;
    current = cameFrom[current.x][current.y];
    step_stack++;
  }

  if(current.x == start.x && current.y == start.y)
    traceBack[step_stack] = current;

  for(int i=0; i<=step_stack; i++)
    invert_traceBack[i] = traceBack[step_stack-i];

  stack_size = step_stack;
  step_stack = 0;
}

// GET RECONSTRUCT PATH DIRECTION FUNCTION
void Reconstruct_PathDir(void){
  Point prev[stack_size];

  for(int i=0; i<stack_size; i++){
    if(head_dir == HEAD_YN){
      if(invert_traceBack[i+1].y - invert_traceBack[i].y == 1) path_step[i] = 'f';
      else if(invert_traceBack[i+1].x - invert_traceBack[i].x == 1){
        path_step[i] = 'l';
        head_dir = HEAD_XP;
      }
      else if(invert_traceBack[i+1].x - invert_traceBack[i].x == -1){
        path_step[i] = 'r';
        head_dir = HEAD_XN;
      }
    }

    else if(head_dir == HEAD_YP){
      if(invert_traceBack[i+1].y - invert_traceBack[i].y == -1) path_step[i] = 'f';
      else if(invert_traceBack[i+1].x - invert_traceBack[i].x == -1){
        path_step[i] = 'l';
        head_dir = HEAD_XN;
      }
      else if(invert_traceBack[i+1].x - invert_traceBack[i].x == 1){
        path_step[i] = 'r';
        head_dir = HEAD_XP;
      }
    }

    else if(head_dir == HEAD_XN){
      if(invert_traceBack[i+1].x - invert_traceBack[i].x == -1) path_step[i] = 'f';
      else if(invert_traceBack[i+1].y - invert_traceBack[i].y == -1){
        path_step[i] = 'r';
        head_dir = HEAD_YP;
      }
      else if(invert_traceBack[i+1].y - invert_traceBack[i].y == 1){
        path_step[i] = 'l';
        head_dir = HEAD_YN;
      }
    }

    else if(head_dir == HEAD_XP){
      if(invert_traceBack[i+1].x - invert_traceBack[i].x == 1) path_step[i] = 'f';
      else if(invert_traceBack[i+1].y - invert_traceBack[i].y == 1){
        path_step[i] = 'r';
        head_dir = HEAD_YN;
      }
      else if(invert_traceBack[i+1].y - invert_traceBack[i].y == -1){
        path_step[i] = 'l';
        head_dir = HEAD_YP;
      }
    }
  }
}