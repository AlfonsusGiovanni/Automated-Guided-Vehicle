/*
  Automated Guided Vehicle - Arduino Main Program
  PT. Stechoq Robotika Indonesia

  Data    : 18 Oktober 2024
  Author  : Alfonsus Giovanni Mahendra Puta - Universitas Diponegoro
*/

/*User Private Include*/
#include "Wire.h"
#include "DFRobot_BNO055.h"
#include "Adafruit_PN532.h"
#include "PID_driver.h"
#include "CustomSerial.h"
#include "Odo_Localization.h"
#include "Motor_Driver.h"

/*User Private Define*/
#define SENS_SWITCH 2

#define ENA_L       5    // Left Motor Run and Brake Pin (EL) <-PCB_PIN
#define PWM_L       6    // Left Motor Speed PWM (VR) <- PCB PIN
#define DIR_L       7    // Left Motor Direction (Z/F) <- PCB PIN
#define ENA_R       8    // Right Motor Run and Brake Pin (EL) <-PCB_PIN
#define PWM_R       9    // Right Motor Speed PWM (VR) <- PCB PIN
#define DIR_R       10   // Right Motor Direction (Z/F) <- PCB PIN

#define SIG_LR      11   // Left Right Motor Start Stop (SIGNAL) <-PCB PIN
#endif

#define ENCL_A    55
#define ENCL_B    14
#define ENCL_C    15

#define ENCR_A    55
#define ENCR_B    16
#define ENCR_C    17

#define PROX_REAR  12
#define PROX_FRONT 13

#define PROX_FRONT  A9 
#define PROX_REAR   A10

#define IRQ_NFC1    14
#define IRQ_NFC2    16
#define VIR_VCC1    15
#define VIR_VCC2    17

#define SERVO_HOOK  38
#define LS1         39
#define LS2         40

#define START_BTN       41
#define SPEAKER_PIN     42
#define PILOTLAMP_PIN   43

#define VOLTAGE_SENS_PIN  A14
#define CURRENT_SENS_PIN  A15

#define SENS_COUNT 2
#define SENS_NUM  16
#define DATA_AUTH_HEADER    0xFF

/*User Private Variable*/
const uint8_t
sens_pin[SENS_COUNT][SENS_NUM] = {53, 51, 49, 47, 45, 43, 41, 39, 37, 35, 33, 31, 29, 27, 25, 23};

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
hook_state,
prev_dir;

uint8_t
NFC_Timeout = 25,
brake_val = 10,
decrease_speedL,
decrease_speedR,
balance_speedL,
balance_speedR;

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
prev_goalX, prev_goalY,
path_step[gridWidth*gridHeight];

uint16_t
sens_value,
sens_data;

uint32_t
Left_PulseA, Left_PulseB, Left_PulseC, prev_Left_PulseC,
Right_PulseA, Right_PulseB, Right_PulseC, prev_Right_PulseC;

int16_t
L_speed,
R_speed;

int
total_weight,
sens_count;

const float
VCC = 5.00,
cutOffLimit = 1.00,
sensitivity = 40.0,
quiescent_Output_voltage = 0.5,
FACTOR = 0.04,
QOV = 2.5,
cutOff = 0.04;

float
line_pos,
lf_pid_val,
balancer_pid_val,
bat_voltage,
bat_current,
bat_power;

double
current_heading,
prev_heading,
update_heading,
full_rotation_dist,
current_distance,
current_position,
prev_position;

unsigned long
prev_dummytick1,  // -> Dummy Testing Tick 1
prev_dummytick2,  // -> Dummy Testing Tick 2
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
path_planned = false,
head_aligned = false,
end_pos = false,
independent_inputspeed = false;

String
Tag_Start,
Tag_Destination;

// int grid[gridHeight][gridWidth] = {
//   {0, 1, 1, 1, 1, 1, 1, 0},
//   {0, 1, 1, 1, 1, 1, 1, 0},
//   {0, 0, 0, 0, 0, 0, 0, 0},
//   {0, 1, 0, 1, 1, 0, 1, 0},
//   {0, 1, 0, 1, 1, 0, 1, 0},
//   {1, 1, 0, 1, 1, 0, 1, 1},
//   {1, 1, 0, 1, 1, 0, 1, 1},
//   {1, 1, 0, 0, 0, 0, 1, 1},
// };

int grid[gridHeight][gridWidth] = {
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0},
};

int 
openSetSize = 0,
openSetF[gridWidth * gridHeight];

/*User Private Typedef*/
struct Point{
  int x, y;
  Point(int x = 0, int y = 0) : x(x), y(y) {}
};

Point openSet[gridWidth * gridHeight];          // Open all available nodes
Point cameFrom[gridWidth][gridHeight];          // Save current pos to nodes
Point traceBack[gridWidth * gridHeight];        // Traceback nodes pattern from goal to start
Point invert_traceBack[gridWidth * gridHeight]; // Invert traceback value
Point nodes_checked[gridWidth * gridHeight];    // Check nodes detected

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
  FRONT = 0x01,
  REAR
}Sens_sel_t;

typedef enum{
  DOWN = 0x01,
  UP,
}Hook_dir_t;

typedef enum{
  EUL_ROLL,
  EUL_PITCH,
  EUL_YAW,
}Gyro_Eul_t;

typedef enum{
  ENC_MODE,
  GYRO_MODE
}Balancer_Mode_t;

/*User Private Typedef Init*/
Tag_Data_t NFC_F;
Tag_Data_t NFC_R;

Param_t parameter;

PIDController pid_agv_f;      // -> PID Jalan Maju
PIDController pid_agv_b;      // -> PID Jalan Mundur
PIDController pid_odo;        // -> PID Odometry
PIDController pid_gyro_corr;  // -> PID Odometry Gyro Correction

typedef DFRobot_BNO055_IIC  BNO;
BNO::sEulAnalog_t eul_value;

/*User Private Class Init*/
Adafruit_PN532 nfc(IRQ_NFC1, 3);
Odometry odometry(150, 413, 60);
BNO bno(&Wire, 0x28);
Motor_Driver L_Motor1(80, 100, ACTIVE_LOW);   // LF LEFT MOTOR
Motor_Driver R_Motor1(80, 100, ACTIVE_LOW);   // LF RIGHT MOTOR
Motor_Driver L_Motor2(80, 100, ACTIVE_HIGH);  // LIDAR LEFT MOTOR
Motor_Driver R_Motor2(80, 100, ACTIVE_HIGH);  // LIDAR RIGHT MOTOR

/*User Private Function Declaration*/
void Read_Sens(Sens_sel_t sensor_sel);
void Calc_LF_PID(Sens_sel_t sensor_sel);
void Calc_Balancer_PID(Balancer_Mode_t sel_mode);
void Line_Check(uint8_t mode);
bool Line_Search(Sens_sel_t sensor_sel);
void PinHook_Handler(Hook_dir_t dir);
void Motor_Handler(uint8_t mode, uint8_t direction, uint8_t accel, uint8_t brake, uint8_t speed);
void Run_AGV(uint8_t agv_mode);
void Destination_Handler(void);
void Custom_Destination_Handler(void);

bool Station_Handler(void);

void NFC_Handler(NFC_Select_t select);
void Turn_Check(NFC_Select_t select);
void Cross_Check(NFC_Select_t select);
void Station_Check(NFC_Select_t select);
void Nodes_check(NFC_Select_t select);
void Run_Path_Planning(void);

bool Read_Proximity(Sens_sel_t sensor_sel);
float Read_Gyro(Gyro_Eul_t sel_angle);
float Check_Battery_Cappacity(void);

NFC_Status_t NFC_readData(Tag_Data_t *nfc);
NFC_Status_t NFC_readTag(Tag_Data_t *nfc);

Point Return_OpenSet(void);
int Calc_Heuristic(Point a, Point b);
void Insert_OpenSet(Point p, int f);
bool Run_AStar(Point start, Point goal);
void Reconstruct_Path(Point start, Point goal, char displayGrid[gridWidth][gridHeight]);
void Reconstruct_PathDir(void);

void Change_Heading(double start_heading, double end_heading, double tollerance);
void GoTo(double start_position, double end_position, double tollerance);

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

  pinMode(CURRENT_SENS_PIN, INPUT);
  pinMode(VOLTAGE_SENS_PIN, INPUT);

  #ifdef USE_OLD_PINOUT
    digitalWrite(SIG_LR, HIGH);
    delay(20);
    digitalWrite(ENA_R, LOW);
    digitalWrite(ENA_L, LOW);
    delay(20);
    analogWrite(PWM_R, 0);
    analogWrite(PWM_L, 0);
    delay(20);
  #endif

  // Interface Pin Setup
  pinMode(START_BTN, INPUT_PULLUP);
  pinMode(SPEAKER_PIN, OUTPUT);
  pinMode(PILOTLAMP_PIN, OUTPUT);

  // Control Pin Setup
  #ifdef USE_NEW_PINOUT
    pinMode(SERVO_HOOK, OUTPUT);
    pinMode(LS1, INPUT_PULLUP);
    pinMode(LS2, INPUT_PULLUP);
  #endif

  pinMode(SENS_SWITCH, OUTPUT);
  digitalWrite(SENS_SWITCH, LOW);

  // Self Setup
  parameter.Running_Mode = NOT_SET,
  parameter.Base_Speed = 80;
  parameter.Running_State = STOP;
  parameter.Running_Accel = REGENERATIVE_ACCEL;
  parameter.Running_Brake = REGENERATIVE_BRAKE;
  parameter.Current_Pos = HOME;
  parameter.SensorA_Status = NOT_DETECTED;
  parameter.SensorB_Status = NOT_DETECTED;
  parameter.Tag_sign = NONE_SIGN;

  // PID LF Setup
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

  pid_agv_b.Kp        = 1.115;
  pid_agv_b.Ki        = 0.001;       
  pid_agv_b.Kd        = 0.035; 		
  pid_agv_b.tau       = 0.01;
	pid_agv_b.limMax    = 100;     
  pid_agv_b.limMin    = -100;     
  pid_agv_b.limMaxInt = 5.0; 	   
  pid_agv_b.limMinInt = -5.0;
	pid_agv_b.T_sample  = 0.01;
  PIDController_Init(&pid_agv_b);

  // PID Odometry Setup
  pid_odo.Kp          = 2.5;
  pid_odo.Ki          = 0.00;       
  pid_odo.Kd          = 0.00; 		
  pid_odo.tau         = 0.01;
	pid_odo.limMax      = 50.00;     
  pid_odo.limMin      = -50.00;     
  pid_odo.limMaxInt   = 5.00; 	   
  pid_odo.limMinInt   = -5.00;
	pid_odo.T_sample    = 0.01;
  PIDController_Init(&pid_odo);

  // PID Gyro Correction Setup
  pid_gyro_corr.Kp          = 0.00;
  pid_gyro_corr.Ki          = 0.00;       
  pid_gyro_corr.Kd          = 0.00; 		
  pid_gyro_corr.tau         = 0.01;
	pid_gyro_corr.limMax      = 50.00;     
  pid_gyro_corr.limMin      = -50.00;     
  pid_gyro_corr.limMaxInt   = 5.00; 	   
  pid_gyro_corr.limMinInt   = -5.00;
	pid_gyro_corr.T_sample    = 0.01;
  PIDController_Init(&pid_gyro_corr);

  // NFC_F.tagType = HOME_SIGN;
  // parameter.Running_Mode = LF_MODE;
  parameter.Running_Mode = LIDAR_MODE;

  while(1){
    digitalWrite(PILOTLAMP_PIN, LOW);
    //Receive_Serial(&parameter);

    if(parameter.Running_Mode == LF_MODE){
      // Motor Pin Setup
      L_Motor1.driver_Pinset(ENA_L, PWM_L, DIR_L, SIG_LR);
      R_Motor1.driver_Pinset(ENA_R, PWM_R, DIR_R, SIG_LR);

      // NFC Pin Setup
      pinMode(IRQ_NFC1, OUTPUT);
      pinMode(IRQ_NFC2, OUTPUT);
      pinMode(VIR_VCC1, OUTPUT);
      pinMode(VIR_VCC2, OUTPUT);

      // NFC Setup
      nfc.begin();
      nfc.SAMConfig();

      digitalWrite(PILOTLAMP_PIN, HIGH);
      delay(500);
      digitalWrite(PILOTLAMP_PIN, LOW);
      break;
    }

    else if(parameter.Running_Mode == LIDAR_MODE){
      // Motor Pin Setup
      L_Motor2.driver_Pinset(ENA_L, PWM_L, DIR_L, SIG_LR);
      R_Motor2.driver_Pinset(ENA_R, PWM_R, DIR_R, SIG_LR);

      // Encoder Pin Setup
      odometry.enc_Pinset(ENCL_A, ENCL_B, ENCL_C, ENCR_A, ENCR_B, ENCR_C);

      // BNO Setup
      /*
      bno.reset();
      while(bno.begin() != BNO::eStatusOK) {
        Serial.println("bno begin faild");
        delay(1000);
      }
      Serial.println("bno begin success");
      */

      digitalWrite(PILOTLAMP_PIN, HIGH);
      delay(500);
      digitalWrite(PILOTLAMP_PIN, LOW);
      break;
    }
    else continue;
  }

  while(1){
    Serial.println(digitalRead(START_BTN));
    if(digitalRead(START_BTN) == LOW) break;
    else continue;
  }
}

void loop(){
  digitalWrite(PILOTLAMP_PIN, HIGH);

  #ifdef TEST_PROXIMITY 
    front_state = Read_Proximity(FRONT);
    rear_state = Read_Proximity(REAR);

    Serial.print("State 1: ");
    Serial.print(front_state);
    Serial.print("\t");
    Serial.print("State 2: ");
    Serial.println(rear_state);
  #endif

  #ifdef TEST_BATTERY
    Check_Battery_Cappacity();
    Serial.print("Voltage: ");
    Serial.print(bat_voltage);
    Serial.print("\t");
    Serial.print("Current: ");
    Serial.print(bat_current);
    Serial.print("\t");
    Serial.print("Power: ");
    Serial.println(bat_power);

    delay(500);
  #endif

  #ifdef TEST_NFC
    NFC_Handler(FRONT_NFC);

    Serial.print("NFC Data: ");
    for(int i=0; i<16; i++){
      Serial.print("0x");
      Serial.print(NFC_F.stored_data[i], HEX);
      if(i<15) Serial.print("-");
    }
    Serial.println(" ");

    delay(250);
  #endif

  #ifdef TEST_PINHOOK
    Serial.print(digitalRead(LS1));
    Serial.print(" ");
    Serial.println(digitalRead(LS2));

    PinHook_Handler(UP);
    delay(2000);
    PinHook_Handler(DOWN);
    delay(2000);
  #endif

  #ifdef TEST_ENCODER
    Motor_Handler(LIDAR_MODE, FORWARD, NORMAL_ACCEL, NORMAL_BRAKE, 80);

    Serial.print(digitalRead(ENCR_A));
    Serial.print("\t");

    Serial.print(digitalRead(ENCR_B) + 2);
    Serial.print("\t");

    Serial.println(digitalRead(ENCR_C) + 4);
  #endif

  #ifdef TEST_ODOMETRY
    // GoTo(0, 1500, 150);
    Motor_Handler(LIDAR_MODE, FORWARD, NORMAL_ACCEL, REGENERATIVE_BRAKE, 50);

    Serial.print("Left Counter: ");
    Serial.print(odometry.wl_counter);
    Serial.print("  ");

    Serial.print("Right Counter: ");
    Serial.print(odometry.wr_counter);
    Serial.print("  ");

    Serial.print("Distance: ");
    Serial.println(current_position);
  #endif

  #ifdef TEST_ODOMETRY_ROTASI
  digitalWrite(SPEAKER_PIN, HIGH);
  Change_Heading(0, 90, 12);
  eul_value = bno.getEul();
  Serial.print(" head: "); Serial.print(eul_value.head); Serial.print(" roll: "); Serial.print(eul_value.roll);  Serial.print(" pitch: "); Serial.println(eul_value.pitch);
  #endif

  #ifdef TEST_LINE
    Motor_Handler(LF_MODE, BACKWARD, NORMAL_ACCEL, REGENERATIVE_BRAKE, 80);
    delay(2000);
    Motor_Handler(LF_MODE, BRAKE, NORMAL_ACCEL, NORMAL_BRAKE, 80);
    delay(5000);
  #endif

  #ifdef TEST_SERIAL
    if(!config_done){
      digitalWrite(LED_BUILTIN, LOW);
      Receive_Serial(&parameter);

      if(parameter.Running_Mode != NOT_SET && parameter.Running_State == START){
        prev_dummytick1 = millis();
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

      if(millis() - prev_dummytick1 > 5000){
        parameter.Running_Mode = NOT_SET;
        parameter.Running_State = STOP;
        config_done = false;
      }
    }

    parameter.Current_Pos = 1;
    parameter.CurrentPos_Value = 1000;
    parameter.Tag_sign = 1;
    parameter.Tag_value = 500;
    parameter.Tag_num = 425;

    parameter.Current_coordinateX = 8;
    parameter.Current_coordinateX = 8;
    
    parameter.Current_Pos = ON_THE_WAY;
    parameter.Send_counter = 10;
    parameter.Pickup_counter = 11;
    parameter.Battery_level = 90.5;
    Transmit_Serial(&parameter);

    delay(500);
  #endif

  #ifdef TEST_MAIN
    if(!config_done){
      Receive_Serial(&parameter);
      btn_pressed = !digitalRead(START_BTN);

      if(parameter.Running_Mode == LF_MODE){
        Read_Sens(FRONT);
        NFC_readTag(&NFC_F);

        // Check if there is no line and home tag detected
        if(!line_detected && NFC_F.tagType != HOME_SIGN){
          if(millis() - prev_dummytick1 > 150){
            if(digitalRead(PILOTLAMP_PIN) == LOW) digitalWrite(PILOTLAMP_PIN, HIGH);
            else digitalWrite(PILOTLAMP_PIN, LOW);
            prev_dummytick1 = millis();
          }
        }

        // Check if line is detected but home tag is not
        else if(line_detected && NFC_F.tagType != HOME_SIGN){
          if(millis() - prev_dummytick1 > 500){
            if(digitalRead(PILOTLAMP_PIN) == LOW) digitalWrite(PILOTLAMP_PIN, HIGH);
            else digitalWrite(PILOTLAMP_PIN, LOW);
            prev_dummytick1 = millis();
          }
          head_dir = HEAD_XP;
        }

        // All needed state is accomplished
        else if(line_detected && NFC_F.tagType == HOME_SIGN){
          digitalWrite(PILOTLAMP_PIN, HIGH);

          if(parameter.Running_State == START){
            digitalWrite(PILOTLAMP_PIN, LOW);
            Serial.flush();
            delay(1000);
            config_done = true;
          }

          else if(btn_pressed){
            parameter.Running_State = START;
            parameter.Running_Dir = FORWARD;
            
            digitalWrite(PILOTLAMP_PIN, LOW);
            delay(1000);
            parameter.Running_State = START;
            config_done = true;
          }
        }
      }

      else if(parameter.Running_Mode == LIDAR_MODE){
        digitalWrite(PILOTLAMP_PIN, HIGH);

        if(parameter.Running_State == START){
          digitalWrite(PILOTLAMP_PIN, LOW);
          Serial.flush();
          delay(1000);
          config_done = true;
        }

        else if(btn_pressed){
          parameter.Running_State = START;
          parameter.Running_Dir = FORWARD;
          
          digitalWrite(PILOTLAMP_PIN, LOW);
          delay(1000);
          parameter.Running_State = START;
          odometry.reset_Pos();
          config_done = true;
        }
      }

      else{
        digitalWrite(PILOTLAMP_PIN, LOW);
      }
    }

    else if(config_done){
      Check_Battery_Cappacity();

      switch(parameter.Running_State){
        case START:
        parameter.Current_Pos = ON_THE_WAY;
        Transmit_Serial(&parameter);

        digitalWrite(PILOTLAMP_PIN, HIGH);
        Run_AGV(parameter.Running_Mode);
        break;

        case STOP:
        if(millis() > prev_dummytick1 > 500){
          if(digitalRead(PILOTLAMP_PIN) == LOW) digitalWrite(PILOTLAMP_PIN, HIGH);
          else digitalWrite(PILOTLAMP_PIN, LOW);
          prev_dummytick1 = millis();
        }

        Motor_Handler(parameter.Running_Mode, BRAKE, NORMAL_ACCEL, REGENERATIVE_BRAKE, parameter.Base_Speed);
        break;

        case PAUSE:
        if(millis() > prev_tickComrx > 100){
          Receive_Serial(&parameter);
          prev_tickComrx = millis();
        }
        break;
      }
    }
 #endif
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

// PID LF Calculation Function
void Calc_LF_PID(Sens_sel_t sensor_sel){
  Read_Sens(sensor_sel);

  if(sensor_sel == FRONT){
    PIDController_Update(&pid_agv_f, 0, line_pos);
    lf_pid_val = pid_agv_f.out;
  }
  else{
    PIDController_Update(&pid_agv_b, 0, line_pos);
    lf_pid_val = pid_agv_b.out;
  }
}

// PID Balancer Calculation Function
void Calc_Balancer_PID(Balancer_Mode_t sel_mode){
  double start_angle, current_angle;

  if(start_angle == 0)
    start_angle = Read_Gyro(EUL_YAW);

  else
    start_angle = start_angle;

  if(sel_mode == ENC_MODE){
    PIDController_Update(&pid_odo, 0, (odometry.wl_counter - odometry.wr_counter));
    balancer_pid_val = pid_odo.out;
  }

  else if(sel_mode == GYRO_MODE){
    current_angle = Read_Gyro(EUL_YAW);

    PIDController_Update(&pid_gyro_corr, start_angle, current_angle);
    balancer_pid_val = pid_gyro_corr.out;
  }
}

// Line Check Subroutine Function
void Line_Check(uint8_t mode){
  if(millis()-prev_tickA > line_false_interval && !line_detected){
    line_false_cnt++;
    prev_tickA = millis();
  }
  else if (line_detected) line_false_cnt = 0;

  if(line_false_cnt >= 5) no_line = true;

  switch(mode){
    case 0:
    if(no_line){
      Motor_Handler(parameter.Running_Mode, BRAKE, NORMAL_ACCEL, REGENERATIVE_BRAKE, parameter.Base_Speed);
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
    break;

    case 1:
    if(no_line){
      Motor_Handler(parameter.Running_Mode, BRAKE, NORMAL_ACCEL, REGENERATIVE_BRAKE, parameter.Base_Speed);
      parameter.Running_State = PAUSE;
      line_false_cnt = 0;
      no_line = false;
      config_done = false;
    }
    break;
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
    Motor_Handler(parameter.Running_Mode, direction, NORMAL_ACCEL, NORMAL_BRAKE, 30);
    if(line_pos >= -20 && line_pos <= 20 && line_detected){
      Motor_Handler(parameter.Running_Mode, BRAKE, NORMAL_ACCEL, NORMAL_BRAKE, 30);
      break;
    }
  }
  return true;
}

// Pin Hook Handler Function
void PinHook_Handler(Hook_dir_t dir){
  if(dir != prev_dir) hook_state = 0;

  while(hook_state == 0){
    if(dir == DOWN && hook_state == 0){
      if(digitalRead(LS1) == HIGH) digitalWrite(SERVO_HOOK, HIGH);
      else{
        hook_state = 1;
        digitalWrite(SERVO_HOOK, LOW);
      }
    }
    else if(dir == UP && hook_state == 0){
      if(!digitalRead(LS2) == HIGH) digitalWrite(SERVO_HOOK, HIGH);
      else{
        hook_state = 1;
        digitalWrite(SERVO_HOOK, LOW);
      }
    }
    prev_dir = dir;
  }
}

// Proximity Sensor Read Function
bool Read_Proximity(Sens_sel_t sensor_sel){
  bool value;
  if(sensor_sel == FRONT) value = !digitalRead(PROX_FRONT);
  else if(sensor_sel == REAR) value = !digitalRead(PROX_REAR);
  return value;
}

// Read Gyro Euller Angle
float Read_Gyro(Gyro_Eul_t sel_angle){
  BNO::sEulAnalog_t angle = bno.getEul();

  if(sel_angle == EUL_ROLL)
    return (float)angle.roll;

  else if(sel_angle == EUL_PITCH)
    return (float)angle.pitch;

  else if(sel_angle == EUL_YAW)
    return (float)angle.head;
}

// Check Battery Cappacity
float Check_Battery_Cappacity(void){
  float raw_current = (5.0/1023) * analogRead(CURRENT_SENS_PIN);
  bat_current = (raw_current - QOV + 0.007);

  float raw_voltage = analogRead(VOLTAGE_SENS_PIN) * (5.0/1024) * 5.35;
  bat_voltage = (raw_voltage * (4700 + 1100) / 1100);

  bat_power = bat_voltage * bat_current;
}

// LF Motor Handler Function
void Motor_Handler(uint8_t mode, uint8_t direction, uint8_t accel, uint8_t brake, uint8_t speed){
  uint8_t 
  left_tolerance = 0,
  right_tolerance = 0;

  balance_speedL = parameter.Left_Speed;
  balance_speedR = parameter.Right_Speed;

  if(balance_speedL > 0 || balance_speedR > 0) independent_inputspeed = true;
  else independent_inputspeed = false;

  switch(direction){
    case FORWARD:
    running = false;
    
    if(mode == LF_MODE){
      L_Motor1.motor_Run();
      R_Motor1.motor_Run();

      L_Motor1.driver_Enable();
      R_Motor1.driver_Enable();

      Calc_LF_PID(FRONT);

      L_Motor1.set_Dir(CCW);
      R_Motor1.set_Dir(CW);
    }

    else if(mode == LIDAR_MODE){
      L_Motor2.driver_Enable();
      R_Motor2.driver_Enable();
     
      L_Motor2.set_Dir(CCW);
      R_Motor2.set_Dir(CW);
    }

    switch(accel){
      case NORMAL_ACCEL:
      if(mode == LF_MODE){
        L_speed = speed - lf_pid_val - decrease_speedL;
        R_speed = speed + lf_pid_val - decrease_speedR;

        L_Motor1.set_Speed(L_speed);
        R_Motor1.set_Speed(R_speed);
      }

      else if(mode == LIDAR_MODE){
        if(!independent_inputspeed){
          L_speed = speed  - balancer_pid_val;
          R_speed = speed  + balancer_pid_val;
        }
        else{
          L_speed = balance_speedL;
          R_speed = balance_speedR;
        }

        L_Motor2.set_Speed(L_speed);
        R_Motor2.set_Speed(R_speed);
      }
      break;

      case REGENERATIVE_ACCEL:
      if(millis()-prev_tickB > regenerative_interval && speed_cnt < speed && !running){
        speed_cnt++;

        if(mode == LF_MODE){
          L_speed = speed_cnt - lf_pid_val - decrease_speedL;
          R_speed = speed_cnt + lf_pid_val - decrease_speedR;

          L_Motor1.set_Speed(L_speed);
          R_Motor1.set_Speed(R_speed);
        }

        else if(mode == LIDAR_MODE){
          L_speed = speed_cnt - balancer_pid_val;
          R_speed = speed_cnt + balancer_pid_val;

          L_Motor2.set_Speed(L_speed);
          R_Motor2.set_Speed(R_speed);
        }

        prev_tickB = millis();
      }
      else if(speed_cnt == speed) running = true;

      if(running){
        if(mode == LF_MODE){
          L_speed = speed - lf_pid_val - decrease_speedL;
          R_speed = speed + lf_pid_val - decrease_speedR;

          L_Motor1.set_Speed(L_speed);
          R_Motor1.set_Speed(R_speed);
        }

        else if(mode == LIDAR_MODE){
          if(!independent_inputspeed){
            L_speed = speed  - balancer_pid_val;
            R_speed = speed  + balancer_pid_val;
          }
          else{
            L_speed = balance_speedL;
            R_speed = balance_speedR;
          }

          L_Motor2.set_Speed(L_speed);
          R_Motor2.set_Speed(R_speed);
        }

        speed_cnt = 0;
      }
      break;
    }
    break;

    case BACKWARD:
    running = true;

    if(mode == LF_MODE){
      L_Motor1.motor_Run();
      R_Motor1.motor_Run();

      L_Motor1.driver_Enable();
      R_Motor1.driver_Enable();

      Calc_LF_PID(REAR);

      L_Motor1.set_Dir(CW);
      R_Motor1.set_Dir(CCW);
    }

    else if(mode == LIDAR_MODE){
      L_Motor2.driver_Enable();
      R_Motor2.driver_Enable();
     
      L_Motor2.set_Dir(CW);
      R_Motor2.set_Dir(CCW);
    }

    switch(accel){
      case NORMAL_ACCEL:
      if(mode == LF_MODE){
        L_speed = speed - lf_pid_val - decrease_speedL;
        R_speed = speed + lf_pid_val - decrease_speedR;

        L_Motor1.set_Speed(L_speed);
        R_Motor1.set_Speed(R_speed);
      }

      else if(mode == LIDAR_MODE){
        if(!independent_inputspeed){
          L_speed = speed  + balancer_pid_val;
          R_speed = speed  - balancer_pid_val;
        }
        else{
          L_speed = balance_speedL;
          R_speed = balance_speedR;
        }

        L_Motor2.set_Speed(L_speed);
        R_Motor2.set_Speed(R_speed);
      }
      break;

      case REGENERATIVE_ACCEL:
      if(millis()-prev_tickB > regenerative_interval && speed_cnt < speed && !running){
        speed_cnt++;
        
        if(mode == LF_MODE){
          L_speed = speed_cnt - lf_pid_val - decrease_speedL;
          R_speed = speed_cnt + lf_pid_val - decrease_speedR;

          L_Motor1.set_Speed(L_speed);
          R_Motor1.set_Speed(R_speed);
        }

        else if(mode == LIDAR_MODE){
          L_speed = speed_cnt + balancer_pid_val;
          R_speed = speed_cnt - balancer_pid_val;

          L_Motor2.set_Speed(L_speed);
          R_Motor2.set_Speed(R_speed);
        }

        prev_tickB = millis();
      }
      else if(speed_cnt == speed) running = true;

      if(running){
        if(mode == LF_MODE){
          L_speed = speed - lf_pid_val - decrease_speedL;
          R_speed = speed + lf_pid_val - decrease_speedR;

          L_Motor1.set_Speed(L_speed);
          R_Motor1.set_Speed(R_speed);
        }

        else if(mode == LIDAR_MODE){
          L_speed = speed + balancer_pid_val;
          R_speed = speed - balancer_pid_val;

          L_Motor2.set_Speed(L_speed);
          R_Motor2.set_Speed(R_speed);
        }

        speed_cnt = 0;
      }
      break;
    }
    break;

    case LEFT:
    running = true;

    if(mode == LF_MODE){
      L_Motor1.motor_Run();
      R_Motor1.motor_Run();

      L_Motor1.driver_Disable();
      R_Motor1.driver_Enable();

      R_Motor1.set_Dir(CW);
      R_Motor1.set_Speed(speed);
    }

    else if(mode == LIDAR_MODE){
      L_Motor2.driver_Disable();
      R_Motor2.driver_Enable();

      R_Motor2.set_Dir(CW);
      R_Motor2.set_Speed(speed);
    }
    break;
  
    case RIGHT:
    running = true;

    if(mode == LF_MODE){
      L_Motor1.motor_Run();
      R_Motor1.motor_Run();

      L_Motor1.driver_Enable();
      R_Motor1.driver_Disable();

      L_Motor1.set_Dir(CCW);
      L_Motor1.set_Speed(speed);
    }

    else if(mode == LIDAR_MODE){
      L_Motor2.driver_Enable();
      R_Motor2.driver_Disable();

      L_Motor2.set_Dir(CCW);
      L_Motor2.set_Speed(speed);
    }
    break;
  
    case ROTATE_LEFT:
    running = true;

    if(mode == LF_MODE){
      L_Motor1.motor_Run();
      R_Motor1.motor_Run();

      L_Motor1.driver_Enable();
      R_Motor1.driver_Enable();

      L_Motor1.set_Dir(CW);
      R_Motor1.set_Dir(CW);

      L_Motor1.set_Speed(speed);
      R_Motor1.set_Speed(speed);
    }

    else if(mode == LIDAR_MODE){
      L_Motor2.driver_Enable();
      R_Motor2.driver_Enable();

      L_Motor2.set_Dir(CW);
      R_Motor2.set_Dir(CW);

      L_Motor2.set_Speed(speed);
      R_Motor2.set_Speed(speed);
    }
    break;
  
    case ROTATE_RIGHT:
    running = true;

    if(mode == LF_MODE){
      L_Motor1.motor_Run();
      R_Motor1.motor_Run();

      L_Motor1.driver_Enable();
      R_Motor1.driver_Enable();

      L_Motor1.set_Dir(CCW);
      R_Motor1.set_Dir(CCW);

      L_Motor1.set_Speed(speed);
      R_Motor1.set_Speed(speed);
    }

    else if(mode == LIDAR_MODE){
      L_Motor2.driver_Enable();
      R_Motor2.driver_Enable();

      L_Motor2.set_Dir(CCW);
      R_Motor2.set_Dir(CCW);

      L_Motor2.set_Speed(speed);
      R_Motor2.set_Speed(speed);
    }
    break;

    case BRAKE:
    switch(brake){
      case NORMAL_BRAKE:
      if(mode == LF_MODE){
        L_Motor1.motor_Run();
        R_Motor1.motor_Run();

        L_Motor1.driver_Disable();
        R_Motor1.driver_Disable();
      }

      else if(mode == LIDAR_MODE){
        L_Motor2.driver_Disable();
        R_Motor2.driver_Disable();
      }
      running = false;
      break;

      case REGENERATIVE_BRAKE:
      if(running){
        for(int i=speed; i>= 0; i-=brake_val){
          if(mode == LF_MODE){
            L_Motor1.set_Speed(i);
            R_Motor1.set_Speed(i);
          }

          else if(mode == LIDAR_MODE){
            L_Motor2.set_Speed(i);
            R_Motor2.set_Speed(i);
          }
          Serial.println(i);
          delay(regenerative_interval);
        }

        if(mode == LF_MODE){
          L_Motor1.motor_Stop();
          R_Motor1.motor_Stop();

          L_Motor1.driver_Enable();
          R_Motor1.driver_Enable();
        }

        else if(mode == LIDAR_MODE){
          L_Motor2.driver_Disable();
          R_Motor2.driver_Disable();
        }
        running = false;
      }

      else if(!running){
        
      }
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
        Calc_LF_PID(FRONT);
        if(millis() - prev_tickC > nfc_read_interval){
          NFC_Handler(FRONT_NFC);
          prev_tickC = millis();
        }
      }
      else{
        Calc_LF_PID(REAR);
        if(millis() - prev_tickC > nfc_read_interval){
          NFC_Handler(REAR_NFC);
          prev_tickC = millis();
        }
      }
      Line_Check(1);
      Motor_Handler(parameter.Running_Mode, parameter.Running_Dir, parameter.Running_Accel, parameter.Running_Brake, parameter.Base_Speed);
    }

    else{
      Motor_Handler(parameter.Running_Mode, parameter.Running_Dir, parameter.Running_Accel, parameter.Running_Brake, parameter.Base_Speed);
    }
    break;

    case LIDAR_MODE:
    if(parameter.Start_coordinateY > 0 || parameter.Goal_coordinateY > 0){
      parameter.Left_enc_counter = odometry.wl_counter;
      parameter.Right_enc_counter = odometry.wr_counter;
      GoTo(parameter.Start_coordinateY, parameter.Goal_coordinateY, 150);
    }
    break;
  }
}

// Destination To Current Position Check FUnction
void Destination_Handler(void){
  if(prev_goalX != parameter.Goal_coordinateX || prev_goalY != parameter.Goal_coordinateY){
    prev_goalX = parameter.Goal_coordinateX;
    prev_goalY = parameter.Goal_coordinateY;

    path_planned = false;
  } 

  if(turning_decision == TURN_STOP){
    if(parameter.Tag_sign == HOME_SIGN){
      parameter.Current_Pos = HOME;
      Motor_Handler(parameter.Running_Mode, BRAKE, NORMAL_ACCEL, REGENERATIVE_BRAKE, parameter.Base_Speed);
      delay(1000);
      Line_Search(FRONT);
      parameter.Running_State = PAUSE;
    }

    else if(parameter.Tag_sign == CARRIER_SIGN){
      parameter.Current_Pos = ON_STATION;
      parameter.CurrentPos_Value = parameter.Tag_value;
      delay(1000);
      Motor_Handler(parameter.Running_Mode, BRAKE, NORMAL_ACCEL, REGENERATIVE_BRAKE, parameter.Base_Speed);
      delay(1000);
      Line_Search(FRONT);
      parameter.Running_State = PAUSE;
    }

    else if(parameter.Tag_sign == STATION_SIGN){
      parameter.Current_Pos = ON_STATION;
      parameter.CurrentPos_Value = parameter.Tag_value;
      Motor_Handler(parameter.Running_Mode, BRAKE, NORMAL_ACCEL, REGENERATIVE_BRAKE, parameter.Base_Speed);
      digitalWrite(SIG_LR, HIGH);

      while(1){
        if(Station_Handler() == true) break;
        else continue;
      }

      turning_decision = NONE_TURN;
      parameter.Running_State = PAUSE;
    }
  }

  else if(turning_decision != TURN_STOP && path_planned == false){
    Run_Path_Planning();
  }
}

// Custom Destination Handler Without Path Planning
void Custom_Destination_Handler(void){
  if(parameter.Current_coordinateX != parameter.Goal_coordinateX || parameter.Current_coordinateX != parameter.Goal_coordinateY){
    parameter.Current_Pos = ON_STATION;
    parameter.CurrentPos_Value = parameter.Tag_value;
    Motor_Handler(parameter.Running_Mode, BRAKE, NORMAL_ACCEL, REGENERATIVE_BRAKE, parameter.Base_Speed);
    digitalWrite(SIG_LR, HIGH);

    while(1){
      if(Station_Handler() == true) break;
      else continue;
    }

    turning_decision = NONE_TURN;
    parameter.Running_State = PAUSE;
  } 
}


// Station Package Sync Algorithm
bool Station_Handler(void){
  rear_state = Read_Proximity(REAR);

  if(rear_state != 1) return false;
  else return true;
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
void Run_Path_Planning(void){
  Point start_pos(parameter.Start_coordinateX, parameter.Start_coordinateY);
  Point goal_pos(parameter.Goal_coordinateX, parameter.Goal_coordinateY);

  if(!Run_AStar(start_pos, goal_pos)){
    path_planned = false;
  }

  else{
    Reconstruct_Path(start_pos, goal_pos);
    Reconstruct_PathDir();
    path_planned = true;
  }
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
    for(int i=0; i<stack_size; i++){
      if(NFC_R.tagPosX == invert_traceBack[i].x && NFC_R.tagPosY == invert_traceBack[i].y && nodes_checked[i].x != invert_traceBack[i].x && nodes_checked[i].y != invert_traceBack[i].y){
        nodes_checked[i] = invert_traceBack[i];

        if(NFC_R.tagType == NONE_SIGN){
          if(path_step[i] == 'f')
            turning_decision = NONE_TURN;

          else if(path_step[i] == 'r')
            turning_decision = TURN_RIGHT;

          else if(path_step[i] == 'l')
            turning_decision = TURN_LEFT;

          else if(path_step[i] == 's')
            turning_decision = TURN_STOP;
        }
        
        else if(NFC_R.tagType == CROSSECTION_SIGN){
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
}

// NFC Handler Function
void NFC_Handler(NFC_Select_t select){
  if(select == FRONT_NFC){
    digitalWrite(VIR_VCC1, HIGH);
    digitalWrite(VIR_VCC2, LOW);

    NFC_readTag(&NFC_F);
    // Nodes_Check(FRONT_NFC);
    Turn_Check(FRONT_NFC);
    // Cross_Check(FRONT_NFC);
    // Destination_Handler();
    Custom_Destination_Handler();

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
    // Nodes_Check(REAR_NFC);
    Turn_Check(REAR_NFC);
    // Cross_Check(REAR_NFC);
    // Destination_Handler();
    Custom_Destination_Handler();

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

// CHANGE HEADING POSITION
void Change_Heading(double start_heading, double end_heading, double tollerance){
  full_rotation_dist = 2 * M_PI * (odometry.wL/2);

  if(end_heading != prev_heading){
    current_heading = start_heading + update_heading;
    Serial.println(current_heading);

    odometry.enc_Read();
    odometry.est_Speed();

    current_distance = (odometry.Left_dist + odometry.Right_dist) / 2;
    update_heading = (current_distance / full_rotation_dist) * 360;

    if(start_heading > end_heading)
      Motor_Handler(LIDAR_MODE, ROTATE_RIGHT, NORMAL_ACCEL, NORMAL_BRAKE, 50);

    else if(end_heading > start_heading)
      Motor_Handler(LIDAR_MODE, ROTATE_LEFT, NORMAL_ACCEL, NORMAL_BRAKE, 50);

    if(current_heading >= end_heading - tollerance && current_heading <= end_heading + tollerance){
      digitalWrite(PILOTLAMP_PIN, HIGH);
      Motor_Handler(LIDAR_MODE, BRAKE, NORMAL_ACCEL, NORMAL_BRAKE, 50);
      prev_heading = end_heading;
      head_aligned =  true;
    }
    else head_aligned = false;
  }
}

// Go To Destination Point From Start Point
void GoTo(double start_position, double end_position, double tollerance){
  if(end_position != prev_position){
    Calc_Balancer_PID(ENC_MODE);
    odometry.enc_Read();
    odometry.est_Speed();
    current_position = start_position + ((odometry.Right_dist + odometry.Left_dist)/2);
    Motor_Handler(LIDAR_MODE, FORWARD, NORMAL_ACCEL, REGENERATIVE_BRAKE, 50);
    
    if(start_position < end_position){
      Motor_Handler(LIDAR_MODE, FORWARD, REGENERATIVE_ACCEL, REGENERATIVE_BRAKE, 50);
    }

    else if(start_position > end_position){
      Motor_Handler(LIDAR_MODE, BACKWARD, REGENERATIVE_ACCEL, REGENERATIVE_BRAKE, 50);
    }

    if(current_position >= end_position - tollerance && current_position <= end_position + tollerance){
      digitalWrite(PILOTLAMP_PIN, HIGH);
      Motor_Handler(LIDAR_MODE, BRAKE, REGENERATIVE_ACCEL, REGENERATIVE_BRAKE, 50);
      prev_position = end_position;
      end_pos =  true;
    }
  }

  else end_pos = false;
}