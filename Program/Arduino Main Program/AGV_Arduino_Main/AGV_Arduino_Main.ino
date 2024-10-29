/*
  Automated Guided Vehicle - Arduino Main Program
  PT. Stechoq Robotika Indonesia

  Data    : 18 Oktober 2024
  Author  : Alfonsus Giovanni Mahendra Puta - Universitas Diponegoro
*/

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
#define SIG_LR      11   // Left Right Motor Start Stop (SIGNAL) <-PCB PIN

#define PROX_FRONT  12
#define PROX_REAR   13

#define IRQ_NFC1    14
#define IRQ_NFC2    16
#define VIR_VCC1    15
#define VIR_VCC2    17

#define SERVO_LOCK  38
#define LS1         39
#define LS2         40

#define START_BTN       41
#define SPEAKER_BTN     42
#define PILOTLAMP_PIN   43

#define SENS_NUM  16

#define DATA_AUTH_HEADER1   0x5A
#define DATA_AUTH_HEADER2   0xA5
#define DATA_CMD_STATION    0xFE
#define DATA_CMD_CROSSPOINT 0xFF

/*User Private Typedef*/

/*User Private Variable*/
const uint8_t
sens_pin[SENS_NUM] = {22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37};

const int32_t
sens_weight[SENS_NUM] = {0, 7.5, 12.5, 17.5, 22.5, 27.5, 32.5, 37.5, 42.5, 47.5, 52.5, 57.5, 62.5, 67.5, 72.5, 80};

uint8_t
interval = 5,
brake_val,
decrease_speed, 
speed_cnt,
brake_cnt,
sens_cnt,
prev_sel_sens;

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
prev_tickA; // -> Timer Regenerative

bool
running,
line_turn,
no_line,
line_detected;

/*User Private Typedef*/

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
  tagType,
  timeout = 100;

  uint16_t
  tagTypeValue,
  tagNum;
}Tag_Data_t;

typedef enum{
  FRONT = 0x00,
  REAR  = 0x01
}LineSens_sel_t;

Tag_Data_t NFC_F;
Tag_Data_t NFC_R;

Param_t parameter;

PIDController pid_agv_f;  // -> PID Jalan Maju
PIDController pid_agv_b;  // -> PID Jalan Mundur

Adafruit_PN532 nfc(IRQ_NFC1, 3);

/*User Private Function Declaration*/
void Read_Sens(LineSens_sel_t sensor_sel);
void Calc_PID(LineSens_sel_t sensor_sel);
void Line_Search(LineSens_sel_t sensor_sel);
void Motor_Handler(uint8_t direction, uint8_t accel, uint8_t brake, uint8_t speed);
void Run_AGV(uint8_t agc_mode);
void Turn_Check(void);

uint8_t Run_PathPlanner(uint8_t start_pos, uint8_t end_pos);

void NFC_Handler(NFC_Select_t select);
NFC_Status_t NFC_readData(Tag_Data_t *nfc);
NFC_Status_t NFC_readTag(Tag_Data_t *nfc);

void setup(){
  // Communication Setup
  Serial.begin(115200);

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
  pinMode(SPEAKER_BTN, OUTPUT);
  pinMode(PILOTLAMP_PIN, OUTPUT);

  // Control Pin Setup
  pinMode(IRQ_NFC1, OUTPUT);
  pinMode(IRQ_NFC2, OUTPUT);
  pinMode(VIR_VCC1, OUTPUT);
  pinMode(VIR_VCC2, OUTPUT);

  pinMode(SERVO_LOCK, OUTPUT);
  pinMode(LS1, INPUT_PULLUP);
  pinMode(LS2, INPUT_PULLUP);

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
  pid_agv_f.Kp        = 0.825;     
  pid_agv_f.Ki        = 0.00002;       
  pid_agv_f.Kd        = 0.020; 		
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
  
}

/*User Private Function Initialize*/
// Read Magnetic Line Sensor Function
void Read_Sens(LineSens_sel_t sensor_sel){
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

  for(int i=0; i<SENS_NUM; i++){
    int weight = 8 - i;
    sens_value = !digitalRead(sens_pin[i]);
    bitWrite(sens_data, i, sens_value);

    total_weight += (bitRead(sens_data, i) & 0xFF) * sens_weight[i];
    sens_count += (bitRead(sens_data, i) & 0xFF);
  }

  if(sens_data == 0b0000000000000000){
    line_pos = 0;
    line_detected = false;
  }
  else line_detected = true;

  if(!line_turn) line_pos = 40 - (total_weight/sens_count);
  else line_pos = 0;
}

// PID Calculation Function
void Calc_PID(LineSens_sel_t sensor_sel){
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
void Line_Search(LineSens_sel_t sensor_sel){
  uint8_t direction;
  if(line_pos > 10) direction = ROTATE_LEFT;
  else if(line_pos <= 10 && line_pos >= -10) direction = BACKWARD;
  else if(line_pos <= -10) direction = ROTATE_RIGHT;

  while(1){
    Read_Sens(sensor_sel);
    Motor_Handler(direction, NORMAL_ACCEL, NORMAL_BRAKE, 30);
    if(line_pos >= -35 && line_pos <= 35 && line_detected){
      Motor_Handler(BRAKE, NORMAL_ACCEL, NORMAL_BRAKE, 0);
      break;
    }
  }
  return true;
}

// Motor Handler Function
void Motor_Handler(uint8_t direction, uint8_t accel, uint8_t brake, uint8_t speed){
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
      R_speed = speed + pid_val - decrease_speed;
      L_speed = speed - pid_val - decrease_speed;

      constrain(R_speed, 0, speed);
      constrain(L_speed, 0, speed);
      analogWrite(PWM_R, R_speed);
      analogWrite(PWM_L, L_speed);
      break;

      case REGENERATIVE_ACCEL:
      if(millis()-prev_tickA > interval && speed_cnt <= speed && !running){
        speed_cnt++;
        analogWrite(PWM_R, speed_cnt);
        analogWrite(PWM_L, speed_cnt);
        prev_tickA = millis();
      }
      else if(speed_cnt == speed) running = true;

      if(running){
        R_speed = speed + pid_val - right_tolerance - decrease_speed;
        L_speed = speed - pid_val - left_tolerance - decrease_speed;
        constrain(R_speed, 0, speed - right_tolerance);
        constrain(L_speed, 0, speed - left_tolerance);

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
      R_speed = speed + pid_val - decrease_speed;
      L_speed = speed - pid_val - decrease_speed;

      constrain(R_speed, 0, speed);
      constrain(L_speed, 0, speed);
      analogWrite(PWM_R, R_speed);
      analogWrite(PWM_L, L_speed);
      break;

      case REGENERATIVE_ACCEL:
      if(millis()-prev_tickA > interval && speed_cnt <= speed && !running){
        speed_cnt++;
        analogWrite(PWM_R, speed_cnt);
        analogWrite(PWM_L, speed_cnt);
        prev_tickA = millis();
      }
      else if(speed_cnt == speed) running = true;

      if(running){
        R_speed = speed + pid_val - right_tolerance - decrease_speed;
        L_speed = speed - pid_val - left_tolerance - decrease_speed;
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
        for(int i=speed; i>= 0; i-= brake_val){
          analogWrite(PWM_R, i);
          analogWrite(PWM_L, i);
          delay(interval);
        }
      }
      running = false;
      break;
    }
    break;
  }
}

// Run AGV Function
void Run_AGV(uint8_t agc_mode){

}

// Turning Check Function
void Turn_Check(void){
  
}

// Path Planner Algorithm Function
uint8_t Run_PathPlanner(uint8_t start_pos, uint8_t end_pos){
  
}

// NFC Handler Function
void NFC_Handler(NFC_Select_t select){
  if(select == FRONT_NFC){
    digitalWrite(VIR_VCC1, HIGH);
    digitalWrite(VIR_VCC2, LOW);

    NFC_readTag(&NFC_F);
    parameter.Tag_sign = NFC_F.tagType;
    parameter.Tag_value = NFC_F.tagTypeValue;
    parameter.Tag_num = NFC_F.tagNum;
  }

  else if(select == REAR_NFC){
    digitalWrite(VIR_VCC1, LOW);
    digitalWrite(VIR_VCC2, HIGH);

    NFC_readTag(&NFC_R);
    parameter.Tag_sign = NFC_R.tagType;
    parameter.Tag_value = NFC_R.tagTypeValue;
    parameter.Tag_num = NFC_R.tagNum;
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

  read_success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, get_uid, &uid_Length);

  if(read_success && uid_Length == 4){
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
  uint8_t tag_data[16];

  if(NFC_readData(nfc_data) == SUCCESS){
    if(tag_data[0] == DATA_AUTH_HEADER1 && tag_data[1] == DATA_AUTH_HEADER2){
      nfc_data->tagType = tag_data[2];
      nfc_data->tagTypeValue = (tag_data[3] >> 8) | tag_data[4];
      nfc_data->tagNum = (tag_data[5] >> 8) | tag_data[6];
      
      return SUCCESS;
    }
    else return UNREGISTERED_TAG;
  } 
  else return TAG_READ_ERROR;
}
