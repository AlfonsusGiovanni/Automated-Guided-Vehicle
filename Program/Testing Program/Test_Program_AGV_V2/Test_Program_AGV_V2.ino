/*  
  Line Following AGV Testing Program
  PT. Stechoq Robotika Indonesia
  
  Date    : 18 SEPTEMBER 2024
  Author  : Alfonsus Giovanni Mahendra Putra - Universitas Diponegoro
*/

/*USER PRIVATE INCLUDE*/
//************************************************************************************************************************************************************************************
#include "PID_driver.h"
#include "Wire.h" 
#include "Adafruit_PN532.h"
#include "SerialCom_Slave.h"
#include "SoftwareSerial.h"
#include "Wire.h"
#include "LiquidCrystal_I2C.h"
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
#define REG_VAL    10
// ------------------

// MFRC PIN --------
#define IRQ_NFC1    14
#define IRQ_NFC2    16
#define VIR_VCC1    15
#define VIR_VCC2    17
#define DUMMY_RESET 2
// -------------------

// TAG DATA ---------------------
#define DATA_AUTH_HEADER1   0x5A
#define DATA_AUTH_HEADER2   0xA5
#define DATA_CMD_STATION    0xFE
#define DATA_CMD_WAYPOINT   0xFF
// ------------------------------

// SENS COUNT ------
#define SENS_NUM  16
// -----------------

// TESTING SELECT ------
//#define LINESENS_TEST
//#define MOTOR_TEST
//#define PID_TEST
//#define SERIAL_TEST
//#define USE_ADAPTIVE_SPEED
// ---------------------

// ALGORITHM SEL --------------
//#define USE_PLXDAQ
//#define USE_EARLY_CHECK
// ----------------------------

//************************************************************************************************************************************************************************************


/*USER PRIVATE TYPEDEF*/
//************************************************************************************************************************************************************************************

// COM TYPEDEF ---
Param_t parameter;
// ---------------

// LCD TYPEDEF --------------------
LiquidCrystal_I2C lcd(0x27, 16, 2);
// --------------------------------

// PID TYPEDEF ---------
PIDController pid_agv_f;  // -> PID Jalan Maju
PIDController pid_agv_b;  // -> PID Jalan Mundur
// ---------------------

// NFC TYPEDEF ---------------------------
Adafruit_PN532 nfc(IRQ_NFC1, DUMMY_RESET);

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

Tag_Data_t NFC_Tag;
// ---------------------------------------

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

//************************************************************************************************************************************************************************************


/*USER PRIVATE VARIABLE*/
//************************************************************************************************************************************************************************************
const uint8_t
sens_pin[2][SENS_NUM] = {
  {53, 51, 49, 47, 45, 43, 41, 39, 37, 35, 33, 31, 29, 27, 25, 23},
  {38, 36, 40, 34, 42, 32, 44, 30, 46, 28, 48, 26, 50, 24, 52, 22}
};

const int32_t
sens_weight[SENS_NUM] = {0, 7.5, 12.5, 17.5, 22.5, 27.5, 32.5, 37.5, 42.5, 47.5, 52.5, 57.5, 62.5, 67.5, 72.5, 80};

uint8_t
nfc_data[16],
nfc_uid[4],
NFC_Timeout = 20,
decrease_speed,
interval = 5,
line_false_interval = 150,
line_false_cnt,
turn_check_interval = 50,
turn_start,
turn_end,
turning,
turn_validation_cnt,
speed_cnt,
brake_cnt,
sens_cnt;

uint16_t 
sens_value,
sens_data;

int8_t
R_speed,
L_speed;

int16_t
R_line_pos,
L_line_pos;

int
total_weight = 0, 
sens_count = 0;

float
line_pos,
pid_val;

unsigned long
prev_tickA, // -> Timer Deteksi Garis
prev_tickB, // -> Timer Serial Print PLX DAQ
prev_tickC, // -> Timer Regenerative
prev_tickD, // -> Timer Deteksi Penanda Belokan (Deakselerasi)
prev_tickE; // -> Timer Deteksi Penanda Belokan (Akselerasi)

bool
no_line = false,
line_detected = false,
line_turn = false,
prev_line_turn = false,
speed_down = false,
start = false,
running = false;
//************************************************************************************************************************************************************************************

/*USER PRIVATE FUNCTION*/
//************************************************************************************************************************************************************************************

// MAIN FUNCTION --------------------------------------------------------------------------------------
void read_sens(LineSens_sel_t sensor_sel);
void calculate_pid(LineSens_sel_t sensor_sel);
void motor_handler(uint8_t direction, uint8_t accel, uint8_t brake, uint8_t speed);
bool line_search(LineSens_sel_t sensor_sel, uint8_t direction);
bool turn_check(void);
void com_handler(void);
void return_home(void);
void run_agv(uint8_t agv_mode);
void NFC_Handler(NFC_Select_t select);

NFC_Status_t NFC_readData(uint8_t *stored_Data);
NFC_Status_t NFC_writeData(uint8_t *Data_to_store);
NFC_Status_t NFC_readTag(Tag_Data_t *data);
NFC_Status_t NFC_writeTag(Tag_Data_t *data, uint8_t tag_type, uint16_t tag_value);
// ----------------------------------------------------------------------------------------------------

// TESTING FUNCTION ------------------------------------------------------------------------------------
void check_sensor(LineSens_sel_t sensor_sel);
void check_motor(Motor_sel_t motor_sel, uint8_t direction, uint8_t accel, uint8_t brake, uint8_t speed);
void check_agvrun(Select_dir_data_t direction, Accel_data_t accel, uint8_t speed);
// -----------------------------------------------------------------------------------------------------

//************************************************************************************************************************************************************************************


/*VOID SETUP*/
//************************************************************************************************************************************************************************************
void setup(){
  // BAUD RATE SETUP --------
  Serial.begin(115200);
  // ------------------------

  // SETUP LCD ----
  lcd.init();
  lcd.backlight();
  // --------------

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

  // MOTOR PIN SETUP --------
  pinMode(ENA_R, OUTPUT);
  pinMode(PWM_R, OUTPUT);
  pinMode(DIR_R, OUTPUT);

  pinMode(ENA_L, OUTPUT);
  pinMode(PWM_L, OUTPUT);
  pinMode(DIR_L, OUTPUT);

  pinMode(RUN_LR, OUTPUT);

  digitalWrite(RUN_LR, HIGH);
  delay(20);
  digitalWrite(ENA_R, LOW);
  digitalWrite(ENA_L, LOW);
  delay(20);
  analogWrite(PWM_R, 0);
  analogWrite(PWM_L, 0);
  delay(20);
  // ------------------------

  // COMMUNICATION SETUP -------------------------
  parameter.Select_mode = NOT_SET,
  parameter.Base_speed = 80;
  parameter.Select_state = STOP;
  parameter.Set_Acceleration = REGENERATIVE_ACCEL;
  parameter.Set_Braking = REGENERATIVE_BRAKE;
  parameter.Position = HOME;
  parameter.SensorA = NOT_DETECTED;
  parameter.SensorB = NOT_DETECTED;
  parameter.Tag_sign = HOME_SIGN;
  // ---------------------------------------------

  // PID SETUP -----------------------
  pid_agv_f.Kp        = 0.825;     
  pid_agv_f.Ki        = 0.00002;       
  pid_agv_f.Kd        = 0.020; 		
  pid_agv_f.tau       = 0.01;
	pid_agv_f.limMax    = 100;     
  pid_agv_f.limMin    = -100;     
  pid_agv_f.limMaxInt = 5.0; 	   
  pid_agv_f.limMinInt = -5.0;
	pid_agv_f.T_sample  = SAMPLE_TIME_S;
  PIDController_Init(&pid_agv_f);
  // ---------------------------------

  // SETUP NFC ----------------
  pinMode(VIR_VCC1, OUTPUT);
  pinMode(VIR_VCC2, OUTPUT);
  digitalWrite(VIR_VCC1, HIGH);
  digitalWrite(VIR_VCC2, LOW);

  nfc.begin();
  nfc.SAMConfig();
  // --------------------------

  // EARLY CHECK SETUP -----------------------------------
  #ifdef USE_EARLY_CHECK
  while(1){
    start = !digitalRead(START_BTN);
    if(start){
      while(1){
        if(line_search(FRONT, ROTATE_LEFT) == true) break;
        else continue;
      }
      digitalWrite(SPEAKER_PIN, HIGH);
      delay(500);
      digitalWrite(SPEAKER_PIN, LOW);
      delay(500);
      break;
    }
  }

  #else
  while(1){
    start = !digitalRead(START_BTN);
    if(start){
      digitalWrite(SPEAKER_PIN, HIGH);
      delay(500);
      digitalWrite(SPEAKER_PIN, LOW);
      delay(500);
      break;
    }
  }
  #endif
  // -----------------------------------------------------

  #ifdef SERIAL_TEST
  while(1){
    Receive_Instruction(&parameter);
    if(parameter.Select_state == START){
      digitalWrite(SPEAKER_PIN, HIGH);
      delay(500);
      digitalWrite(SPEAKER_PIN, LOW);
      delay(500);
      break;
    }
  }
  #endif

  parameter.Select_mode = LF_MODE;
  parameter.Select_state = START;
  parameter.Set_Direction = FORWARD;
  NFC_Tag.tagType = HOME_SIGN;
  Serial.println("START");
}
//************************************************************************************************************************************************************************************


/*VOID LOOP*/
//************************************************************************************************************************************************************************************
void loop(){
  digitalWrite(SPEAKER_PIN, HIGH);
  /*
  Serial.print(NFC_Tag.tagType);
  Serial.print(" ");
  Serial.print(NFC_Tag.tagTypeValue);
  Serial.print(" ");
  Serial.println(NFC_Tag.tagNum);
  */
}
//************************************************************************************************************************************************************************************


/*--- READ SENSOR FUNCTION ---*/
//************************************************************************************************************************************************************************************
void read_sens(LineSens_sel_t sensor_sel){
  total_weight = 0;
  sens_count = 0;

  if(sensor_sel == FRONT){
    for(int i=0; i<16; i++){
      int weight = 8 - i;
      sens_value = !digitalRead(sens_pin[FRONT][i]);
      bitWrite(sens_data, i, sens_value);

      total_weight += (bitRead(sens_data, i) & 0xFF) * sens_weight[i];
      sens_count += (bitRead(sens_data, i) & 0xFF);
    }

    line_pos = 40 - (total_weight/sens_count);
  }

  else if(sensor_sel == REAR){
    // NOT SET
  }

  if(sens_data == 0b0000000000000000){
    line_pos = 0;
    line_detected = false;
  }
  else line_detected = true;
}
//************************************************************************************************************************************************************************************


/*--- CALCULATE PID FUNCTION ---*/
//************************************************************************************************************************************************************************************
void calculate_pid(LineSens_sel_t sensor_sel){
  read_sens(sensor_sel);

  if(sensor_sel == FRONT){
    PIDController_Update(&pid_agv_f, 0, line_pos);
    pid_val = pid_agv_f.out;
  }

  else{
    PIDController_Update(&pid_agv_b, 0, line_pos);
    pid_val = pid_agv_b.out;
  }
}
//************************************************************************************************************************************************************************************


/*--- MOTOR HANDLER FUNCTION ---*/
//************************************************************************************************************************************************************************************
void motor_handler(uint8_t direction, uint8_t accel, uint8_t brake, uint8_t speed){
  uint8_t 
  left_tolerance = 0,
  right_tolerance = 0;

  digitalWrite(RUN_LR, LOW);
  digitalWrite(ENA_R, LOW);
  digitalWrite(ENA_L, LOW);

  if(direction == FORWARD){
    if(accel == NORMAL_ACCEL){
      calculate_pid(FRONT);

      R_speed = speed + pid_val - decrease_speed;
      L_speed = speed - pid_val - decrease_speed;

      digitalWrite(DIR_R, LOW);
      digitalWrite(DIR_L, HIGH);

      constrain(R_speed, 0, speed);
      constrain(L_speed, 0, speed);
      analogWrite(PWM_R, R_speed);
      analogWrite(PWM_L, L_speed);
    }

    else if(accel == REGENERATIVE_ACCEL){
      calculate_pid(FRONT);

      digitalWrite(DIR_R, LOW);
      digitalWrite(DIR_L, HIGH);

      if(millis()-prev_tickC > interval && speed_cnt <= speed && !running){
        speed_cnt++;
        analogWrite(PWM_R, speed_cnt);
        analogWrite(PWM_L, speed_cnt);
        prev_tickC = millis();
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
    }
  }

  else if(direction == BACKWARD){
    if(accel == NORMAL_ACCEL){
      calculate_pid(REAR);
      running = true;

      R_speed = speed + pid_val - decrease_speed;
      L_speed = speed - pid_val - decrease_speed;

      digitalWrite(DIR_R, HIGH);
      digitalWrite(DIR_L, LOW);

      constrain(R_speed, 0, speed);
      constrain(L_speed, 0, speed);
      analogWrite(PWM_R, R_speed);
      analogWrite(PWM_L, L_speed);
    }

    else if(accel == REGENERATIVE_ACCEL){
      calculate_pid(REAR);
      
      digitalWrite(DIR_R, HIGH);
      digitalWrite(DIR_L, LOW);

      if(millis()-prev_tickC > interval && speed_cnt <= speed && !running){
        speed_cnt++;
        analogWrite(PWM_R, speed_cnt);
        analogWrite(PWM_L, speed_cnt);
        prev_tickC = millis();
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
    }
  }

  else if(direction == LEFT){
    running = true;
    digitalWrite(ENA_R, HIGH);
    digitalWrite(ENA_L, LOW);

    digitalWrite(DIR_R, LOW);
    analogWrite(PWM_R, speed);
  }

  else if(direction == RIGHT){
    running = true;
    digitalWrite(ENA_R, LOW);
    digitalWrite(ENA_L, HIGH);

    digitalWrite(DIR_L, HIGH);
    analogWrite(PWM_L, speed);
  }

  else if(direction == ROTATE_LEFT){
    running = true;
    digitalWrite(DIR_R, LOW);
    digitalWrite(DIR_L, LOW);
    analogWrite(PWM_R, speed);
    analogWrite(PWM_L, speed);
  }

  else if(direction == ROTATE_RIGHT){
    running = true;
    digitalWrite(DIR_R, HIGH);
    digitalWrite(DIR_L, HIGH);
    analogWrite(PWM_R, speed);
    analogWrite(PWM_L, speed);
  }
  
  else if(direction == BRAKE){
    if(brake == NORMAL_BRAKE){
      digitalWrite(RUN_LR, HIGH);
      running = false;
    }
    else if(brake == REGENERATIVE_BRAKE){
      if(running){
        for(int i=speed; i>= 0; i-=REG_VAL){
          analogWrite(PWM_R, i);
          analogWrite(PWM_L, i);
          delay(interval);
        }
      }
      running = false;
    }
  }
}
//************************************************************************************************************************************************************************************


/*--- LINE SEARCHING ALGORITHM ---*/
//************************************************************************************************************************************************************************************
bool line_search(LineSens_sel_t sensor_sel, uint8_t direction){
  while(1){
    read_sens(sensor_sel);
    motor_handler(direction, NORMAL_ACCEL, NORMAL_BRAKE, 30);
    if(line_pos >= -35 && line_pos <= 35 && line_detected){
      motor_handler(BRAKE, NORMAL_ACCEL, NORMAL_BRAKE, 0);
      break;
    }
  }
  return true;
}
//************************************************************************************************************************************************************************************


/*--- TURN CHECK ALGORITHM ---*/
//************************************************************************************************************************************************************************************
bool turn_check(void){
  if(bitRead(sens_data, 15) == 0b1 && bitRead(sens_data, 0) == 0b1) return true;
  else if(bitRead(sens_data, 14) == 0b1 && bitRead(sens_data, 1) == 0b1) return true;
  else return false;
}
//************************************************************************************************************************************************************************************


/*--- COMMUNICATION HANDLER ---*/
//************************************************************************************************************************************************************************************
void com_handler(void){
  
}
//************************************************************************************************************************************************************************************


/*--- TURNING TO HOME FUNCTION ---*/
//************************************************************************************************************************************************************************************
void return_home(void){

}
//************************************************************************************************************************************************************************************


/*--- MAIN ROBOT ALGORITHM ---*/
//************************************************************************************************************************************************************************************
void run_agv(uint8_t agv_mode){
  if(agv_mode == LF_MODE){
    if(parameter.Select_state == START){
      if(parameter.Set_Direction == FORWARD){
        calculate_pid(FRONT);
        if(millis() - prev_tickD > 100){
          NFC_Handler(FRONT_NFC);
          prev_tickD = millis();
        }
      }
      else if(parameter.Set_Direction == BACKWARD){
        calculate_pid(REAR);
        if(millis() - prev_tickD > 100){
          NFC_Handler(FRONT_NFC);
          prev_tickD = millis();
        }
      }
      
      if(millis()-prev_tickA > line_false_interval && !line_detected){
        line_false_cnt++;
        prev_tickA = millis();
      }
      else if (line_detected) line_false_cnt = 0;

      if(line_false_cnt >= 5) no_line = true;

      if(no_line){
        motor_handler(BRAKE, NORMAL_ACCEL, REGENERATIVE_BRAKE, 50);
        delay(1500);
        while(1){
          if(line_search(FRONT, ROTATE_LEFT) == true){
            line_false_cnt = 0;
            no_line = false;
            delay(1000);
            break;
          }
          else continue;
        }
      }
      motor_handler(parameter.Set_Direction, parameter.Set_Acceleration, parameter.Set_Braking, 50);
    }
    else if(parameter.Select_state == STOP){
      motor_handler(BRAKE, NORMAL_ACCEL, REGENERATIVE_BRAKE, parameter.Base_speed);
      return_home();
    }
    else{
      if(parameter.Set_Direction == FORWARD) calculate_pid(FRONT);
      else if(parameter.Set_Direction == BACKWARD) calculate_pid(REAR);
      motor_handler(BRAKE, NORMAL_ACCEL, REGENERATIVE_BRAKE, parameter.Base_speed);
    }
  }

  else if(agv_mode == LIDAR_MODE){
    // NOT SET
  }
}
//************************************************************************************************************************************************************************************


/*--- NFC TAG READ HANDLER ---*/
//************************************************************************************************************************************************************************************
void NFC_Handler(NFC_Select_t select){
  if(select == FRONT_NFC){
    digitalWrite(VIR_VCC1, HIGH);
    digitalWrite(VIR_VCC2, LOW);

    NFC_readTag(&NFC_Tag);
    parameter.Tag_sign = NFC_Tag.tagType;
    parameter.Tag_value = NFC_Tag.tagTypeValue;
    parameter.Tag_num = NFC_Tag.tagNum;
  }

  else if(select == REAR_NFC){
    digitalWrite(VIR_VCC1, LOW);
    digitalWrite(VIR_VCC2, HIGH);

    NFC_readTag(&NFC_Tag);
    parameter.Tag_sign = NFC_Tag.tagType;
    parameter.Tag_value = NFC_Tag.tagTypeValue;
    parameter.Tag_num = NFC_Tag.tagNum;
  }
}
//************************************************************************************************************************************************************************************


/*--- READ DATA FROM NFC ---*/
//************************************************************************************************************************************************************************************
NFC_Status_t NFC_readData(uint8_t *stored_Data){
  uint8_t
  get_uid[7],
  get_data[6],
  keyA[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
  uid_Length,
  read_success;

  read_success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, get_uid, &uid_Length, NFC_Timeout);

  if(read_success && uid_Length == 4){
    read_success = nfc.mifareclassic_AuthenticateBlock(get_uid, uid_Length, 4, 0, keyA);
    
    if(read_success){
      uint8_t data[16];

      read_success = nfc.mifareclassic_ReadDataBlock(4, data);

      if(read_success){
        for(int i=0; i<sizeof(data); i++){
          stored_Data[i] = data[i];
        }
        return SUCCESS;
      }
      else return DATA_READ_ERROR;
    }
    else return BLOCK_AUTH_ERROR;
  }
  else return UID_READ_ERROR;
}
//************************************************************************************************************************************************************************************


/*--- WRITE DATA TO NFC ---*/
//************************************************************************************************************************************************************************************
NFC_Status_t NFC_writeData(uint8_t *Data_to_store){
  uint8_t
  get_uid[7],
  keyA[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
  uid_Length,
  read_success,
  write_success;

  read_success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, get_uid, &uid_Length, NFC_Timeout);

  if(read_success && uid_Length == 4){
    read_success = nfc.mifareclassic_AuthenticateBlock(get_uid, uid_Length, 4, 0, keyA);
    
    if(read_success){
      write_success = nfc.mifareclassic_WriteDataBlock(4, Data_to_store);

      if(write_success) return SUCCESS;
      else return DATA_WRITE_ERROR;
    }
    else return BLOCK_AUTH_ERROR;
  }
  else return UID_READ_ERROR;
}
//************************************************************************************************************************************************************************************


/*--- READ TAG DATA ---*/
//************************************************************************************************************************************************************************************
NFC_Status_t NFC_readTag(Tag_Data_t *data){
  uint8_t tag_data[16];

  if(NFC_readData(tag_data) == SUCCESS){
    if(tag_data[0] == DATA_AUTH_HEADER1 && tag_data[1] == DATA_AUTH_HEADER2){
      data->tagType = tag_data[2];
      data->tagTypeValue = (tag_data[3] >> 8) | tag_data[4];
      data->tagNum = (tag_data[5] >> 8) | tag_data[6];
      
      return SUCCESS;
    }
    else return UNREGISTERED_TAG;
  } 
  else return TAG_READ_ERROR;
}
//************************************************************************************************************************************************************************************


/*--- WRITE TAG DATA ---*/
//************************************************************************************************************************************************************************************
NFC_Status_t NFC_writeTag(Tag_Data_t *data, uint8_t tag_type, uint16_t tag_value){
  uint8_t tag_data[16] = {DATA_AUTH_HEADER1, DATA_AUTH_HEADER2, tag_type};

  tag_data[3] = (tag_value >> 8) & 0xFF;
  tag_data[4] = tag_value & 0xFF;

  if(NFC_writeData(tag_data) == SUCCESS) return SUCCESS;
  else return TAG_WRITE_ERROR;
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
void check_motor(Motor_sel_t motor_sel, uint8_t direction, uint8_t accel, uint8_t brake, uint8_t speed){
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
      if(accel == NORMAL_ACCEL){
        running = true;
        digitalWrite(DIR_R, LOW);
        digitalWrite(DIR_L, HIGH);
        analogWrite(PWM_R, speed);
        analogWrite(PWM_L, speed);
      }
      
      else if(accel == REGENERATIVE_ACCEL){
        digitalWrite(DIR_R, LOW);
        digitalWrite(DIR_L, HIGH);

        if(millis()-prev_tickC > interval && speed_cnt <= speed && !running){
          speed_cnt++;
          analogWrite(PWM_R, speed_cnt);
          analogWrite(PWM_L, speed_cnt);
          prev_tickC = millis();
        }
        else if(speed_cnt == speed) running = true;

        if(running){
          analogWrite(PWM_R, speed);
          analogWrite(PWM_L, speed);
          speed_cnt = 0;
        }
      }
    }

    else if(direction == BACKWARD){
      if(accel == NORMAL_ACCEL){
        running = true;
        digitalWrite(DIR_R, HIGH);
        digitalWrite(DIR_L, LOW);
        analogWrite(PWM_R, speed);
        analogWrite(PWM_L, speed);
      }

      else if(accel == REGENERATIVE_ACCEL){
        digitalWrite(DIR_R, HIGH);
        digitalWrite(DIR_L, LOW);

        if(millis()-prev_tickC > interval && speed_cnt <= speed && !running){
          speed_cnt++;
          analogWrite(PWM_R, speed_cnt);
          analogWrite(PWM_L, speed_cnt);
          prev_tickC = millis();
        }
        else if(speed_cnt == speed) running = true;

        if(running){
          analogWrite(PWM_R, speed);
          analogWrite(PWM_L, speed);
          speed_cnt = 0;
        }
      }
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
    
    else if(direction == BRAKE){
      if(brake == NORMAL_BRAKE){
        digitalWrite(RUN_LR, HIGH);
      }
      else if(brake == REGENERATIVE_BRAKE){
        for(int i=speed; i>= 0; i-=REG_VAL){
          analogWrite(PWM_R, i);
          analogWrite(PWM_L, i);
          delay(interval);
        }
        running = false;
      }
    }
  }
}
//************************************************************************************************************************************************************************************



/*--- AGV LINE RUNNING FUNCTION ---*/
//************************************************************************************************************************************************************************************
void check_agvrun(Select_dir_data_t direction, Accel_data_t accel, Brake_data_t brake, uint8_t speed){
  uint8_t 
  left_tolerance = 0,
  right_tolerance = 0;

  digitalWrite(RUN_LR, LOW);
  digitalWrite(ENA_R, LOW);
  digitalWrite(ENA_L, LOW);

  if(direction == FORWARD){
    if(accel == NORMAL_ACCEL){
      calculate_pid(FRONT);
      R_speed = speed + pid_val;
      L_speed = speed - pid_val;

      digitalWrite(DIR_R, LOW);
      digitalWrite(DIR_L, HIGH);

      constrain(R_speed, 0, speed);
      constrain(L_speed, 0, speed);
      analogWrite(PWM_R, R_speed);
      analogWrite(PWM_L, L_speed);
    }

    else if(accel == REGENERATIVE_ACCEL){
      calculate_pid(FRONT);

      digitalWrite(DIR_R, LOW);
      digitalWrite(DIR_L, HIGH);

      if(millis()-prev_tickC > interval && speed_cnt <= speed && !running){
        speed_cnt++;
        analogWrite(PWM_R, speed_cnt);
        analogWrite(PWM_L, speed_cnt);
        prev_tickC = millis();
      }
      else if(speed_cnt == speed) running = true;

      if(running){
        R_speed = speed + pid_val - right_tolerance;
        L_speed = speed - pid_val - left_tolerance;
        constrain(R_speed, 0, speed - right_tolerance);
        constrain(L_speed, 0, speed - left_tolerance);

        analogWrite(PWM_R, R_speed);
        analogWrite(PWM_L, L_speed);
        speed_cnt = 0;
      }
    }
  }

  else if(direction == BACKWARD){
    if(accel == NORMAL_ACCEL){
      // NOT SET
    }

    else if(accel == REGENERATIVE_ACCEL){
      // NOT SET
    }
  }
}
//************************************************************************************************************************************************************************************