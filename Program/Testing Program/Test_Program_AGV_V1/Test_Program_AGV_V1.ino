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
#include "Adafruit_NFCShield_I2C.h"
#include "SerialCom_Slave.h"
#include "SoftwareSerial.h"
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

// MFRC PIN ---
#define IRQ   2
#define RESET 3
// ------------


// TAG DATA ---------------------
#define DATA_AUTH_HEADER1   0x5A
#define DATA_AUTH_HEADER2   0xA5
#define DATA_CMD_STATION    0xFE
#define DATA_CMD_WAYPOINT   0xFF
// ------------------------------

// SENS COUNT ------
#define SENS_NUM  16
// -----------------

// TESTING SELECT ----
//#define LINESENS_TEST
//#define MOTOR_TEST
#define PID_TEST
//#define SERIAL_TEST
// -------------------

// ALGORITHM SEL --------------
//#define USE_PLXDAQ
#define USE_EARLY_CHECK
//#define USE_OLD_SENS_COORDINATE
#define USE_NEW_SENS_COORDINATE
// ----------------------------

//************************************************************************************************************************************************************************************


/*USER PRIVATE TYPEDEF*/
//************************************************************************************************************************************************************************************

// COM TYPEDEF --

// --------------

// PID TYPEDEF ---------
PIDController pid_agv_f;  // -> PID Jalan Maju
PIDController pid_agv_b;  // -> PID Jalan Mundur
// ---------------------

// NFC TYPEDEF ------------------------
Adafruit_NFCShield_I2C nfc(IRQ, RESET);

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
  READ = 0x01,
  WRITE,
  ERASE,
}NFC_Data_Action_t;

typedef struct{
  uint8_t type;
  uint16_t value;
}Tag_Data_t;

Tag_Data_t NFC_Tag;
// ------------------------------------


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
  REGENERATIVE_FORWARD,
  BACKWARD,
  REGENERATIVE_BACKWARD,
  ROTATE_LEFT,
  ROTATE_RIGHT,
  NORMAL_BREAK,
  REGENERATIVE_BREAK,
}Run_Dir_t;
// --------------

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
adaptive_turn_spd,
interval_check = 150,
line_false_cnt,
speed_cnt,
break_cnt,
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
prev_tickC; // -> Timer Regenerative

bool
no_line = false,
line_detected = false,
line_turn = false,
start = false,
running = false;
//************************************************************************************************************************************************************************************

/*USER PRIVATE FUNCTION*/
//************************************************************************************************************************************************************************************
void motor_set(Motor_sel_t motor_sel, Run_Dir_t direction, uint8_t speed, uint8_t interval);
void check_sensor(LineSens_sel_t sensor_sel);

void read_sens(LineSens_sel_t sensor_sel);
void calculate_pid(LineSens_sel_t sensor_sel);
void agv_run(Run_Dir_t direction, uint8_t speed, uint8_t interval);
bool line_search(LineSens_sel_t sensor_sel, Run_Dir_t direction);

NFC_Status_t NFC_readData(uint8_t *stored_Data);
NFC_Status_t NFC_writeData(uint8_t *Data_to_store);
NFC_Status_t NFC_readTag(Tag_Data_t *data);
NFC_Status_t NFC_writeTag(Tag_Data_t *data, uint8_t tag_type, uint16_t tag_value);
//************************************************************************************************************************************************************************************


/*VOID SETUP*/
//************************************************************************************************************************************************************************************
void setup(){
  // BAUD RATE SETUP --------
  Serial.begin(115200);
  // ------------------------

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

  digitalWrite(RUN_LR, HIGH);
  delay(20);
  digitalWrite(ENA_R, LOW);
  digitalWrite(ENA_L, LOW);
  delay(20);
  analogWrite(PWM_R, 0);
  analogWrite(PWM_L, 0);
  delay(20);
  // ---------------------

  // PID SETUP -------------------------
  /*
  pid_agv_f.Kp        = 1.4;     
  pid_agv_f.Ki        = 0.000025;       
  pid_agv_f.Kd        = 7.25; 		
  pid_agv_f.tau       = 0.01;
	pid_agv_f.limMax    = 100;     
  pid_agv_f.limMin    = -100;     
  pid_agv_f.limMaxInt = 5.0; 	   
  pid_agv_f.limMinInt = -5.0;
	pid_agv_f.T_sample  = SAMPLE_TIME_S;
  PIDController_Init(&pid_agv_f);
  */
  
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

  // SETUP NFC ---
  nfc.begin();
  nfc.SAMConfig();
  // -------------

  // EARLY CHECK SETUP --------------------
  #ifdef USE_EARLY_CHECK
  while(1){
    start = !digitalRead(START_BTN);
    if(start){
      while(1){
        if(line_search(FRONT, ROTATE_LEFT) == true) break;
        else continue;
      }
      break;
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
  // --------------------------------------
}
//************************************************************************************************************************************************************************************


/*VOID LOOP*/
//************************************************************************************************************************************************************************************
void loop(){
  #ifdef SERIAL_TEST
  
  #endif

  #ifdef LINESENS_TEST
  digitalWrite(RUN_LR, HIGH);
  digitalWrite(ENA_R, LOW);
  digitalWrite(ENA_L, LOW);
  check_sensor(FRONT);

  delay(500);
  #endif

  #ifdef MOTOR_TEST
  digitalWrite(SPEAKER_PIN, HIGH);
  motor_set(BOTH_MOTOR, REGENERATIVE_FORWARD, 150, 15);

  if(running == true){
    digitalWrite(SPEAKER_PIN, LOW);
    delay(2000);
    motor_set(BOTH_MOTOR, REGENERATIVE_BREAK, 150, 15);
    delay(1000);
  }
  #endif 

  #ifdef PID_TEST
  digitalWrite(SPEAKER_PIN, HIGH);
  agv_run(REGENERATIVE_FORWARD, 80, 5);
  
  if(millis()-prev_tickA > interval_check && !line_detected){
    line_false_cnt++;
    prev_tickA = millis();
  }
  else if (line_detected) line_false_cnt = 0;

  if(line_false_cnt >= 5) no_line = true;

  if(no_line){
    motor_set(BOTH_MOTOR, REGENERATIVE_BREAK, 80, 25);
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

  #ifdef USE_PLXDAQ
  if(millis()-prev_tickB > 10){
    Serial.print("DATA,");
    Serial.print(prev_tickB);
    Serial.print(" ,");
    Serial.println(line_pos);

    prev_tickB = millis();
  }

  #else
  //Serial.print("Line Position: ");
  Serial.println(line_pos);
  //Serial.print("  ");
  //Serial.print("PID: ");
  //Serial.println(pid_val);
  #endif
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

  if     (sens_data == 0b1000000000000000) line_pos = -22;
  else if(sens_data == 0b1100000000000000) line_pos = -20;
  else if(sens_data == 0b1110000000000000) line_pos = -18;
  else if(sens_data == 0b1111000000000000) line_pos = -16;
  else if(sens_data == 0b1111100000000000) line_pos = -14;
  else if(sens_data == 0b0111100000000000) line_pos = -12;
  else if(sens_data == 0b0111110000000000) line_pos = -10;
  else if(sens_data == 0b0011110000000000) line_pos = -8;
  else if(sens_data == 0b0011111000000000) line_pos = -6;
  else if(sens_data == 0b0001111000000000) line_pos = -5;
  else if(sens_data == 0b0001111100000000) line_pos = -4;
  else if(sens_data == 0b0000111100000000) line_pos = -3;
  else if(sens_data == 0b0000111110000000) line_pos = -2;
  else if(sens_data == 0b0000011110000000) line_pos = -1;
  else if(sens_data == 0b0000011111000000) line_pos = -1;
  else if(sens_data == 0b0000001111000000) line_pos = 0;
  else if(sens_data == 0b0000001111100000) line_pos = 1;
  else if(sens_data == 0b0000000111100000) line_pos = 1;
  else if(sens_data == 0b0000000111110000) line_pos = 2;
  else if(sens_data == 0b0000000011110000) line_pos = 3;
  else if(sens_data == 0b0000000011111000) line_pos = 4;
  else if(sens_data == 0b0000000001111000) line_pos = 5;
  else if(sens_data == 0b0000000001111100) line_pos = 6;
  else if(sens_data == 0b0000000000111100) line_pos = 8;
  else if(sens_data == 0b0000000000111110) line_pos = 10;
  else if(sens_data == 0b0000000000011110) line_pos = 12;
  else if(sens_data == 0b0000000000011111) line_pos = 14;
  else if(sens_data == 0b0000000000001111) line_pos = 16;
  else if(sens_data == 0b0000000000000111) line_pos = 18;
  else if(sens_data == 0b0000000000000011) line_pos = 20;
  else if(sens_data == 0b0000000000000001) line_pos = 22;
  #endif

  #ifdef USE_NEW_SENS_COORDINATE
  total_weight = 0;
  sens_count = 0;

  if(sensor_sel == FRONT){
    for(int i=0; i<16; i++) {
      int weight = 8 - i;
      sens_value = !digitalRead(sens_pin[FRONT][i]);
      bitWrite(sens_data, i, sens_value);

      total_weight += (bitRead(sens_data, i) & 0xFF) * sens_weight[i];
      sens_count += (bitRead(sens_data, i) & 0xFF);
    }
    line_pos = 40 - (total_weight/sens_count);
  }
  #endif

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


/*--- AGV RUNNING FUNCTION ---*/
//************************************************************************************************************************************************************************************
void agv_run(Run_Dir_t direction, uint8_t speed, uint8_t interval){
  uint8_t 
  left_tolerance = 0,
  right_tolerance = 0;

  digitalWrite(RUN_LR, LOW);
  digitalWrite(ENA_R, LOW);
  digitalWrite(ENA_L, LOW);

  if(line_turn == true) speed = speed - 15;
  else speed = speed;

  if(direction == FORWARD){
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

  else if(direction == REGENERATIVE_FORWARD){
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

  else if(direction == BACKWARD){
    // MASIH KOSONG
  }

  else if(direction == REGENERATIVE_BACKWARD){
    // MASIH KOSONG
  }
}
//************************************************************************************************************************************************************************************


//************************************************************************************************************************************************************************************
bool line_search(LineSens_sel_t sensor_sel, Run_Dir_t direction){
  while(1){
    read_sens(sensor_sel);
    motor_set(BOTH_MOTOR, direction, 30, 0);
    if(line_pos >= -20 && line_pos <= 20 && line_detected){
      motor_set(BOTH_MOTOR, NORMAL_BREAK, 0, 0);
      break;
    }
  }
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
void motor_set(Motor_sel_t motor_sel, Run_Dir_t direction, uint8_t speed, uint8_t interval){
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
      running = true;
      digitalWrite(DIR_R, LOW);
      digitalWrite(DIR_L, HIGH);
      analogWrite(PWM_R, speed);
      analogWrite(PWM_L, speed);
    }

    else if(direction == REGENERATIVE_FORWARD){
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

    else if(direction == BACKWARD){
      running = true;
      digitalWrite(DIR_R, HIGH);
      digitalWrite(DIR_L, LOW);
      analogWrite(PWM_R, speed);
      analogWrite(PWM_L, speed);
    }

    else if(direction == REGENERATIVE_BACKWARD){
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
    
    else if(direction == NORMAL_BREAK){
      digitalWrite(RUN_LR, HIGH);
    }

    else if(direction == REGENERATIVE_BREAK){
      for(int i=speed; i>= 0; i-=REG_VAL){
        analogWrite(PWM_R, i);
        analogWrite(PWM_L, i);
        delay(interval);
      }
      running = false;
    }
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

  read_success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, get_uid, &uid_Length);

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

  read_success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, get_uid, &uid_Length);

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
      data->type = tag_data[2];
      data->value = (tag_data[3] >> 8) | tag_data[4];
      
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