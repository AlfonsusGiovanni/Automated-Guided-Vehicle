/*
  Automated Guided Vehicle - Arduino Main Program
  PT. Stechoq Robotika Indonesia

  Data    : 18 Oktober 2024
  Author  : Alfonsus Giovanni Mahendra Puta - Universitas Diponegoro
*/

#define USE_NEW_PINOUT
// #define USE_OLD_PINOUT

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
#define SIG_LR      30   // Left Right Motor Start Stop (SIGNAL) <-PCB PIN

#ifdef USE_NEW_PINOUT
  #define PROX_FRONT  12
  #define PROX_REAR   13
#endif

#ifdef USE_OLD_PINOUT
  #define PROX_FRONT  A9
  #define PROX_REAR   A10
#endif

#define IRQ_NFC1    14
#define IRQ_NFC2    16
#define VIR_VCC1    15
#define VIR_VCC2    17

#define SERVO_LOCK  38
#define LS1         39
#define LS2         40


#ifdef USE_OLD_PINOUT
  #define START_BTN       41
  #define SPEAKER_PIN     42
  #define PILOTLAMP_PIN   43
#endif

#ifdef USE_NEW_PINOUT
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
start_destination,
end_destination,
turning_decision;

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
front_proximity_state,
rear_proximity_state;

/*User Private Typedef*/

typedef enum{
  NONE_TURN,
  STRAIGHT,
  TURN_LEFT,
  TURN_RIGHT
}Decision_t;

typedef enum{
  HOME_STATION = 0x01,
  STATION_A,
  STATION_B
}Station_t;

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
  new_uid[12],
  tagType;

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
void Motor_Handler(uint8_t direction, uint8_t accel, uint8_t brake, uint8_t speed);
void Run_AGV(uint8_t agv_mode);
void Destination_Handler(void);

void NFC_Handler(NFC_Select_t select);
void Turn_Check(NFC_Select_t select);
void Cross_Check(NFC_Select_t select);
void Station_Check(NFC_Select_t select);

Decision_t Run_Path_Planning(uint8_t start_destination, uint8_t end_destination);

bool Read_Proximity(Sens_sel_t sensor_sel);

float Read_Voltage(void);
float Read_Current(void);
float Check_Battery_Cappacity(void);

NFC_Status_t NFC_readData(Tag_Data_t *nfc);
NFC_Status_t NFC_readTag(Tag_Data_t *nfc);

/*User Main Program*/
int main(){
  // Communication Setup
  Serial.begin(1000000);
  Serial.setTimeout(100);

  // Sensor Pin Setup
  for(int i=0; i<SENS_NUM; i++){
    pinMode(sens_pin[i], INPUT_PULLUP);
  }

  pinMode(PROX_FRONT, INPUT_PULLUP);
  pinMode(PROX_REAR, INPUT_PULLUP);

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

  /*
  parameter.Running_Mode = LF_MODE;
  parameter.Running_Dir = FORWARD;

  parameter.Start_Pos = HOME;
  parameter.Destination = STATION_B;
  */

  parameter.Battery_level = 90.5;

  while(1){
    /*
    rear_proximity_state = Read_Proximity(REAR);
    Serial.print("Sensor State: ");
    Serial.println(front_proximity_state);
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

      if(!line_detected){
        if(millis() - prev_dummytick > 150){
          if(digitalRead(PILOTLAMP_PIN) == LOW) digitalWrite(PILOTLAMP_PIN, HIGH);
          else digitalWrite(PILOTLAMP_PIN, LOW);
          prev_dummytick = millis();
        }
      }
      else if(line_detected && parameter.Running_State != START && !btn_pressed){
        if(millis() - prev_dummytick > 500){
          if(digitalRead(PILOTLAMP_PIN) == LOW) digitalWrite(PILOTLAMP_PIN, HIGH);
          else digitalWrite(PILOTLAMP_PIN, LOW);
          prev_dummytick = millis();
        }
      }
      else if(line_detected && parameter.Running_Mode != NOT_SET && parameter.Running_State == START){
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
        
        parameter.Start_Pos = HOME_STATION;
        parameter.Destination = STATION_A;

        digitalWrite(PILOTLAMP_PIN, LOW);
        delay(1000);
        parameter.Running_State = START;
        config_done = true;
      }
    }

    else if(config_done){
      switch(parameter.Running_State){
        case START:
        Transmit_Serial(&parameter);
        
        digitalWrite(PILOTLAMP_PIN, HIGH);
        Run_AGV(parameter.Running_Mode);
        break;

        case STOP:
        if(millis() > prev_dummytick > 500){
          if(digitalRead(PILOTLAMP_PIN) == LOW) digitalWrite(PILOTLAMP_PIN, HIGH);
          else digitalWrite(PILOTLAMP_PIN, LOW);
          prev_dummytick = millis();
        }
        Motor_Handler(BRAKE, NORMAL_ACCEL, REGENERATIVE_BRAKE, parameter.Base_Speed);
        break;

        case PAUSE:
        if(millis() > prev_tickComrx > 100){
          Receive_Serial(&parameter);
          prev_tickComrx = millis();
        }

        if(parameter.Running_Dir == FORWARD) Calc_PID(FRONT);
        else if(parameter.Running_Dir == BACKWARD) Calc_PID(REAR);
        Motor_Handler(BRAKE, NORMAL_ACCEL, REGENERATIVE_BRAKE, parameter.Base_Speed);
        break;
      }
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
    Motor_Handler(direction, NORMAL_ACCEL, NORMAL_BRAKE, 30);
    if(line_pos >= -20 && line_pos <= 20 && line_detected){
      Motor_Handler(BRAKE, NORMAL_ACCEL, NORMAL_BRAKE, 30);
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
    Motor_Handler(BRAKE, NORMAL_ACCEL, REGENERATIVE_BRAKE, parameter.Base_Speed);
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
  if(sensor_sel == FRONT) return !digitalRead(PROX_FRONT);
  else return !digitalRead(PROX_REAR);
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
      Motor_Handler(parameter.Running_Dir, parameter.Running_Accel, parameter.Running_Brake, parameter.Base_Speed);
    }

    else{
      Motor_Handler(parameter.Running_Dir, parameter.Running_Accel, parameter.Running_Brake, parameter.Base_Speed);
    }
    break;

    case LIDAR_MODE:
    break;
  }
}

// Destination To Current Position Check FUnction
void Destination_Handler(void){
  if(parameter.Destination == parameter.Tag_sign){
    if(parameter.Tag_sign == HOME_SIGN){
      parameter.Current_Pos = HOME;
      Motor_Handler(BRAKE, NORMAL_ACCEL, REGENERATIVE_BRAKE, parameter.Base_Speed);
      delay(1000);
      Line_Search(FRONT);
      parameter.Running_State = PAUSE;
    }

    else if(parameter.Tag_sign == CARRIER_SIGN){ 
      parameter.Current_Pos = STATION_A;
      delay(1000);
      Motor_Handler(BRAKE, NORMAL_ACCEL, REGENERATIVE_BRAKE, parameter.Base_Speed);
      delay(1000);
      Line_Search(FRONT);
      parameter.Running_State = PAUSE;
    }

    else if(parameter.Tag_sign == STATION_SIGN){
      parameter.Current_Pos = STATION_B;
      Motor_Handler(BRAKE, NORMAL_ACCEL, REGENERATIVE_BRAKE, parameter.Base_Speed);
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
      turning_decision = Run_Path_Planning(parameter.Start_Pos, parameter.Destination);
      
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
Decision_t Run_Path_Planning(uint8_t start, uint8_t end){
  if(start == HOME_STATION && end == STATION_A) return STRAIGHT;
  else if(start == HOME_STATION && end == STATION_B) return TURN_LEFT;
  else if(start == STATION_A && end == HOME_STATION) return STRAIGHT;
  else if(start == STATION_A && end == STATION_B) return TURN_RIGHT;
  else if(start == STATION_B && end == HOME_STATION) return TURN_RIGHT;
  else if(start == STATION_B && end == STATION_A) return TURN_LEFT;
}

// NFC Handler Function
void NFC_Handler(NFC_Select_t select){
  if(select == FRONT_NFC){
    digitalWrite(VIR_VCC1, HIGH);
    digitalWrite(VIR_VCC2, LOW);

    NFC_readTag(&NFC_F);
    Destination_Handler();
    Turn_Check(FRONT_NFC);
    Cross_Check(FRONT_NFC);

    if(NFC_F.tagType != NONE_SIGN){
      parameter.Tag_sign = NFC_F.tagType;
      parameter.Tag_value = NFC_F.tagTypeValue;
      parameter.Tag_num = NFC_F.tagNum;

      parameter.Current_Pos = parameter.Tag_sign;
    }
  }

  else if(select == REAR_NFC){
    digitalWrite(VIR_VCC1, LOW);
    digitalWrite(VIR_VCC2, HIGH);

    NFC_readTag(&NFC_R);
    Destination_Handler();
    Turn_Check(REAR_NFC);
    Cross_Check(REAR_NFC);

    if(NFC_F.tagType != NONE_SIGN){
      parameter.Tag_sign = NFC_R.tagType;
      parameter.Tag_value = NFC_R.tagTypeValue;
      parameter.Tag_num = NFC_R.tagNum;

      parameter.Current_Pos = parameter.Tag_sign;
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

// CHECK NFC DATA
void Check_TagData(void){
  if(NFC_readTag(&NFC_F) == SUCCESS){
    Serial.println("(READ DATA SUCCESS)");

    Serial.print("Tag ID: ");
    for(int i=0; i<4; i++){
      Serial.print("0x");
      Serial.print(NFC_F.new_uid[i], HEX);
      Serial.print(" ");
    }
    
    Serial.print(" ");

    Serial.print("Tag Data Header: ");
    for(int i=4; i<6; i++){
      Serial.print("0x");
      Serial.print(NFC_F.new_uid[i], HEX);
      Serial.print(" ");
    }

    Serial.print(" ");

    Serial.print("Tag Data: ");
    for(int i=6; i<sizeof(NFC_F.new_uid); i++){
      Serial.print("0x");
      Serial.print(NFC_F.new_uid[i], HEX);
      Serial.print(" ");
    }

    Serial.println(" ");
  }

  else{
    Serial.println("(CANT READ DATA FROM CARD)");
  }
}
