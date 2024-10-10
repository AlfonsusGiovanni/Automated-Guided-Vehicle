/*  
  SERIAL COMMUNICATION LIBRARY
  PT. Stechoq Robotika Indonesia
  
  Date    : 27 SEPTEMBER 2024
  Author  : Alfonsus Giovanni Mahendra Putra - Universitas Diponegoro
*/

#ifndef SERIALCOM_MASTER_H
#define SERIALCOM_MASTER_H

#include "Arduino.h"
#include "string.h"

#define HEADER 0xFF
#define ERROR_PACKET 0x54
#define MAX_LEN 16

typedef struct{
  uint8_t
  Ping = 0x01,
  Read = 0x02,
  Write = 0x03;
}Instruction_t;

typedef struct{
  uint8_t
  Running_Mode = 0x01,
  Running_State = 0x02,
  AGV_Status = 0x03,
  Sensor_Data = 0x04,
  NFC_Data = 0x05,
  Joystick_Data = 0x06;
}Item_t;

typedef struct{
  uint8_t
  Sub_item1 = 0x01,
  Sub_item2 = 0x02,
  Sub_item3 = 0x03;
}Sub_Item_t;

typedef struct{
  uint8_t
  No_err = 0x00,
  Instruction_err = 0x01,
  Item_err = 0x02,
  Length_err = 0x04,
  Tag_reading_err = 0x08,
  LoRa_com_err = 0x010,
  Carrier_hook_err = 0x20,
  Battery_overvoltage = 0x40,
  Battery_overcurrent = 0x80;
}Error_t;

typedef enum{
  LF_mode = 0x01,
  Lidar_mode
}Select_mode_data_t;

typedef enum{
  Start = 0x01,
  Stop,
  Pause
}Select_state_data_t;

typedef enum{
  Detected = 0x01,
  Not_detected
}Sensor_data_t;

typedef enum{
  Normal_Accel = 0x01,
  Regenerative_Accel
}Accel_data_t;

typedef enum{
  Normal_Brake = 0x01,
  Regenerative_Brake
}Brake_data_t;

typedef enum{
  Home = 0x01,
  On_station,
  On_the_way
}Position_data_t;

typedef struct{
  uint8_t
  return_data[16],
  data_length,
  Instruction_get,
  Item_get,
  SubItem_get,
  Select_mode,
  Base_speed = 100,
  Select_state = 2,
  Set_Acceleration = 1,
  Set_Breaking = 1,
  Position = 1,
  SensorA,
  SensorB,
  Tag_position;

  uint16_t
  Pos_value,
  Send_counter,
  Pickup_counter,
  Tag_value;

  int16_t
  Xpos,
  Ypos;

  float Battery_level;

  Instruction_t instruction;
  Error_t error_state;
  Item_t item;
  Sub_Item_t sub_item;
}Param_t;

void Send_Ping(Param_t *param);

void Set_Running_Mode(Param_t *param, Select_mode_data_t mode);
void Set_Running_BaseSpeed(Param_t *param, uint8_t base_speed);
void Set_Running_State(Param_t *param, Select_state_data_t state);
void Set_Running_Accel(Param_t *param, Accel_data_t accel);
void Set_Running_Brake(Param_t *param, Brake_data_t brake);
void Set_Joystick(Param_t *param, uint8_t Xvalue, uint8_t Yvalue);

void Read_Running_Mode(Param_t *param);
void Read_Running_State(Param_t *param);
void Read_AGV_Status(Param_t *param);
void Read_Sensor_Data(Param_t *param);
void Read_NFC_Data(Param_t *param);
void Read_Joystick_Data(Param_t *param);

#endif
