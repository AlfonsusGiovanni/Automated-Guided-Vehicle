/*  
  SERIAL COMMUNICATION LIBRARY
  PT. Stechoq Robotika Indonesia
  
  Date    : 27 SEPTEMBER 2024
  Author  : Alfonsus Giovanni Mahendra Putra - Universitas Diponegoro
*/

#ifndef SERIAL_COM_H
#define SERIAL_COM_H

#include "Arduino.h"
#include "string.h"

#define HEADER 0xFF
#define MAX_LEN 64

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
  NFC_Data = 0x05;
}Item_t;

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
  Send_success = 0x01,
  Receive_success,
  Data_valid,
  Dara_not_valid,
  Serial_error,
}Com_status_t;

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
  Select_mode,
  Base_speed = 100,
  Select_state = 2,
  Set_Acceleration = 1,
  Set_Breaking = 1,
  Position = 1,
  SensorA,
  SensorB,
  Tag_position,
  Error_value;

  uint16_t
  Pos_value,
  Send_counter,
  Pickup_counter,
  Tag_value;

  float *Battery_level;

  Instruction_t instruction;
  Error_t error_state;
  Com_status_t com_status;
  Item_t item;
}Param_t;

uint8_t Length_calculator(uint8_t *data);

Com_status_t 
Ping_Return(Param_t *param),
Running_Mode_Return(Param_t *param),
Running_State_Return(Param_t *param),
AGV_Status_Return(Param_t *param),
Sensor_Data_Return(Param_t *param),
NFC_Data_Return(Param_t *param),
Error_Packet_Return(Param_t *param),

Send_Ping(Param_t *param),
Send_Error(Param_t *param, uint8_t error_value),
Set_Running_Mode(Param_t *param, Select_mode_data_t mode, uint8_t base_speed),
Set_Running_State(Param_t *param, Select_state_data_t state, Accel_data_t accel, Brake_data_t brake),
Read_AGV_Status(Param_t *param, Position_data_t pos, uint16_t pos_val, uint16_t send_cnt, uint16_t pickup_cnt, float *battery_lvl),
Read_Sensor_Data(Param_t *param, Sensor_data_t sens_state),
Read_NFC_Data(Param_t *param, Position_data_t pos, uint8_t pos_val);

Com_status_t Receive_Instruction(Param_t *param);

#endif
