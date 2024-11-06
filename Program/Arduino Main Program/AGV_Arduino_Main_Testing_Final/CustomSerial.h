/*  
  SERIAL COMMUNICATION LIBRARY V2
  PT. Stechoq Robotika Indonesia
  
  Date    : 25 OKTOBER 2024
  Author  : Alfonsus Giovanni Mahendra Putra - Universitas Diponegoro
*/

#ifndef CUSTOMSERIAL_H
#define CUSTOMSERIAL_H

#include "Arduino.h"
#include "string.h"

#define HEADER 0xFF
#define TAIL 0xA5

typedef enum{
  NOT_SET,
  LF_MODE,
  LIDAR_MODE
}Select_mode_data_t;

typedef enum{
  NORMAL_SPEED = 0x01,
  HIGH_SPEED
}Select_speed_data_t;

typedef enum{
  START = 0x01,
  STOP,
  PAUSE
}Select_state_data_t;

typedef enum{
  FORWARD = 0x01,
  BACKWARD,
  LEFT,
  RIGHT,
  ROTATE_LEFT,
  ROTATE_RIGHT,
  BRAKE,
}Select_dir_data_t;

typedef enum{
  DETECTED = 0x01,
  NOT_DETECTED
}Sensor_data_t;

typedef enum{
  NORMAL_ACCEL = 0x01,
  REGENERATIVE_ACCEL
}Accel_data_t;

typedef enum{
  NORMAL_BRAKE = 0x01,
  REGENERATIVE_BRAKE
}Brake_data_t;

typedef enum{
  HOME = 0x01,
  ON_STATION,
  ON_THE_WAY
}Position_data_t;

typedef enum{
  NONE_SIGN,
  HOME_SIGN,
  CARRIER_SIGN,
  STATION_SIGN,
  TURN_SIGN,
  CROSSECTION_SIGN,
}NFC_t;

typedef struct{
  uint8_t
  Running_Mode,
  Base_Speed,
  Running_State,
  Running_Dir,
  Running_Accel,
  Running_Brake,
  Current_Pos,
  SensorA_Status,
  SensorB_Status,
  Tag_sign,
  Start_coordinateX,
  Start_coordinateY,
  Goal_coordinateX,
  Goal_coordinateY,
  Current_coordinateX,
  Current_coordinateY;

  uint16_t
  Tag_value,
  Tag_num,
  CurrentPos_Value,
  Send_counter,
  Pickup_counter;

  int16_t
  Xpos,
  Ypos;

  float Battery_level;

  /*
  String Type[22] = {
    // RECIEVE STRING TPYE
    "Running Mode",
    "Base Speed",
    "Running State",
    "Running Dir",
    "Running Accel",
    "Running Brake",
    "Start Pos",
    "Destination",
    "Joystick X",
    "Joystick Y",
    
    // TRANSMIT STRING TYPE
    "Running Mode",
    "Running State",
    "Current Pos",
    "CurrentPos Val",
    "Tag Sign",
    "Tag Value",
    "Tag Num",
    "SensA State",
    "SensB State",
    "Send Counter",
    "Pickup Counter",
    "Battery Level"
  };

  String dataString_Receive;
  String dataString_Send;
  String Type_Receive[10];
  String Data_Receive[10];
  String Type_Send[12];
  String Data_Send[12];
  */
}Param_t;

void Receive_Serial(Param_t *param);
void Transmit_Serial(Param_t *param);

// void Receive_String(Param_t *param);
// void Transmit_String(Param_t *param);

#endif