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

#define HEADER      0xFF
#define TAIL        0xA5
#define BUFF_LEN    16

#define SEND_MODE   0x01
#define SEND_RUN    0x02
#define SEND_STATUS 0x03

typedef enum{
  No_Data = 0x01,
  Data_Received,
}Serial_status_t;

typedef enum{
  LF_MODE = 0x01,
  LID_MODE
}Def_mode_t;

typedef enum{
  RUN_START = 0x01,
  RUN_STOP,
  RUN_PAUSE
}Run_state_t;

typedef enum{
  HOME = 0x01,
  ON_STATION,
  SHIPPING,
  PICKING_UP,
}Position_t;

typedef struct{
  Def_mode_t Default_Mode;
  Run_state_t Running_state;
}Data_t;

void Send_Mode(Def_mode_t mode);
void Send_Run(Run_state_t state);
void Send_Status(Position_t pos, uint16_t pos_value, uint16_t send_cnt, uint16_t pickup_cnt, float battery);
Serial_status_t Receive_Serial(Data_t *get);

#endif
