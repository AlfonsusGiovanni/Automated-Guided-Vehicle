#ifndef SERIAL_COM_H
#define SERIAL_COM_H

#include "Arduino.h"
#include "string.h"

#define HEADER    0xFF
#define TAIL      0xA5
#define BUFF_LEN  12

typedef struct{
  uint8_t 
  position;
  
  uint16_t
  send_counter,
  pickup_counter;

  float
  bat_lvl;
}Data_t;

void TX_Serial(uint8_t pos, uint16_t send_cnt, uint16_t pickup_cnt, float battery);
void RX_Serial(Data_t *get);

#endif
