/*  
  SERIAL COMMUNICATION LIBRARY
  PT. Stechoq Robotika Indonesia
  
  Date    : 27 SEPTEMBER 2024
  Author  : Alfonsus Giovanni Mahendra Putra - Universitas Diponegoro
*/

#include "Serial_Com.h"

static uint8_t rx_data[BUFF_LEN];

void Send_Mode(Def_mode_t mode){
  uint8_t tx_data[BUFF_LEN] = {HEADER, SEND_MODE, mode};
  tx_data[15] = TAIL;

  Serial.write(tx_data, 16);
}

void Send_Run(Run_state_t state){
  uint8_t tx_data[BUFF_LEN] = {HEADER, SEND_RUN, 0x00, state};
  tx_data[15] = TAIL;

  Serial.write(tx_data, 16);
}

void Send_Status(Position_t pos, uint16_t pos_value, uint16_t send_cnt, uint16_t pickup_cnt, float battery){
  uint8_t tx_data[BUFF_LEN] = {HEADER, SEND_STATUS};

  tx_data[4] = pos;
  tx_data[5] = (pos_value >> 8) & 0xFF;
  tx_data[6] = (pos_value & 0xFF);
  tx_data[7] = (send_cnt >> 8) & 0xFF;
  tx_data[8] = (send_cnt & 0xFF);
  tx_data[9] = (pickup_cnt >> 8) & 0xFF;
  tx_data[10] = (pickup_cnt & 0xFF);

  uint8_t batVoltage[4];
  memcpy(batVoltage, &battery, sizeof(float));

  tx_data[11] = batVoltage[0];
  tx_data[12] = batVoltage[1];
  tx_data[13] = batVoltage[2];
  tx_data[14] = batVoltage[3]; 
  tx_data[15] = TAIL;

  Serial.write(tx_data, sizeof(tx_data));
}

Serial_status_t Receive_Serial(Data_t *get){
  if(Serial.available()){
    Serial.readBytes(rx_data, sizeof(rx_data));

    if(rx_data[0] == HEADER && rx_data[15] == TAIL){
      if(rx_data[1] == SEND_MODE) get->Default_Mode = rx_data[2];
      else if(rx_data[1] == SEND_RUN) get->Running_state = rx_data[3];
      return Data_Received;
    }
    else return No_Data;
  }
  else return No_Data;
}
