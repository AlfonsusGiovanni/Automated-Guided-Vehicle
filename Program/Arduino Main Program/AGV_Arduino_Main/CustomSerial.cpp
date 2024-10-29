/*  
  SERIAL COMMUNICATION LIBRARY V2
  PT. Stechoq Robotika Indonesia
  
  Date    : 25 OKTOBER 2024
  Author  : Alfonsus Giovanni Mahendra Putra - Universitas Diponegoro
*/

#include "CustomSerial.h"

static uint8_t rx_buff[13];

uint8_t Receive_Serial(Param_t *param){
  if(Serial.available() > 0){
    Serial.readBytes(rx_buff, 13);

    if(rx_buff[0] == HEADER && rx_buff[1] == HEADER && rx_buff[12] == TAIL){
      param->Running_Mode = rx_buff[2];
      param->Base_Speed = rx_buff[3];
      param->Running_State = rx_buff[4];
      param->Running_Dir = rx_buff[5];
      param->Running_Accel = rx_buff[6];
      param->Running_Brake = rx_buff[7];
      param->Start_Pos = rx_buff[8];
      param->Destination = rx_buff[9];
      param->Xpos = rx_buff[10];
      param->Ypos = rx_buff[11];
    }
  }
}

uint8_t Transmit_Serial(Param_t *param){
  uint8_t tx_buff[32];

  tx_buff[0] = HEADER;
  tx_buff[1] = HEADER;
  tx_buff[2] = param->Running_Mode;
  tx_buff[3] = param->Running_State;
  tx_buff[4] = param->Current_Pos;
  tx_buff[5] = uint8_t ((param->CurrentPos_Value >> 8) & 0xFF);
  tx_buff[6] = uint8_t (param->CurrentPos_Value & 0xFF);
  tx_buff[7] = param->Tag_sign;
  tx_buff[8] = uint8_t ((param->Tag_value >> 8) & 0xFF);
  tx_buff[9] = uint8_t (param->Tag_value & 0xFF);
  tx_buff[10] = uint8_t ((param->Tag_num >> 8) & 0xFF);
  tx_buff[11] = uint8_t (param->Tag_num & 0xFF);
  tx_buff[12] = param->SensorA_Status;
  tx_buff[13] = param->SensorB_Status;
  tx_buff[14] = uint8_t ((param->Send_counter >> 8) & 0xFF);
  tx_buff[15] = uint8_t (param->Send_counter & 0xFF);
  tx_buff[16] = uint8_t ((param->Pickup_counter >> 8) & 0xFF);
  tx_buff[17] = uint8_t (param->Pickup_counter & 0xFF);
  tx_buff[18] = 0x00;
  tx_buff[19] = 0x00;
  tx_buff[20] = 0x00;
  tx_buff[21] = 0x00;

  uint8_t byte_value[4];
  memcpy(byte_value, &param->Battery_level, sizeof(float));
  for(int i=0; i<4; i++) tx_buff[i+18] = byte_value[i]; 

  tx_buff[22] = TAIL;

  Serial.write(tx_buff, sizeof(tx_buff));
}