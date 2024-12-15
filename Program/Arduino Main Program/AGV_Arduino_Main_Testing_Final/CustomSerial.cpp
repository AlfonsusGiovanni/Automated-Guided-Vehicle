/*  
  SERIAL COMMUNICATION LIBRARY V2
  PT. Stechoq Robotika Indonesia
  
  Date    : 25 Oktober 2024
  Author  : Alfonsus Giovanni Mahendra Putra - Universitas Diponegoro
*/

#include "CustomSerial.h"

static uint8_t rx_buff[23];

void Receive_Serial(Param_t *param){
  if(Serial.available() > 0){
    Serial.readBytes(rx_buff, sizeof(rx_buff));

    if(rx_buff[0] == HEADER && rx_buff[1] == HEADER && rx_buff[22] == TAIL){
      param->Running_Mode = rx_buff[2];
      param->Base_Speed = rx_buff[3];
      param->Running_State = rx_buff[4];
      param->Running_Dir = rx_buff[5];
      param->Running_Accel = rx_buff[6];
      param->Running_Brake = rx_buff[7];

      param->Start_coordinateX = (rx_buff[8] << 8) | rx_buff[9];
      param->Start_coordinateY = (rx_buff[10] << 8) | rx_buff[11];
      param->Goal_coordinateX = (rx_buff[12] << 8) | rx_buff[13];
      param->Goal_coordinateY = (rx_buff[14] << 8) | rx_buff[15];

      param->Xpos = (rx_buff[16] << 8) | rx_buff[17];
      param->Ypos = (rx_buff[18] << 8) | rx_buff[19];

      param->Left_Speed = rx_buff[20];
      param->Right_Speed = rx_buff[21];
    }
  }
}

void Transmit_Serial(Param_t *param){
  uint8_t tx_buff[32];

  tx_buff[0] = HEADER; 
  tx_buff[1] = HEADER;
  tx_buff[2] = param->Running_Mode;
  tx_buff[3] = param->Running_State;
  tx_buff[4] = param->Current_Pos;
  tx_buff[5] = (uint8_t)((param->CurrentPos_Value >> 8) & 0xFF);
  tx_buff[6] = (uint8_t)(param->CurrentPos_Value & 0xFF);
  tx_buff[7] = param->Tag_sign;
  tx_buff[8] = (uint8_t)((param->Tag_value >> 8) & 0xFF);
  tx_buff[9] = (uint8_t)(param->Tag_value & 0xFF);
  tx_buff[10] = (uint8_t)((param->Tag_num >> 8) & 0xFF);
  tx_buff[11] = (uint8_t)(param->Tag_num & 0xFF);
  tx_buff[12] = (uint8_t)((param->Current_coordinateX >> 8) & 0xFF);
  tx_buff[13] = (uint8_t)(param->Current_coordinateX & 0xFF);
  tx_buff[14] = (uint8_t)((param->Current_coordinateY >> 8) & 0xFF);
  tx_buff[15] = (uint8_t)(param->Current_coordinateY & 0xFF);
  tx_buff[16] = (uint8_t)((param->Left_enc_counter >> 8) & 0xFF);
  tx_buff[17] = (uint8_t)(param->Left_enc_counter & 0xFF);
  tx_buff[18] = (uint8_t)((param->Right_enc_counter >> 8) & 0xFF);
  tx_buff[19] = (uint8_t)(param->Right_enc_counter & 0xFF);
  tx_buff[20] = param->SensorA_Status;
  tx_buff[21] = param->SensorB_Status;
  tx_buff[22] = (uint8_t)((param->Send_counter >> 8) & 0xFF);
  tx_buff[23] = (uint8_t)(param->Send_counter & 0xFF);
  tx_buff[24] = (uint8_t)((param->Pickup_counter >> 8) & 0xFF);
  tx_buff[25] = (uint8_t)(param->Pickup_counter & 0xFF);

  uint8_t byte_value[4];
  memcpy(byte_value, &param->Battery_level, sizeof(float));
  for(int i=0; i<4; i++) tx_buff[i+26] = byte_value[i];

  tx_buff[30] = 0x00; // Reserved
  tx_buff[31] = TAIL;
  
  Serial.write(tx_buff, sizeof(tx_buff));
}