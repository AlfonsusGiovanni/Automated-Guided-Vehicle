/*  
  SERIAL COMMUNICATION LIBRARY V2
  PT. Stechoq Robotika Indonesia
  
  Date    : 25 OKTOBER 2024
  Author  : Alfonsus Giovanni Mahendra Putra - Universitas Diponegoro
*/

#include "CustomSerial.h"

static uint8_t rx_buff[15];

void Receive_Serial(Param_t *param){
  if(Serial.available() > 0){
    Serial.readBytes(rx_buff, sizeof(rx_buff));

    if(rx_buff[0] == HEADER && rx_buff[1] == HEADER && rx_buff[14] == TAIL){
      param->Running_Mode = rx_buff[2];
      param->Base_Speed = rx_buff[3];
      param->Running_State = rx_buff[4];
      param->Running_Dir = rx_buff[5];
      param->Running_Accel = rx_buff[6];
      param->Running_Brake = rx_buff[7];

      param->Start_coordinateX = rx_buff[8],
      param->Start_coordinateY = rx_buff[9],
      param->Goal_coordinateX = rx_buff[10],
      param->Goal_coordinateY = rx_buff[11];

      param->Xpos = rx_buff[12];
      param->Ypos = rx_buff[13];
    }
  }
}

void Transmit_Serial(Param_t *param){
  uint8_t tx_buff[25];

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
  tx_buff[12] = param->Current_coordinateX;
  tx_buff[13] = param->Current_coordinateY;
  tx_buff[14] = param->SensorA_Status;
  tx_buff[15] = param->SensorB_Status;
  tx_buff[16] = uint8_t ((param->Send_counter >> 8) & 0xFF);
  tx_buff[17] = uint8_t (param->Send_counter & 0xFF);
  tx_buff[18] = uint8_t ((param->Pickup_counter >> 8) & 0xFF);
  tx_buff[19] = uint8_t (param->Pickup_counter & 0xFF);
  tx_buff[20] = 0x00;
  tx_buff[21] = 0x00;
  tx_buff[22] = 0x00;
  tx_buff[23] = 0x00;

  uint8_t byte_value[4];
  memcpy(byte_value, &param->Battery_level, sizeof(float));
  for(int i=0; i<4; i++) tx_buff[i+20] = byte_value[i]; 

  tx_buff[24] = TAIL;

  Serial.write(tx_buff, sizeof(tx_buff));
}

/*
void Receive_String(Param_t *param){
  if(Serial.available() > 0){
    param->dataString_Receive = Serial.readStringUntil("\n");

    // param->dataString_Receive = Serial.readStringUntil("\n");
    int Typeseparator1    = param->dataString_Receive.indexOf('-');
    int Typeseparator2    = param->dataString_Receive.indexOf('-', Typeseparator1+1);
    int Typeseparator3    = param->dataString_Receive.indexOf('-', Typeseparator2+1);
    int Typeseparator4    = param->dataString_Receive.indexOf('-', Typeseparator3+1);
    int Typeseparator5    = param->dataString_Receive.indexOf('-', Typeseparator4+1);
    int Typeseparator6    = param->dataString_Receive.indexOf('-', Typeseparator5+1);
    int Typeseparator7    = param->dataString_Receive.indexOf('-', Typeseparator6+1);
    int Typeseparator8    = param->dataString_Receive.indexOf('-', Typeseparator7+1);
    int Typeseparator9    = param->dataString_Receive.indexOf('-', Typeseparator8+1);
    int Typeseparator10   = param->dataString_Receive.indexOf('-', Typeseparator9+1);

    int Typeseparator[10] = {
      Typeseparator1,
      Typeseparator2,
      Typeseparator3,
      Typeseparator4,
      Typeseparator5,
      Typeseparator6,
      Typeseparator7,
      Typeseparator8,
      Typeseparator9,
      Typeseparator10
    };

    for(int i=0; i<10; i++){
      if(i < 9) param->Type_Receive[i] = param->dataString_Receive.substring(Typeseparator[i]+1, Typeseparator[i+1]);
      else param->Type_Receive[i] = param->dataString_Receive.substring(Typeseparator[i]+1);
    }

    int Dataseparator[10];
    for(int i=0; i<10; i++){
      Dataseparator[i] = param->Type_Receive[i].indexOf('/');
    };

    for(int i=0; i<10; i++){
      param->Data_Receive[i] = param->Type_Receive[i].substring(Dataseparator[i]+1);
    }

    param->Running_Mode = uint8_t (param->Data_Receive[0].toInt());
    param->Base_Speed = uint8_t (param->Data_Receive[1].toInt());
    param->Running_State = uint8_t (param->Data_Receive[2].toInt());
    param->Running_Dir = uint8_t (param->Data_Receive[3].toInt());
    param->Running_Accel = uint8_t (param->Data_Receive[4].toInt());
    param->Running_Brake = uint8_t (param->Data_Receive[5].toInt());
    
    param->Xpos = uint8_t (param->Data_Receive[8].toInt());
    param->Ypos = uint8_t (param->Data_Receive[9].toInt());

    Serial.flush();
  }
}

void Transmit_String(Param_t *param){
  String dataType[12] = {
    String(param->Running_Mode),
    String(param->Running_State),
    String(param->Current_Pos),
    String(param->CurrentPos_Value),
    String(param->Tag_sign),
    String(param->Tag_value),
    String(param->Tag_num),
    String(param->SensorA_Status),
    String(param->SensorB_Status),
    String(param->Send_counter),
    String(param->Pickup_counter),
    String(param->Battery_level)
  };

  param->dataString_Send = "";
  for(int i=0; i<12; i++){
    if (i == 0) param->dataString_Send += "-" + param->Type[i+10] + "/" + dataType[i] + "-";
    else if (i < 11) param->dataString_Send += param->Type[i+10] + "/" + dataType[i] + "-";
    else param->dataString_Send += param->Type[i+10] + "/" + dataType[i] + "\n";
  }

  Serial.print(param->dataString_Send);
}
*/