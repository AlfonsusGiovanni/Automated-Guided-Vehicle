/*  
  SERIAL COMMUNICATION LIBRARY
  PT. Stechoq Robotika Indonesia
  
  Date    : 27 SEPTEMBER 2024
  Author  : Alfonsus Giovanni Mahendra Putra - Universitas Diponegoro
*/

#include "Serial_Com.h"

static uint8_t
rx_buff[MAX_LEN];

uint8_t Length_calculator(uint8_t *data){
  uint8_t data_len = 0;
  for(size_t i=3; i<sizeof(data); i++){
    if(data[i] & data[i+1] != 0x00){
      data_len++;
    }
  }
  return data_len;
}

Com_status_t Ping_Return(Param_t *param){
  param->Error_value = 0;
  
  uint8_t 
  len = 0x02,
  tx_buff[] = {HEADER, HEADER, len, param->error_state.No_err, param->instruction.Ping};

  if(Serial.write(tx_buff, sizeof(tx_buff)) != 0) return Send_success;
  else return Serial_error;
}

Com_status_t Running_Mode_Return(Param_t *param){
  param->Error_value = 0;
  
  uint8_t 
  len = 0x03,
  tx_buff[] = {HEADER, HEADER, len, param->error_state.No_err, param->Select_mode, param->Base_speed};

  if(Serial.write(tx_buff, sizeof(tx_buff)) != 0) return Send_success;
  else return Serial_error;
}

Com_status_t Running_State_Return(Param_t *param){
  param->Error_value = 0;
  
  uint8_t 
  len = 0x04,
  tx_buff[] = {HEADER, HEADER, len, param->error_state.No_err, param->Select_state, param->Set_Acceleration, param->Set_Breaking};

  if(Serial.write(tx_buff, sizeof(tx_buff)) != 0) return Send_success;
  else return Serial_error;
}
Com_status_t AGV_Status_Return(Param_t *param){
  param->Error_value = 0;
  
  uint8_t 
  len = 0x0B,
  tx_buff[] = {
    HEADER, 
    HEADER, 
    len, 
    param->error_state.No_err, 
    param->Position,
    (param->Pos_value >> 8) & 0xFF,
    param->Pos_value & 0xFF,
    (param->Send_counter >> 8) & 0xFF,
    param->Send_counter & 0xFF,
    (param->Pickup_counter >> 8) & 0xFF,
    param->Pickup_counter & 0xFF,
    0x00,
    0x00,
    0x00,
    0x00
  };

  uint8_t byte_value[4];
  memcpy(byte_value, &param->Battery_level, sizeof(float));
  for(int i=0; i<4; i++) tx_buff[i+11] = byte_value[i]; 

  if(Serial.write(tx_buff, sizeof(tx_buff)) != 0) return Send_success;
  else return Serial_error;
}
Com_status_t Sensor_Data_Return(Param_t *param){
  param->Error_value = 0;
  
  uint8_t 
  len = 0x03,
  tx_buff[] = {HEADER, HEADER, len, param->error_state.No_err, param->SensorA, param->SensorB};

  if(Serial.write(tx_buff, sizeof(tx_buff)) != 0) return Send_success;
  else return Serial_error;
}
Com_status_t NFC_Data_Return(Param_t *param){
  param->Error_value = 0;
  
  uint8_t 
  len = 0x04,
  tx_buff[] = {
    HEADER, 
    HEADER, 
    len, 
    param->error_state.No_err,
    param->Tag_position, 
    (param->Tag_value >> 8) & 0xFF,
    param->Tag_value & 0xFF,
  };

  if(Serial.write(tx_buff, sizeof(tx_buff)) != 0) return Send_success;
  else return Serial_error;
}

Com_status_t Error_Packet_Return(Param_t *param){
  uint8_t
  len = 0x01,
  tx_buff[] = {HEADER, HEADER, len, param->Error_value};

  if(Serial.write(tx_buff, sizeof(tx_buff)) != 0) return Send_success;
  else return Serial_error;
}

Com_status_t Send_Ping(Param_t *param){

}
Com_status_t Send_Error(Param_t *param, uint8_t error_value){

}
Com_status_t Set_Running_Mode(Param_t *param, Select_mode_data_t mode, uint8_t base_speed){

}
Com_status_t Set_Running_State(Param_t *param, Select_state_data_t state, Accel_data_t accel, Brake_data_t brake){

}
Com_status_t Read_AGV_Status(Param_t *param, Position_data_t pos, uint16_t pos_val, uint16_t send_cnt, uint16_t pickup_cnt, float *battery_lvl){

}
Com_status_t Read_Sensor_Data(Param_t *param, Sensor_data_t sens_state){

}
Com_status_t Read_NFC_Data(Param_t *param, Position_data_t pos, uint8_t pos_val){
  
}

Com_status_t Receive_Instruction(Param_t *param){
  uint8_t data_len;
  uint8_t len_validation;

  // CHECK SERIAL AVAILABILITY
  if(Serial.available()){
    Serial.readBytes(rx_buff, MAX_LEN);
    len_validation = Length_calculator(rx_buff);

    if(rx_buff[0] == HEADER && rx_buff[1] == HEADER){
      data_len = rx_buff[2];
      // DATA LENGTH VALIDATION TRUE
      if(data_len == len_validation){
        if(rx_buff[3] == param->instruction.Ping){
          Ping_Return(param);
        }

        else if(rx_buff[3] == param->instruction.Read){
          if(rx_buff[4] == param->item.Running_Mode) Running_Mode_Return(param);
          else if(rx_buff[4] == param->item.Running_State) Running_State_Return(param);
          else if(rx_buff[4] == param->item.AGV_Status) AGV_Status_Return(param);
          else if(rx_buff[4] == param->item.Sensor_Data) Sensor_Data_Return(param);
          else if(rx_buff[4] == param->item.NFC_Data) NFC_Data_Return(param);
        }

        else if(rx_buff[3] == param->instruction.Write){
          if(rx_buff[4] == param->item.Running_Mode){
            param->Select_mode = rx_buff[5];
            param->Base_speed = rx_buff[6];

            param->Error_value = 0;
            Error_Packet_Return(param);
          }
          else if(rx_buff[4] == param->item.Running_State){
            param->Select_state = rx_buff[5];
            param->Set_Acceleration = rx_buff[6];
            param->Set_Breaking = rx_buff[7];

            param->Error_value = 0;
            Error_Packet_Return(param);
          }
          else if(rx_buff[4] == param->item.AGV_Status){
            uint8_t byte_data[4];

            param->Position = rx_buff[5];
            param->Pos_value = (rx_buff[6] << 8) | rx_buff[7];
            param->Send_counter = (rx_buff[8] << 8) | rx_buff[9];
            param->Pickup_counter = (rx_buff[10] << 8) | rx_buff[11];

            for(int i=0; i<4; i++) byte_data[i] = rx_buff[12+i];
            memcpy(&param->Battery_level, byte_data, sizeof(float));

            param->Error_value = 0;
            Error_Packet_Return(param);
          }
          else if(rx_buff[4] == param->item.Sensor_Data){
            param->SensorA = rx_buff[5];
            param->SensorB = rx_buff[6];

            param->Error_value = 0;
            Error_Packet_Return(param);
          }
          else if(rx_buff[4] == param->item.NFC_Data){
            param->Tag_position = rx_buff[5];
            param->Tag_value = (rx_buff[6] >> 8) | rx_buff[7];

            param->Error_value = 0;
            Error_Packet_Return(param);
          }

          // DATA ITEM INCORRECT
          else{
            param->Error_value = param->Error_value | param->error_state.Item_err;
            Error_Packet_Return(param);
          }
        }

        // DATA INSTRUCTION INCORRECT
        else{
          param->Error_value = param->Error_value | param->error_state.Instruction_err;
          Error_Packet_Return(param);
        }
      }

      // DATA LENGTH VALIDATION FALSE
      else{
        param->Error_value = param->Error_value | param->error_state.Length_err;
        Error_Packet_Return(param);
      }
    }
  }
}
