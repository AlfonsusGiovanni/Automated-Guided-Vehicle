/*  
  SERIAL COMMUNICATION LIBRARY
  PT. Stechoq Robotika Indonesia
  
  Date    : 27 SEPTEMBER 2024
  Author  : Alfonsus Giovanni Mahendra Putra - Universitas Diponegoro
*/

#include "SerialCom_Slave.h"

static uint8_t
rx_buff[MAX_LEN];

uint8_t Length_calculator(uint8_t *data, size_t length){
  uint8_t len = 0;
  for(size_t i=0; i<length; i++){
    if(data[i+3] != 0) len++;
    else{
      if(data[i+3 | data[i+4] != 0]) len++;
    }
  }
  return len;
}


/*READ WRITE RETURN*/
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

// PING PACKET RETURN
void Return_Ping(Param_t *param){
  param->Error_value = 0;
  
  uint8_t 
  len = 0x02,
  return_buff[] = {HEADER, HEADER, len, param->error_state.No_err, param->instruction.Ping};

  Serial1.write(return_buff, sizeof(return_buff));
}

// RUNNING MODE PACKET RETURN
void Return_Running_Mode(Param_t *param){
  param->Error_value = 0;
  
  uint8_t 
  len = 0x03,
  return_buff[] = {HEADER, HEADER, len, param->error_state.No_err, param->Select_mode, param->Base_speed};

  Serial1.write(return_buff, sizeof(return_buff));
}

// RUNNING STATE PACKET RETURN
void Return_Running_State(Param_t *param){
  param->Error_value = 0;
  
  uint8_t 
  len = 0x04,
  return_buff[] = {HEADER, HEADER, len, param->error_state.No_err, param->Select_state, param->Set_Acceleration, param->Set_Breaking};

  Serial1.write(return_buff, sizeof(return_buff));
}

// AGV STATUS PACKET RETURN
void Return_AGV_Status(Param_t *param){
  param->Error_value = 0;
  
  uint8_t 
  len = 0x0C,
  return_buff[] = {
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
  for(int i=0; i<4; i++) return_buff[i+11] = byte_value[i]; 

  Serial1.write(return_buff, sizeof(return_buff));
}

// SENSOR DATA PACKET RETURN
void Return_Sensor_Data(Param_t *param){
  param->Error_value = 0;
  
  uint8_t 
  len = 0x03,
  return_buff[] = {HEADER, HEADER, len, param->error_state.No_err, param->SensorA, param->SensorB};

  Serial1.write(return_buff, sizeof(return_buff));
}

// NFC DATA PACKET RETURN
void Return_NFC_Data(Param_t *param){
  param->Error_value = 0;
  
  uint8_t 
  len = 0x04,
  return_buff[] = {
    HEADER, 
    HEADER, 
    len, 
    param->error_state.No_err,
    param->Tag_position, 
    (param->Tag_value >> 8) & 0xFF,
    param->Tag_value & 0xFF,
  };

  Serial1.write(return_buff, sizeof(return_buff));
}

// JOYSTICK DATA PACKET RETURN
void Return_Joystick_Data(Param_t *param){
  param->Error_value = 0;

  uint8_t
  len = 0x03,
  return_buff[] = {HEADER, HEADER, len, param->error_state.No_err, param->Xpos, param->Ypos};

  Serial1.write(return_buff, sizeof(return_buff));
}

// ERROR PACKET RETURN
void Return_Error_Packet(Param_t *param){
  uint8_t
  len = 0x01,
  return_buff[] = {HEADER, HEADER, len, param->Error_value};

  Serial1.write(return_buff, sizeof(return_buff));
}
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

// RECEIVE INSTRUCTION
void Receive_Instruction(Param_t *param){
  uint8_t
  data_len,
  len_validation;

  // CHECK SERIAL AVAILABILITY
  if(Serial1.available()){
    Serial1.readBytes(rx_buff, MAX_LEN);

    if(rx_buff[0] == HEADER && rx_buff[1] == HEADER){
      memcpy(param->get_data, rx_buff, 16);

      data_len = rx_buff[2];
      param->data_length = data_len;

      len_validation = Length_calculator(rx_buff, data_len);
      param->length_validation = len_validation;

      // DATA LENGTH VALIDATION TRUE
      if(data_len == len_validation){
        // PING INSTRUCTION HANDLER
        if(rx_buff[3] == param->instruction.Ping){
          param->Instruction_get = param->instruction.Ping;
          param->Item_get = 0;
          Return_Ping(param);
        }

        // READ INSTRUCTION HANDLER ----------------------------------------------------
        else if(rx_buff[3] == param->instruction.Read){
          param->Instruction_get = param->instruction.Read;

          if(rx_buff[4] == param->item.Running_Mode){
            param->Item_get = param->item.Running_Mode;
            Return_Running_Mode(param);
          }
          else if(rx_buff[4] == param->item.Running_State){
            param->Item_get = param->item.Running_State;
            Return_Running_State(param);
          }
          else if(rx_buff[4] == param->item.AGV_Status){
            param->Item_get = param->item.AGV_Status;
            Return_AGV_Status(param);
          }
          else if(rx_buff[4] == param->item.Sensor_Data){
            param->Item_get = param->item.Sensor_Data;
            Return_Sensor_Data(param);
          }
          else if(rx_buff[4] == param->item.NFC_Data){
            param->Item_get = param->item.NFC_Data;
            Return_NFC_Data(param);
          }
          else if(rx_buff[4] == param->item.Joystick_Data){
            param->Item_get = param->item.Joystick_Data;
            Return_NFC_Data(param);
          }
        }
        //------------------------------------------------------------------------------

        // WRITE INSTRUCTION HANDLER ---------------------------------------------------------------
        else if(rx_buff[3] == param->instruction.Write){
          param->Instruction_get = param->instruction.Write;

          // DATA ITEM RUNNING MODE
          if(rx_buff[4] == param->item.Running_Mode){
            param->Item_get = param->item.Running_Mode;

            if(rx_buff[5] == param->sub_item.Sub_item1) param->Select_mode = rx_buff[5];
            else if(rx_buff[5] == param->sub_item.Sub_item2) param->Base_speed = rx_buff[5];

            param->Error_value = 0;
            Return_Error_Packet(param);
          }

          // DATA ITEM RUNNING STATE
          else if(rx_buff[4] == param->item.Running_State){
            param->Item_get = param->item.Running_State;

            if(rx_buff[5] == param->sub_item.Sub_item1) param->Select_state = rx_buff[6];
            else if(rx_buff[5] == param->sub_item.Sub_item2) param->Set_Acceleration = rx_buff[6];
            else if(rx_buff[5] == param->sub_item.Sub_item3) param->Set_Breaking = rx_buff[6];
      
            param->Error_value = 0;
            Return_Error_Packet(param);
          }

          // DATA ITEM JOYSTICK
          else if(rx_buff[4] == param->item.Joystick_Data){
            param->Item_get = param->item.Joystick_Data;

            param->Xpos = rx_buff[5];
            param->Ypos = rx_buff[6];

            param->Error_value = 0;
            Return_Error_Packet(param);
          }

          // DATA ITEM INCORRECT
          else{
            param->Error_value = param->error_state.Item_err;
            Return_Error_Packet(param);
          }
        }
        //------------------------------------------------------------------------------------------

        // DATA INSTRUCTION INCORRECT
        else{
          param->Error_value = param->error_state.Instruction_err;
          Return_Error_Packet(param);
        }
      }

      // DATA LENGTH VALIDATION FALSE
      else{
        param->Error_value = param->error_state.Length_err;
        Return_Error_Packet(param);
      }
    }
  }
}
