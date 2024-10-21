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
  return_buff[] = {HEADER, HEADER, len, param->error_type.No_err, param->instruction_type.Ping};

  Serial.write(return_buff, sizeof(return_buff));
}

// RUNNING MODE PACKET RETURN
void Return_Running_Mode(Param_t *param){
  param->Error_value = 0;
  
  uint8_t 
  len = 0x03,
  return_buff[] = {HEADER, HEADER, len, param->error_type.No_err, param->Select_mode, param->Base_speed};

  Serial.write(return_buff, sizeof(return_buff));
}

// RUNNING STATE PACKET RETURN
void Return_Running_State(Param_t *param){
  param->Error_value = 0;
  
  uint8_t 
  len = 0x05,
  return_buff[] = {HEADER, HEADER, len, param->error_type.No_err, param->Select_state, param->Set_Direction, param->Set_Acceleration, param->Set_Braking};

  Serial.write(return_buff, sizeof(return_buff));
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
    param->error_type.No_err, 
    param->Position,
    uint8_t ((param->Pos_value >> 8) & 0xFF),
    uint8_t (param->Pos_value & 0xFF),
    uint8_t ((param->Send_counter >> 8) & 0xFF),
    uint8_t (param->Send_counter & 0xFF),
    uint8_t ((param->Pickup_counter >> 8) & 0xFF),
    uint8_t (param->Pickup_counter & 0xFF),
    0x00,
    0x00,
    0x00,
    0x00
  };

  uint8_t byte_value[4];
  memcpy(byte_value, &param->Battery_level, sizeof(float));
  for(int i=0; i<4; i++) return_buff[i+11] = byte_value[i]; 

  Serial.write(return_buff, sizeof(return_buff));
}

// SENSOR DATA PACKET RETURN
void Return_Sensor_Data(Param_t *param){
  param->Error_value = 0;
  
  uint8_t 
  len = 0x03,
  return_buff[] = {HEADER, HEADER, len, param->error_type.No_err, param->SensorA, param->SensorB};

  Serial.write(return_buff, sizeof(return_buff));
}

// NFC DATA PACKET RETURN
void Return_NFC_Data(Param_t *param){
  param->Error_value = 0;
  
  uint8_t 
  len = 0x06,
  return_buff[] = {
    HEADER, 
    HEADER, 
    len, 
    param->error_type.No_err,
    param->Tag_sign, 
    uint8_t((param->Tag_value >> 8) & 0xFF),
    param->Tag_value & 0xFF,
    uint8_t((param->Tag_num >> 8) & 0xFF),
    param->Tag_num & 0xFF,
  };

  Serial.write(return_buff, sizeof(return_buff));
}

// JOYSTICK DATA PACKET RETURN
void Return_Joystick_Data(Param_t *param){
  param->Error_value = 0;

  uint8_t
  len = 0x03,
  return_buff[] = {HEADER, HEADER, len, param->error_type.No_err, param->Xpos, param->Ypos};

  Serial.write(return_buff, sizeof(return_buff));
}

// ERROR PACKET RETURN
void Return_Error_Packet(Param_t *param){
  uint8_t
  len = 0x01,
  return_buff[] = {HEADER, HEADER, len, param->Error_value};

  Serial.write(return_buff, sizeof(return_buff));
}
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

// RECEIVE INSTRUCTION
void Receive_Instruction(Param_t *param){
  uint8_t
  data_len,
  len_validation;

  //Serial.flush();
  //for(int i=0; i<MAX_LEN; i++) rx_buff[i] = 0x00;

  // CHECK SERIAL AVAILABILITY
  if(Serial.available()){
    Serial.readBytes(rx_buff, MAX_LEN);

    if(rx_buff[0] == HEADER && rx_buff[1] == HEADER){
      memcpy(param->get_data, rx_buff, 16);

      data_len = rx_buff[2];
      param->data_length = data_len;

      len_validation = Length_calculator(rx_buff, data_len);
      param->length_validation = len_validation;

      // DATA LENGTH VALIDATION TRUE
      if(data_len == len_validation){
        // PING INSTRUCTION HANDLER
        if(rx_buff[3] == param->instruction_type.Ping){
          param->Instruction_get = param->instruction_type.Ping;
          param->Item_get = 0;
          Return_Ping(param);
        }

        // READ INSTRUCTION HANDLER ----------------------------------------------------
        else if(rx_buff[3] == param->instruction_type.Read){
          param->Instruction_get = param->instruction_type.Read;

          if(rx_buff[4] == param->item_type.Running_Mode){
            param->Item_get = param->item_type.Running_Mode;
            Return_Running_Mode(param);
          }
          else if(rx_buff[4] == param->item_type.Running_State){
            param->Item_get = param->item_type.Running_State;
            Return_Running_State(param);
          }
          else if(rx_buff[4] == param->item_type.AGV_Status){
            param->Item_get = param->item_type.AGV_Status;
            Return_AGV_Status(param);
          }
          else if(rx_buff[4] == param->item_type.Sensor_Data){
            param->Item_get = param->item_type.Sensor_Data;
            Return_Sensor_Data(param);
          }
          else if(rx_buff[4] == param->item_type.NFC_Data){
            param->Item_get = param->item_type.NFC_Data;
            Return_NFC_Data(param);
          }
          else if(rx_buff[4] == param->item_type.Joystick_Data){
            param->Item_get = param->item_type.Joystick_Data;
            Return_NFC_Data(param);
          }
        }
        //------------------------------------------------------------------------------

        // WRITE INSTRUCTION HANDLER ---------------------------------------------------------------
        else if(rx_buff[3] == param->instruction_type.Write){
          param->Instruction_get = param->instruction_type.Write;

          // DATA ITEM RUNNING MODE
          if(rx_buff[4] == param->item_type.Running_Mode){
            param->Item_get = param->item_type.Running_Mode;

            if(rx_buff[5] == param->sub_item_type.Sub_item1){
              if(rx_buff[6] == 0x00) param->Select_mode = NOT_SET;
              else if(rx_buff[6] == 0x01) param->Select_mode = LF_MODE;
              else param->Select_mode = LIDAR_MODE;
            }
            else if(rx_buff[5] == param->sub_item_type.Sub_item2){
              if(rx_buff[6] == 0x01) param->Base_speed = NORMAL_SPEED;
              else param->Base_speed = HIGH_SPEED;
            }

            param->Error_value = 0;
            Return_Error_Packet(param);
          }

          // DATA ITEM RUNNING STATE
          else if(rx_buff[4] == param->item_type.Running_State){
            param->Item_get = param->item_type.Running_State;

            if(rx_buff[5] == param->sub_item_type.Sub_item1){
              if(rx_buff[6] == 0x01) param->Select_state = START;
              else if(rx_buff[6] == 0x02) param->Select_state = STOP;
              else if(rx_buff[6] == 0x03) param->Select_state = PAUSE;

              if(rx_buff[7] == 0x01) param->Set_Direction = FORWARD;
              else if(rx_buff[7] == 0x02) param->Set_Direction = BACKWARD;
              else if(rx_buff[7] == 0x03) param->Set_Direction = LEFT;
              else if(rx_buff[7] == 0x04) param->Set_Direction = RIGHT;
              else if(rx_buff[7] == 0x05) param->Set_Direction = ROTATE_LEFT;
              else if(rx_buff[7] == 0x06) param->Set_Direction = ROTATE_RIGHT;
              else if(rx_buff[7] == 0x07) param->Set_Direction = BRAKE;
            }
            else if(rx_buff[5] == param->sub_item_type.Sub_item2){
              if(rx_buff[6] == 0x01) param->Set_Acceleration = NORMAL_ACCEL;
              else param->Set_Acceleration = REGENERATIVE_ACCEL;
            }
            else if(rx_buff[5] == param->sub_item_type.Sub_item3){
              if(rx_buff[6] == 0x01) param->Set_Braking = NORMAL_BRAKE;
              else param->Set_Braking = REGENERATIVE_BRAKE;
            }
      
            param->Error_value = 0;
            Return_Error_Packet(param);
          }

          // DATA ITEM JOYSTICK
          else if(rx_buff[4] == param->item_type.Joystick_Data){
            param->Item_get = param->item_type.Joystick_Data;

            param->Xpos = rx_buff[5];
            param->Ypos = rx_buff[6];

            param->Error_value = 0;
            Return_Error_Packet(param);
          }
        }
        //------------------------------------------------------------------------------------------

        // DATA INSTRUCTION INCORRECT
        else{
          param->Error_value = param->error_type.Instruction_err;
          Return_Error_Packet(param);
        }
      }

      // DATA LENGTH VALIDATION FALSE
      else{
        param->Error_value = param->error_type.Length_err;
        Return_Error_Packet(param);
      }
    }
  }
}
