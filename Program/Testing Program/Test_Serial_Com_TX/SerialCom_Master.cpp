/*  
  SERIAL COMMUNICATION LIBRARY
  PT. Stechoq Robotika Indonesia
  
  Date    : 27 SEPTEMBER 2024
  Author  : Alfonsus Giovanni Mahendra Putra - Universitas Diponegoro
*/

#include "SerialCom_Master.h"

static uint8_t rx_buff[MAX_LEN];

/*INSTRUCTION FUNCTION*/
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

// SEND PING
void Send_Ping(Param_t *param){
  uint8_t
  len = 0x01,
  tx_buff[] = {HEADER, HEADER, len, param->instruction.Ping};

  Serial1.write(tx_buff, sizeof(tx_buff));
  Serial1.readBytes(rx_buff, MAX_LEN);
  
  if(rx_buff[0] == HEADER && rx_buff[1] == HEADER){
    param->data_length = rx_buff[2];
    memcpy(param->return_data, rx_buff, 5);
  }
}

// SET RUNNING MODE
void Set_Running_Mode(Param_t *param, Select_mode_data_t mode){
  uint8_t
  len = 0x04,
  tx_buff[] = {HEADER, HEADER, len, param->instruction.Write, param->item.Running_Mode, param->sub_item.Sub_item1, mode};

  Serial1.write(tx_buff, sizeof(tx_buff));
  Serial1.readBytes(rx_buff, MAX_LEN);
  
  if(rx_buff[0] == HEADER && rx_buff[1] == HEADER){
    param->data_length = rx_buff[2];
    memcpy(param->return_data, rx_buff, 4);
  }
}

// SET RUNNING BASE SPEED
void Set_Running_BaseSpeed(Param_t *param, uint8_t base_speed){
  uint8_t
  len = 0x04,
  tx_buff[] = {HEADER, HEADER, len, param->instruction.Write, param->item.Running_Mode, param->sub_item.Sub_item2, base_speed};
  
  Serial1.write(tx_buff, sizeof(tx_buff));
  Serial1.readBytes(rx_buff, MAX_LEN);
  
  if(rx_buff[0] == HEADER && rx_buff[1] == HEADER){
    param->data_length = rx_buff[2];
    memcpy(param->return_data, rx_buff, 4);
  }
}

// SET RUNNING STATE
void Set_Running_State(Param_t *param, Select_state_data_t state, Select_dir_data_t dir){
  uint8_t
  len = 0x05,
  tx_buff[] = {HEADER, HEADER, len, param->instruction.Write, param->item.Running_State, param->sub_item.Sub_item1, state, dir};

  Serial1.write(tx_buff, sizeof(tx_buff));
  Serial1.readBytes(rx_buff, MAX_LEN);

  if(rx_buff[0] == HEADER && rx_buff[1] == HEADER){
    param->data_length = rx_buff[2];
    memcpy(param->return_data, rx_buff, 4);
  }
}

// SET RUNNING ACCELERATION
void Set_Running_Accel(Param_t *param, Accel_data_t accel){
  uint8_t
  len = 0x04,
  tx_buff[] = {HEADER, HEADER, len, param->instruction.Write, param->item.Running_State, param->sub_item.Sub_item2, accel};

  Serial1.write(tx_buff, sizeof(tx_buff));
  Serial1.readBytes(rx_buff, MAX_LEN);

  if(rx_buff[0] == HEADER && rx_buff[1] == HEADER){
    param->data_length = rx_buff[2];
    memcpy(param->return_data, rx_buff, 4);
  }
}

// SET RUNNING BRAKE MODE
void Set_Running_Brake(Param_t *param, Brake_data_t brake){
  uint8_t
  len = 0x04,
  tx_buff[] = {HEADER, HEADER, len, param->instruction.Write, param->item.Running_State, param->sub_item.Sub_item3, brake};

  Serial1.write(tx_buff, sizeof(tx_buff));
  Serial1.readBytes(rx_buff, MAX_LEN);

  if(rx_buff[0] == HEADER && rx_buff[1] == HEADER){
    param->data_length = rx_buff[2];
    memcpy(param->return_data, rx_buff, 4);
  }
}

// SET JOYSTIK VALUE
void Set_Joystick(Param_t *param, uint8_t Xvalue, uint8_t Yvalue){
  uint8_t
  len = 0x04,
  tx_buff[] = {HEADER, HEADER, len, param->instruction.Write, param->item.Joystick_Data, Xvalue, Yvalue};

  Serial1.write(tx_buff, sizeof(tx_buff));
  Serial1.readBytes(rx_buff, MAX_LEN);

  if(rx_buff[0] == HEADER && rx_buff[1] == HEADER){
    param->data_length = rx_buff[2];
    memcpy(param->return_data, rx_buff, 4);
  }
}

// READ RUNNING MODE
void Read_Running_Mode(Param_t *param){
  uint8_t
  len = 0x02,
  tx_buff[] = {HEADER, HEADER, len, param->instruction.Read, param->item.Running_Mode};
  
  Serial1.write(tx_buff, sizeof(tx_buff));
  Serial1.readBytes(rx_buff, MAX_LEN);

  if(rx_buff[0] == HEADER && rx_buff[1] == HEADER && rx_buff[3] == 0){
    param->data_length = rx_buff[2];
    
    if(rx_buff[4] == 0x00) param->Select_mode = NOT_SET;
    else if(rx_buff[4] == 0x01) param->Select_mode = LF_MODE;
    else param->Select_mode = LIDAR_MODE;
    param->Base_speed = rx_buff[5];
    memcpy(param->return_data, rx_buff, 6);
  }
  else if(rx_buff[0] == HEADER && rx_buff[1] == HEADER && rx_buff[3] != 0){
    memcpy(param->return_data, rx_buff, 6);
  }
}

// READ RUNNING STATE
void Read_Running_State(Param_t *param){
  uint8_t
  len = 0x02,
  tx_buff[] = {HEADER, HEADER, len, param->instruction.Read, param->item.Running_State};

  Serial1.write(tx_buff, sizeof(tx_buff));
  Serial1.readBytes(rx_buff, MAX_LEN);

  if(rx_buff[0] == HEADER && rx_buff[1] == HEADER && rx_buff[3] == 0){
    param->data_length = rx_buff[2];

    if(rx_buff[4] == 0x01) param->Select_state = START;
    else if(rx_buff[4] == 0x02) param->Select_state = STOP;
    else param->Select_state = PAUSE;

    if(rx_buff[5] == 0x01) param->Set_Direction = FORWARD;
    else if(rx_buff[5] == 0x02) param->Set_Direction = BACKWARD;
    else if(rx_buff[5] == 0x03) param->Set_Direction = LEFT;
    else if(rx_buff[5] == 0x04) param->Set_Direction = RIGHT;
    else if(rx_buff[5] == 0x05) param->Set_Direction = ROTATE_LEFT;
    else if(rx_buff[5] == 0x06) param->Set_Direction = ROTATE_RIGHT;
    else if(rx_buff[5] == 0x07) param->Set_Direction = BRAKE;

    if(rx_buff[6] == 0x01) param->Set_Acceleration = NORMAL_ACCEL;
    else param->Set_Acceleration = REGENERATIVE_ACCEL;

    if(rx_buff[7] == 0x01) param->Set_Breaking = NORMAL_BRAKE;
    else param->Set_Breaking = REGENERATIVE_BRAKE;

    memcpy(param->return_data, rx_buff, 8);
  }
  else if(rx_buff[0] == HEADER && rx_buff[1] == HEADER && rx_buff[3] != 0){
    memcpy(param->return_data, rx_buff, 8);
  }
}

// READ AGV STATUS
void Read_AGV_Status(Param_t *param){
  uint8_t
  len = 0x02,
  tx_buff[] = {HEADER, HEADER, len, param->instruction.Read, param->item.AGV_Status};

  Serial1.write(tx_buff, sizeof(tx_buff));
  Serial1.readBytes(rx_buff, MAX_LEN);

  if(rx_buff[0] == HEADER && rx_buff[1] == HEADER && rx_buff[3] == 0){
    param->data_length = rx_buff[2];

    if(rx_buff[4] == 0x01) param->Position = HOME;
    else if(rx_buff[4] == 0x02) param->Position = ON_STATION;
    else param->Position = ON_THE_WAY;

    param->Pos_value = (rx_buff[5] << 8) | rx_buff[6];
    param->Send_counter = (rx_buff[7] << 8) | rx_buff[8];
    param->Pickup_counter = (rx_buff[9] << 8) | rx_buff[10];

    uint8_t data_bytes[4] = {rx_buff[11], rx_buff[12], rx_buff[13], rx_buff[14]};
    memcpy(&param->Battery_level, data_bytes, sizeof(float));

    memcpy(param->return_data, rx_buff, 15);
  }
  else if(rx_buff[0] == HEADER && rx_buff[1] == HEADER && rx_buff[3] != 0){
    memcpy(param->return_data, rx_buff, 15);
  }
}

// READ SENSOR DATA STATE
void Read_Sensor_Data(Param_t *param){
  uint8_t
  len = 0x02,
  tx_buff[] = {HEADER, HEADER, len, param->instruction.Read, param->item.Sensor_Data};

  Serial1.write(tx_buff, sizeof(tx_buff));
  Serial1.readBytes(rx_buff, MAX_LEN);

  if(rx_buff[0] == HEADER && rx_buff[1] == HEADER && rx_buff[3] == 0){
    param->data_length = rx_buff[2];

    if(rx_buff[4] == 0x01) param->SensorA = DETECTED;
    else param->SensorA = NOT_DETECTED;

    if(rx_buff[5] == 0x01) param->SensorB = DETECTED;
    else param->SensorB = NOT_DETECTED;

    memcpy(param->return_data, rx_buff, 6);
  }
  else if(rx_buff[0] == HEADER && rx_buff[1] == HEADER && rx_buff[3] != 0){
    memcpy(param->return_data, rx_buff, 6);
  }
}

// READ NFC DATA
void Read_NFC_Data(Param_t *param){
  uint8_t
  len = 0x02,
  tx_buff[] = {HEADER, HEADER, len, param->instruction.Read, param->item.NFC_Data};

  Serial1.write(tx_buff, sizeof(tx_buff));
  Serial1.readBytes(rx_buff, MAX_LEN);

  if(rx_buff[0] == HEADER && rx_buff[1] == HEADER && rx_buff[3] == 0){
    param->data_length = rx_buff[2];

    if(rx_buff[4] == 0x01) param->Tag_position = HOME;
    else if(rx_buff[4] == 0x02) param->Tag_position = ON_STATION;
    else param->Tag_position = ON_THE_WAY;

    param->Tag_value = (rx_buff[5] << 8) | rx_buff[6];

    memcpy(param->return_data, rx_buff, 7);
  }
  else if(rx_buff[0] == HEADER && rx_buff[1] == HEADER && rx_buff[3] != 0){
    memcpy(param->return_data, rx_buff, 6);
  }
}

//  READ JOYSTICK DATA
void Read_Joystick_Data(Param_t *param){
  uint8_t
  len = 0x02,
  tx_buff[] = {HEADER, HEADER, len, param->instruction.Read, param->item.Joystick_Data};

  Serial1.write(tx_buff, sizeof(tx_buff));
  Serial1.readBytes(rx_buff, MAX_LEN);

  if(rx_buff[0] == HEADER && rx_buff[1] == HEADER && rx_buff[3] == 0){
    param->data_length = rx_buff[2];

    param->Xpos = rx_buff[4];
    param->Ypos = rx_buff[5];
    memcpy(param->return_data, rx_buff, 6);
  }
  else if(rx_buff[0] == HEADER && rx_buff[1] == HEADER && rx_buff[3] != 0){
    memcpy(param->return_data, rx_buff, 6);
  }
}
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

