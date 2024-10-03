#include "Serial_Com.h"

static uint8_t rx_data[BUFF_LEN];

void TX_Serial(uint8_t pos, uint16_t send_cnt, uint16_t pickup_cnt, float battery){
  uint8_t battery_val[4];
  memcpy(battery_val, &battery, sizeof(float));
  
  uint8_t tx_data[BUFF_LEN] = {
    HEADER,
    HEADER,
    pos,
    (send_cnt >> 8) & 0xFF,
    send_cnt & 0xFF,
    (pickup_cnt >> 8) & 0xFF,
    pickup_cnt & 0xFF,
    battery_val[0],
    battery_val[1],
    battery_val[2],
    battery_val[3],
    TAIL
  };

  Serial.write(tx_data, BUFF_LEN);
}

void RX_Serial(Data_t *get){
  Serial.readBytes(rx_data, BUFF_LEN);
  uint8_t battery_val[4];

  for(int i=0; i<BUFF_LEN; i++){
    if(rx_data[i] = HEADER && rx_data[i+1] == HEADER && rx_data[i+11] == TAIL){
      get->position = rx_data[i+2];
      get->send_counter = (rx_data[i+3] << 8) | rx_data[i+4];
      get->pickup_counter = (rx_data[i+5] << 8) | rx_data[i+6];

      for(int j=0; j<4; j++) battery_val[j] = rx_data[i+7+j];
      memcpy(&get->bat_lvl , rx_data, sizeof(float));
    }
  }
}
