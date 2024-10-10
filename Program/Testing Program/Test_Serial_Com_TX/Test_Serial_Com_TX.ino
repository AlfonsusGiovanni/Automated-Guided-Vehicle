#include "SerialCom_Master.h"

Param_t parameter;

uint8_t dummy_rx[32];

void setup(){
  Serial.begin(115200);
  Serial1.begin(115200);
}

void loop(){
  Read_AGV_Status(&parameter);

  for(int i=0; i<parameter.data_length+3; i++){
    Serial.print("0x");
    Serial.print(parameter.return_data[i], HEX);
    Serial.print(" ");
  }
  Serial.println("");
  delay(10);
}
