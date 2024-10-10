#include "Wire.h"
#include "LiquidCrystal_I2C.h"
#include "SerialCom_Slave.h"

LiquidCrystal_I2C lcd(0x27, 16, 2);
Param_t parameter;

uint8_t dummy_rx[32];

void setup(){
  Serial1.begin(115200);

  lcd.init();
  lcd.backlight();
}

void loop(){
  Receive_Instruction(&parameter);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Len1: ");
  lcd.print(parameter.data_length);

  lcd.setCursor(8, 0);
  lcd.print("Len2: ");
  lcd.print(parameter.length_validation);

  lcd.setCursor(0, 1);
  lcd.print("Inst: ");
  if(parameter.Instruction_get == parameter.instruction.Ping){
    lcd.print("P");
  }
  else if(parameter.Instruction_get == parameter.instruction.Read){
    lcd.print("R");
  }
  else if(parameter.Instruction_get == parameter.instruction.Write){
    lcd.print("W");
  }

  lcd.setCursor(8, 1);
  lcd.print("Item: ");
  lcd.print(parameter.Item_get); 
  
  for(int i=0; i<parameter.data_length+3; i++){
    Serial.print("0x");
    Serial.print(parameter.get_data[i], HEX);
    Serial.print(" ");
  }
  Serial.println("");

  delay(100);
}
