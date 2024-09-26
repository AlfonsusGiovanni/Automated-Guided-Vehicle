/*  
  NFC Master Tag Test Program
  PT. Stechoq Robotika Indonesia
  
  Date    : 23 SEPTEMBER 2024
  Author  : Alfonsus Giovanni Mahendra Putra - Universitas Diponegoro
*/

/*USER PRIVATE INCLUDE*/
//************************************************************************************************************************************************************************************
#include "Wire.h" 
#include "Adafruit_NFCShield_I2C.h"
#include "LiquidCrystal_I2C.h"
//************************************************************************************************************************************************************************************


/*USER PRIVATE DEFINE*/
//************************************************************************************************************************************************************************************
#define IRQ     2
#define RESET   3

#define DATA_AUTH_HEADER1   0x5A
#define DATA_AUTH_HEADER2   0xA5
#define DATA_CMD_STATION    0xFE
#define DATA_CMD_WAYPOINT   0xFF
//************************************************************************************************************************************************************************************


/*USER PRIVATE TYPEDEF*/
//************************************************************************************************************************************************************************************
LiquidCrystal_I2C lcd(0x27,20,4);
Adafruit_NFCShield_I2C nfc(IRQ, RESET);

typedef enum{
  SUCCESS = 0x01,
  UID_READ_ERROR,
  BLOCK_AUTH_ERROR,
  DATA_READ_ERROR, 
  DATA_WRITE_ERROR,
  TAG_READ_ERROR,
  TAG_WRITE_ERROR,
  UNREGISTERED_TAG,
}NFC_Status_t;

typedef enum{
  READ = 0x01,
  WRITE,
  ERASE,
}NFC_Data_Action_t;

typedef struct{
  uint8_t type;
  uint16_t value;
}Tag_Data_t;

Tag_Data_t NFC_Tag;
//************************************************************************************************************************************************************************************


/*USER PRIVATE VARIABLE*/
//************************************************************************************************************************************************************************************
uint8_t
nfc_data[16],
nfc_uid[4];
//************************************************************************************************************************************************************************************


/*USER PRIVATE FUNCTION*/
//************************************************************************************************************************************************************************************
NFC_Status_t readUID(uint8_t *stored_UID);
NFC_Status_t NFC_readData(uint8_t *stored_Data);
NFC_Status_t NFC_writeData(uint8_t *Data_to_store);
NFC_Status_t NFC_eraseData(void);
NFC_Status_t NFC_readTag(Tag_Data_t *data);
NFC_Status_t NFC_writeTag(Tag_Data_t *data, uint8_t tag_type, uint16_t tag_value);


void check_nfc_UID(void);
void check_nfc_Data(NFC_Data_Action_t action);
//************************************************************************************************************************************************************************************


/*VOID SETUP*/
//************************************************************************************************************************************************************************************
void setup(){
  Serial.begin(115200);

  lcd.init();
  lcd.backlight();

  nfc.begin();

  uint32_t versiondata = nfc.getFirmwareVersion();
  if (!versiondata) {
    Serial.print("Device Not Found");
    while (1);
  }
  
  Serial.print("Found chip PN5"); Serial.println((versiondata>>24) & 0xFF, HEX); 
  Serial.print("Firmware ver. "); Serial.print((versiondata>>16) & 0xFF, DEC); 
  Serial.print('.'); Serial.println((versiondata>>8) & 0xFF, DEC);
  
  nfc.SAMConfig();
  
  Serial.println("Waiting for an ISO14443A Card ...");
}
//************************************************************************************************************************************************************************************


/*VOID LOOP*/
//************************************************************************************************************************************************************************************
void loop(){
  
}
//************************************************************************************************************************************************************************************


/*--- READ NFC UID ---*/
//************************************************************************************************************************************************************************************
NFC_Status_t readUID(uint8_t *stored_UID){
  uint8_t
  get_uid[7],
  uid_Length,
  read_success;
  
  read_success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, get_uid, &uid_Length);

  if(read_success && uid_Length == 4){
    for(int i=0; i<uid_Length; i++){
      stored_UID[i] = get_uid[i];
    }
    return SUCCESS;
  }

  else return UID_READ_ERROR;
}
//************************************************************************************************************************************************************************************


/*--- READ DATA FROM NFC ---*/
//************************************************************************************************************************************************************************************
NFC_Status_t NFC_readData(uint8_t *stored_Data){
  uint8_t
  get_uid[7],
  get_data[6],
  keyA[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
  uid_Length,
  read_success;

  read_success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, get_uid, &uid_Length);

  if(read_success && uid_Length == 4){
    read_success = nfc.mifareclassic_AuthenticateBlock(get_uid, uid_Length, 4, 0, keyA);
    
    if(read_success){
      uint8_t data[16];

      read_success = nfc.mifareclassic_ReadDataBlock(4, data);

      if(read_success){
        for(int i=0; i<sizeof(data); i++){
          stored_Data[i] = data[i];
        }
        return SUCCESS;
      }
      else return DATA_READ_ERROR;
    }
    else return BLOCK_AUTH_ERROR;
  }
  else return UID_READ_ERROR;
}
//************************************************************************************************************************************************************************************


/*--- WRITE DATA TO NFC ---*/
//************************************************************************************************************************************************************************************
NFC_Status_t NFC_writeData(uint8_t *Data_to_store){
  uint8_t
  get_uid[7],
  keyA[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
  uid_Length,
  read_success,
  write_success;

  read_success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, get_uid, &uid_Length);

  if(read_success && uid_Length == 4){
    read_success = nfc.mifareclassic_AuthenticateBlock(get_uid, uid_Length, 4, 0, keyA);
    
    if(read_success){
      write_success = nfc.mifareclassic_WriteDataBlock(4, Data_to_store);

      if(write_success) return SUCCESS;
      else return DATA_WRITE_ERROR;
    }
    else return BLOCK_AUTH_ERROR;
  }
  else return UID_READ_ERROR;
}
//************************************************************************************************************************************************************************************


/*--- ERASE NFC DATA ---*/
//************************************************************************************************************************************************************************************
NFC_Status_t NFC_eraseData(void){
  uint8_t data_write[16];
  for(int i=0; i<sizeof(data_write); i++) data_write[i] = 0x00;

  if(NFC_writeData(data_write)){
    Serial.println("(ERASE DATA SUCCESS)");
  }

  else{
    Serial.println("(CANT ERASE CARD DATA)");
  }
}
//************************************************************************************************************************************************************************************


/*--- READ TAG DATA ---*/
//************************************************************************************************************************************************************************************
NFC_Status_t NFC_readTag(Tag_Data_t *data){
  uint8_t tag_data[16];

  if(NFC_readData(tag_data) == SUCCESS){
    if(tag_data[0] == DATA_AUTH_HEADER1 && tag_data[1] == DATA_AUTH_HEADER2){
      data->type = tag_data[2];
      data->value = (tag_data[3] >> 8) | tag_data[4];
      
      return SUCCESS;
    }
    else return UNREGISTERED_TAG;
  } 
  else return TAG_READ_ERROR;
}
//************************************************************************************************************************************************************************************


/*--- WRITE TAG DATA ---*/
//************************************************************************************************************************************************************************************
NFC_Status_t NFC_writeTag(Tag_Data_t *data, uint8_t tag_type, uint16_t tag_value){
  uint8_t tag_data[16] = {DATA_AUTH_HEADER1, DATA_AUTH_HEADER2, tag_type};

  tag_data[3] = (tag_value >> 8) & 0xFF;
  tag_data[4] = tag_value & 0xFF;

  if(NFC_writeData(tag_data) == SUCCESS) return SUCCESS;
  else return TAG_WRITE_ERROR;
}
//************************************************************************************************************************************************************************************


/*--- CHECK NFC ID ---*/
//************************************************************************************************************************************************************************************
void check_nfc_UID(void){
  uint8_t uid_check[4];

  if(readUID(uid_check) == SUCCESS){
    Serial.print("UID: ");
    for(int i=0; i<sizeof(uid_check); i++){
      Serial.print("0x");
      Serial.print(uid_check[i], HEX);
      Serial.print(" ");
    }
    Serial.println("(READ UID SUCCESS)");
  }

  else{
    Serial.println("(CARD NOT DETECTED)");
  }
}
//************************************************************************************************************************************************************************************


/*--- CHECK NFC DATA ---*/
//************************************************************************************************************************************************************************************
void check_nfc_Data(NFC_Data_Action_t action){
  uint8_t data_check[16];

  if(action == READ){
    if(NFC_readData(data_check) == SUCCESS){
      Serial.print("Data: ");
      for(int i=0; i<sizeof(data_check); i++){
        Serial.print("0x");
        Serial.print(data_check[i], HEX);
        Serial.print(" ");
      }
      Serial.println("(READ DATA SUCCESS)");
    }

    else{
      Serial.println("(CANT READ DATA FROM CARD)");
    }
  }
  
  else if(action == WRITE){
    uint8_t data_write[16];
    for(int i=0; i<sizeof(data_write); i++) data_write[i] = 0xFF;

    if(NFC_writeData(data_write)){
      Serial.println("(WRITE DATA SUCCESS)");
    }

    else{
      Serial.println("(CANT WRITE DATA TO CARD)");
    }
  }
}
//************************************************************************************************************************************************************************************
