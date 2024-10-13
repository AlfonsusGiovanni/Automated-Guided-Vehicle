/*  
  NFC Master Tag Test Program
  PT. Stechoq Robotika Indonesia
  
  Date    : 23 SEPTEMBER 2024
  Author  : Alfonsus Giovanni Mahendra Putra - Universitas Diponegoro
*/

/*USER PRIVATE INCLUDE*/
//************************************************************************************************************************************************************************************
#include "SoftwareSerial.h"
#include "Wire.h" 
#include "Adafruit_PN532.h"
#include "LiquidCrystal_I2C.h"
//************************************************************************************************************************************************************************************


/*USER PRIVATE DEFINE*/
//************************************************************************************************************************************************************************************
#define DATA_AUTH_HEADER1   0x5A
#define DATA_AUTH_HEADER2   0xA5
#define DATA_CMD_STATION    0xFE
#define DATA_CMD_WAYPOINT   0xFF
//************************************************************************************************************************************************************************************


/*USER PRIVATE TYPEDEF*/
//************************************************************************************************************************************************************************************
LiquidCrystal_I2C lcd(0x27,20,4);
Adafruit_PN532 nfc1(3, &Serial2);
Adafruit_PN532 nfc2(3, &Serial3);

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

typedef enum{
  NFC_COM1 = 0x01, 
  NFC_COM2,
}NFC_Com_t;

Tag_Data_t NFC_Tag;
//************************************************************************************************************************************************************************************


/*USER PRIVATE VARIABLE*/
//************************************************************************************************************************************************************************************
uint8_t
nfc_data[16],
nfc_uid[4];

uint8_t NFC_Timeout = 100;
//************************************************************************************************************************************************************************************


/*USER PRIVATE FUNCTION*/
//************************************************************************************************************************************************************************************
NFC_Status_t readUID(Adafruit_PN532 &nfc, uint8_t *stored_UID);
NFC_Status_t NFC_readData(Adafruit_PN532 &nfc, uint8_t *stored_Data);
NFC_Status_t NFC_writeData(Adafruit_PN532 &nfc, uint8_t *Data_to_store);
NFC_Status_t NFC_eraseData(Adafruit_PN532 &nfc);
NFC_Status_t NFC_readTag(Adafruit_PN532 &nfc,Tag_Data_t *data);
NFC_Status_t NFC_writeTag(Adafruit_PN532 &nfc, Tag_Data_t *data, uint8_t tag_type, uint16_t tag_value);


void check_nfc_UID(Adafruit_PN532 &nfc);
void check_nfc_Data(Adafruit_PN532 &nfc, NFC_Data_Action_t action);
//************************************************************************************************************************************************************************************


/*VOID SETUP*/
//************************************************************************************************************************************************************************************
void setup(){
  Serial.begin(115200);
  
  nfc1.begin();
  nfc2.begin();

  uint32_t versiondata1 = nfc1.getFirmwareVersion();
  while(!versiondata1) {
    versiondata1 = nfc1.getFirmwareVersion();
    Serial.println("Device (1) Not Found");
    delay(100);
  }
  
  Serial.print("Found chip (1) PN5"); Serial.println((versiondata1>>24) & 0xFF, HEX); 
  Serial.print("Firmware ver. "); Serial.print((versiondata1>>16) & 0xFF, DEC); 
  Serial.print('.'); Serial.println((versiondata1>>8) & 0xFF, DEC);

  delay(2000);

  uint32_t versiondata2 = nfc2.getFirmwareVersion();
  while(!versiondata2) {
    versiondata2 = nfc2.getFirmwareVersion();
    Serial.println("Device (2) Not Found");
    delay(100);
  }

  Serial.print("Found chip (2) PN5"); Serial.println((versiondata2>>24) & 0xFF, HEX); 
  Serial.print("Firmware ver. "); Serial.print((versiondata2>>16) & 0xFF, DEC); 
  Serial.print('.'); Serial.println((versiondata2>>8) & 0xFF, DEC);
  
  //nfc1.SAMConfig();
  //nfc2.SAMConfig();
  
  Serial.println("Waiting for an ISO14443A Card ...");
  delay(2000);
}
//************************************************************************************************************************************************************************************

int counter;

/*VOID LOOP*/
//************************************************************************************************************************************************************************************
void loop(){
  check_nfc_UID(nfc1);
  check_nfc_UID(nfc2);
}
//************************************************************************************************************************************************************************************


/*--- READ NFC UID ---*/
//************************************************************************************************************************************************************************************
NFC_Status_t readUID(Adafruit_PN532 &nfc, uint8_t *stored_UID){
  uint8_t
  get_uid[7],
  uid_Length,
  read_success;

  read_success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, get_uid, &uid_Length, NFC_Timeout);
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
NFC_Status_t NFC_readData(Adafruit_PN532 &nfc, uint8_t *stored_Data){
  uint8_t
  get_uid[7],
  get_data[6],
  keyA[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
  uid_Length,
  read_success;

  read_success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, get_uid, &uid_Length, NFC_Timeout);
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
NFC_Status_t NFC_writeData(Adafruit_PN532 &nfc, uint8_t *Data_to_store){
  uint8_t
  get_uid[7],
  keyA[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
  uid_Length,
  read_success,
  write_success;

  read_success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, get_uid, &uid_Length, NFC_Timeout);
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
NFC_Status_t NFC_eraseData(Adafruit_PN532 &nfc){
  uint8_t data_write[16];
  for(int i=0; i<sizeof(data_write); i++) data_write[i] = 0x00;

  if(NFC_writeData(nfc, data_write)) Serial.println("(NFC 1 ERASE DATA SUCCESS)");
  else Serial.println("(NFC 1 CANT ERASE CARD DATA)");
}
//************************************************************************************************************************************************************************************


/*--- READ TAG DATA ---*/
//************************************************************************************************************************************************************************************
NFC_Status_t NFC_readTag(Adafruit_PN532 &nfc, Tag_Data_t *data){
  uint8_t tag_data[16];

  if(NFC_readData(nfc, tag_data) == SUCCESS){
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
NFC_Status_t NFC_writeTag(Adafruit_PN532 &nfc, Tag_Data_t *data, uint8_t tag_type, uint16_t tag_value){
  uint8_t tag_data[16] = {DATA_AUTH_HEADER1, DATA_AUTH_HEADER2, tag_type};

  tag_data[3] = (tag_value >> 8) & 0xFF;
  tag_data[4] = tag_value & 0xFF;

  if(NFC_writeData(nfc, tag_data) == SUCCESS) return SUCCESS;
  else return TAG_WRITE_ERROR;
}
//************************************************************************************************************************************************************************************


/*--- CHECK NFC ID ---*/
//************************************************************************************************************************************************************************************
void check_nfc_UID(Adafruit_PN532 &nfc){
  uint8_t uid_check[4];

  if(readUID(nfc, uid_check) == SUCCESS){
    Serial.print("UID: ");
    for(int i=0; i<sizeof(uid_check); i++){
      Serial.print("0x");
      Serial.print(uid_check[i], HEX);
      Serial.print(" ");
    }
    Serial.println(": READ UID SUCCESS");
  }

  else{
    Serial.println(": CARD NOT DETECTED");
  }
}
//************************************************************************************************************************************************************************************


/*--- CHECK NFC DATA ---*/
//************************************************************************************************************************************************************************************
void check_nfc_Data(Adafruit_PN532 &nfc, NFC_Data_Action_t action){
  uint8_t data_check[16];

  if(action == READ){
    if(NFC_readData(nfc, data_check) == SUCCESS){
      Serial.print("Data: ");
      for(int i=0; i<sizeof(data_check); i++){
        Serial.print("0x");
        Serial.print(data_check[i], HEX);
        Serial.print(" ");
      }
      Serial.println(": READ DATA SUCCESS");
    }

    else{
      Serial.println(": CANT READ DATA FROM CARD");
    }
  }
  
  else if(action == WRITE){
    uint8_t data_write[16];
    for(int i=0; i<sizeof(data_write); i++) data_write[i] = 0xFF;

    if(NFC_writeData(nfc, data_write)){
      Serial.println(": WRITE DATA SUCCESS");
    }

    else{
      Serial.println(": CANT WRITE DATA TO CARD");
    }
  }
}
//************************************************************************************************************************************************************************************
