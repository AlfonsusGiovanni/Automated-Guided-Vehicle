/*  
  NFC Master Tag Test Program
  PT. Stechoq Robotika Indonesia
  
  Date    : 23 SEPTEMBER 2024
  Author  : Alfonsus Giovanni Mahendra Putra - Universitas Diponegoro
*/

/*USER PRIVATE INCLUDE*/
#include "Wire.h"
#include "Adafruit_PN532.h"
#include "LiquidCrystal_I2C.h"

/*USER PRIVATE DEFINE*/
#define IRQ_NFC1 14
#define IRQ_NFC2 16
#define VIR_VCC1 15
#define VIR_VCC2 17

#define DATA_AUTH_HEADER 0xFF

/*USER PRIVATE TYPEDEF*/
LiquidCrystal_I2C lcd(0x27, 16, 2);
Adafruit_PN532 nfc(IRQ_NFC1, 2);
//Adafruit_PN532 nfc(2, &Serial2);

typedef enum {
  SUCCESS = 0x01,
  UID_READ_ERROR,
  BLOCK_AUTH_ERROR,
  DATA_READ_ERROR,
  DATA_WRITE_ERROR,
  TAG_READ_ERROR,
  TAG_WRITE_ERROR,
  UNREGISTERED_TAG,
} NFC_Status_t;

typedef enum {
  READ = 0x01,
  WRITE,
  ERASE,
} NFC_Data_Action_t;

typedef enum {
  NONE_SIGN,
  HOME_SIGN,
  CARRIER_SIGN,
  STATION_SIGN,
  TURN_SIGN,
  CROSS_SIGN,
} NFC_t;

typedef struct {
  uint8_t
    stored_data[16],
    uid[4],
    nfc_payload[13],
    tagType,
    tag_Xpos,
    tag_Ypos;

  uint16_t
    tagTypeValue,
    tagNum;
} Tag_Data_t;

Tag_Data_t NFC_Tag;

typedef enum {
  BOOT_MENU = 0x01,
  TYPE_MENU,
  VALUE_MENU,
  NUM_MENU,
  COORDINATE_MENU,
  WRITETAG_MENU,
  READTAG_MENU
} Menu_t;

Menu_t select_menu;

/*USER PRIVATE VARIABLE*/
const int btn_pin[4] = {1, 2, 3, 4};

uint8_t
  NFC_Timeout = 50,
  debounce_time = 1,
  btn_read[sizeof(btn_pin)],
  prev_btn_read[sizeof(btn_pin)],
  btn_state[sizeof(btn_pin)];

uint8_t
  refresh_rate = 1,
  menu_counter, prev_menu,
  multiplier_counter, multiplier_value,
  tag_type_counter,
  sel_pos_counter;

uint8_t
  gridWidth = 8,
  gridHeight = 8;

int
  tag_posX_counter,
  tag_posY_counter,
  tag_value_counter,
  tag_num_counter;

unsigned long
  prev_refreshTick,
  prev_checkTick,
  prev_debounceTick[sizeof(btn_pin)];

String
  TypeMenu = "SELECT TAG TYPE",
  ValueMenu = "SET TAG VALUE",
  NumMenu = "SET TAG NUM",
  PosMenu = "SET TAG POS",
  WriteMenu = "NFC WRITE",
  ReadMenu = "NFC READ",

  NoneSign = "None Sign",
  HomeSign = "Home Sign",
  CarrierSign = "Carrier Sign",
  StationSign = "Station Sign",
  TurnSign = "Turn Sign",
  CrossSign = "Cross Sign";

uint8_t
  total_menu = 6,
  total_type = 6;

/*USER PRIVATE FUNCTION*/
void Check_Button(void);
void UI_Handler(void);
void Show_Menu(Menu_t menu);

NFC_Status_t readUID(Tag_Data_t *Tag);
NFC_Status_t NFC_readData(Tag_Data_t *nfc_data);
NFC_Status_t NFC_writeData(uint8_t *Data_to_store);
NFC_Status_t NFC_eraseData(void);
NFC_Status_t NFC_readTag(Tag_Data_t *nfc_data);
NFC_Status_t NFC_writeTag(Tag_Data_t *data, NFC_t tag_sign, uint16_t tag_value, uint16_t tag_num, uint8_t tag_Xpos, uint8_t tag_Ypos);
void Check_TagData(void);


/*VOID SETUP*/
void setup() {
  Serial.begin(1000000);

  pinMode(VIR_VCC1, OUTPUT);
  pinMode(VIR_VCC2, OUTPUT);

  lcd.init();
  lcd.backlight();

  select_menu = BOOT_MENU;
  Show_Menu(select_menu);

  for (int i = 0; i < sizeof(btn_pin); i++) {
    pinMode(btn_pin[i], INPUT);
    prev_btn_read[i] = digitalRead(btn_pin[i]);
  }

  digitalWrite(VIR_VCC1, HIGH);
  digitalWrite(VIR_VCC2, LOW);

  nfc.begin();
  uint32_t versiondata = nfc.getFirmwareVersion();
  if (!versiondata) {
    Serial.println("Device Not Found");
    while (1)
      ;
  }

  Serial.print("Found chip PN5");
  Serial.println((versiondata >> 24) & 0xFF, HEX);
  Serial.print("Firmware ver. ");
  Serial.print((versiondata >> 16) & 0xFF, DEC);
  Serial.print('.');
  Serial.println((versiondata >> 8) & 0xFF, DEC);

  nfc.SAMConfig();

  Serial.println("Waiting for an ISO14443A Card ...");

  delay(2000);
  lcd.clear();
  select_menu = TYPE_MENU;
  Show_Menu(select_menu);
}

// VOID LOOP
void loop() {
  Serial.print("Writing...");
  for (int i = 0; i < 15; i++) {
    nfc.begin();
    nfc.SAMConfig();
    digitalWrite(VIR_VCC1, HIGH);
    digitalWrite(VIR_VCC2, LOW);
    if(NFC_writeTag(&NFC_Tag, TURN_SIGN, 40, 3, 0, 0) == SUCCESS){
      Serial.println("Write Success...");
      delay(2000);
      break;
    }
  }

  for (int i = 0; i < 100; i++) {
    nfc.begin();
    nfc.SAMConfig();
    digitalWrite(VIR_VCC1, HIGH);
    digitalWrite(VIR_VCC2, LOW);
    Check_TagData();
  }
  
  // UI_Handler();
  // Show_Menu(select_menu);
}

// BUTTON CHECK FUNCTION
void Check_Button(void) {
  for (int i = 0; i < sizeof(btn_read); i++) {
    btn_read[i] = digitalRead(btn_pin[i]);

    if (btn_read[i] != prev_btn_read[i] && btn_read[i] == LOW) {
      btn_state[i] = LOW;
    }

    else if (btn_read[i] != prev_btn_read[i] && btn_read[i] == HIGH) {
      btn_state[i] = HIGH;
      prev_debounceTick[i] = millis();
    }

    if (btn_state[i] == HIGH) {
      if (millis() - prev_debounceTick[i] > debounce_time) {
        prev_debounceTick[i] = millis();
        btn_state[i] = LOW;
      }
    }
    prev_btn_read[i] = btn_read[i];
  }
}

// UI HANDLER FUNCTION
void UI_Handler(void) {
  Check_Button();

  if (btn_state[0] == HIGH) {
    menu_counter++;
    if (menu_counter > total_menu - 1) menu_counter = 0;
  }

  if (menu_counter == 0) select_menu = TYPE_MENU;
  else if (menu_counter == 1) select_menu = VALUE_MENU;
  else if (menu_counter == 2) select_menu = NUM_MENU;
  else if (menu_counter == 3) select_menu = COORDINATE_MENU;
  else if (menu_counter == 4) select_menu = WRITETAG_MENU;
  else if (menu_counter == 5) select_menu = READTAG_MENU;

  if (select_menu == TYPE_MENU) {
    if (btn_state[1] == HIGH) tag_type_counter++;
    if (tag_type_counter > total_type - 1) tag_type_counter = 0;
  }

  else if (select_menu == VALUE_MENU) {
    if (btn_state[1] == HIGH)
      tag_value_counter += multiplier_value;

    else if (btn_state[2] == HIGH)
      tag_value_counter -= multiplier_value;

    else if (btn_state[3] == HIGH)
      multiplier_counter++;
    if (multiplier_counter > 5) multiplier_counter = 0;

    if (multiplier_counter == 0) multiplier_value = 1;
    else if (multiplier_counter == 1) multiplier_value = 5;
    else if (multiplier_counter == 2) multiplier_value = 10;
    else if (multiplier_counter == 3) multiplier_value = 20;
    else if (multiplier_counter == 4) multiplier_value = 50;
    else if (multiplier_counter == 5) multiplier_value = 100;

    if (tag_value_counter <= 0) tag_value_counter = 0;
  }

  else if (select_menu == NUM_MENU) {
    if (btn_state[1] == HIGH)
      tag_num_counter += multiplier_value;

    else if (btn_state[2] == HIGH)
      tag_num_counter -= multiplier_value;

    else if (btn_state[3] == HIGH)
      multiplier_counter++;
    if (multiplier_counter > 5) multiplier_counter = 0;

    if (multiplier_counter == 0) multiplier_value = 1;
    else if (multiplier_counter == 1) multiplier_value = 5;
    else if (multiplier_counter == 2) multiplier_value = 10;
    else if (multiplier_counter == 3) multiplier_value = 20;
    else if (multiplier_counter == 4) multiplier_value = 50;
    else if (multiplier_counter == 5) multiplier_value = 100;

    if (tag_num_counter <= 0) tag_num_counter = 0;
  }

  else if (select_menu == COORDINATE_MENU) {
    if (sel_pos_counter == 0) {
      if (btn_state[1] == HIGH)
        tag_posX_counter++;

      else if (btn_state[2] == HIGH)
        tag_posX_counter--;

      if (tag_posX_counter <= 0 || tag_posX_counter > gridWidth)
        tag_posX_counter = 0;
    }

    else if (sel_pos_counter == 1) {
      if (btn_state[1] == HIGH)
        tag_posY_counter++;

      else if (btn_state[2] == HIGH)
        tag_posY_counter--;

      if (tag_posY_counter <= 0 || tag_posY_counter > gridHeight)
        tag_posY_counter = 0;
    }

    if (btn_state[3] == HIGH)
      sel_pos_counter++;
    if (sel_pos_counter > 1) sel_pos_counter = 0;
  }

  else if (select_menu == WRITETAG_MENU) {

  }

  else if (select_menu == READTAG_MENU) {
  }
}

/*SHOW MENU FUNCTION*/
void Show_Menu(Menu_t menu) {
  if (prev_menu != menu) {
    multiplier_counter = 0;
    lcd.clear();
  } else if (prev_menu == menu) {
    for (int i = 0; i < sizeof(btn_state); i++) {
      if (btn_state[i] == HIGH) {
        lcd.clear();
      }
    }
  }

  if (menu == BOOT_MENU) {
    lcd.setCursor(0, 0);
    lcd.print("INITIALIZE...");
  }

  else if (menu == TYPE_MENU) {
    lcd.setCursor(0, 0);
    lcd.print(TypeMenu);

    lcd.setCursor(0, 1);
    lcd.print("->");

    lcd.setCursor(2, 1);
    switch (tag_type_counter) {
      case 0:
        lcd.print(NoneSign);
        break;

      case 1:
        lcd.print(HomeSign);
        break;

      case 2:
        lcd.print(CarrierSign);
        break;

      case 3:
        lcd.print(StationSign);
        break;

      case 4:
        lcd.print(TurnSign);
        break;

      case 5:
        lcd.print(CrossSign);
        break;
    }
  }

  else if (menu == VALUE_MENU) {
    lcd.setCursor(0, 0);
    lcd.print(ValueMenu);

    lcd.setCursor(0, 1);
    lcd.print(":");
    lcd.print(tag_value_counter);

    lcd.setCursor(12, 1);

    if (multiplier_value < 10)
      lcd.print("  x");
    else if (multiplier_value >= 10 && multiplier_value < 100)
      lcd.print(" x");
    else
      lcd.print("x");

    lcd.print(multiplier_value);
  }

  else if (menu == NUM_MENU) {
    lcd.setCursor(0, 0);
    lcd.print(NumMenu);

    lcd.setCursor(0, 1);
    lcd.print(":");
    lcd.print(tag_num_counter);

    lcd.setCursor(12, 1);

    if (multiplier_value < 10)
      lcd.print("  x");
    else if (multiplier_value >= 10 && multiplier_value < 100)
      lcd.print(" x");
    else
      lcd.print("x");

    lcd.print(multiplier_value);
  }

  else if (menu == COORDINATE_MENU) {
    lcd.setCursor(0, 0);
    lcd.print(PosMenu);

    if (sel_pos_counter == 0)
      lcd.setCursor(0, 1);

    else if (sel_pos_counter == 1)
      lcd.setCursor(8, 1);

    lcd.print(">");

    lcd.setCursor(1, 1);
    lcd.print("X:");
    lcd.print(tag_posX_counter);

    lcd.setCursor(9, 1);
    lcd.print("Y:");
    lcd.print(tag_posY_counter);
  }

  else if (menu == WRITETAG_MENU) {
    lcd.setCursor(0, 0);
    lcd.print(WriteMenu);
  }

  else if (menu == READTAG_MENU) {
    lcd.setCursor(0, 0);
    lcd.print(ReadMenu);
  }
  prev_menu = menu;
}

// READ NFC UID FUNCTION
NFC_Status_t readUID(Tag_Data_t *Tag) {
  uint8_t
    get_uid[7],
    uid_Length,
    read_success;

  read_success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, get_uid, &uid_Length, NFC_Timeout);

  if (read_success && uid_Length == 4) {
    for (int i = 0; i < uid_Length; i++) {
      Tag->uid[i] = get_uid[i];
    }
    return SUCCESS;
  }

  else return UID_READ_ERROR;
}


// READ DATA FROM NFC FUNCTION
NFC_Status_t NFC_readData(Tag_Data_t *nfc_data) {
  uint8_t
    get_uid[7],
    get_data[6],
    keyA[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },
    uid_Length,
    read_success;

  read_success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, get_uid, &uid_Length, NFC_Timeout);

  if(read_success && uid_Length == 4){
    for (int i = 0; i < uid_Length; i++) nfc_data->uid[i] = get_uid[i];
    read_success = nfc.mifareclassic_AuthenticateBlock(get_uid, uid_Length, 4, 0, keyA);

    if(read_success){

      uint8_t data[16];

      read_success = nfc.mifareclassic_ReadDataBlock(4, data);

      if (read_success) {
        for (int i = 0; i < sizeof(data); i++) {
          nfc_data->stored_data[i] = data[i];
        }
        return SUCCESS;
      } else return DATA_READ_ERROR;
    } else return BLOCK_AUTH_ERROR;
  } else return UID_READ_ERROR;
}


// WRITE DATA TO NFC FUNCTION
NFC_Status_t NFC_writeData(uint8_t *Data_to_store) {
  uint8_t
    get_uid[7],
    keyA[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },
    uid_Length,
    read_success,
    write_success;

  read_success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, get_uid, &uid_Length, NFC_Timeout);

  if (read_success && uid_Length == 4) {
    read_success = nfc.mifareclassic_AuthenticateBlock(get_uid, uid_Length, 4, 0, keyA);

    if (read_success) {
      write_success = nfc.mifareclassic_WriteDataBlock(4, Data_to_store);

      if (write_success) return SUCCESS;
      else return DATA_WRITE_ERROR;
    } else return BLOCK_AUTH_ERROR;
  } else return UID_READ_ERROR;
}


// ERASE NFC DATA FUNCTION
NFC_Status_t NFC_eraseData(void) {
  uint8_t data_write[16];
  for (int i = 0; i < sizeof(data_write); i++) data_write[i] = 0x00;

  if (NFC_writeData(data_write)) {
    Serial.println("(ERASE DATA SUCCESS)");
  }

  else {
    Serial.println("(CANT ERASE CARD DATA)");
  }
}


// READ TAG DATA FUNCTION
NFC_Status_t NFC_readTag(Tag_Data_t *nfc_data) {
  if (NFC_readData(nfc_data) == SUCCESS) {
    if (nfc_data->stored_data[0] == DATA_AUTH_HEADER && nfc_data->stored_data[1] == DATA_AUTH_HEADER) {
      nfc_data->tagType = nfc_data->stored_data[2];
      nfc_data->tagTypeValue = (nfc_data->stored_data[3] >> 8) | nfc_data->stored_data[4];
      nfc_data->tagNum = (nfc_data->stored_data[5] >> 8) | nfc_data->stored_data[6];
      nfc_data->tag_Xpos = nfc_data->stored_data[7];
      nfc_data->tag_Ypos = nfc_data->stored_data[8];

      for (int i = 0; i < sizeof(nfc_data->nfc_payload); i++) {
        if (i < 4) nfc_data->nfc_payload[i] = nfc_data->uid[i];
        else nfc_data->nfc_payload[i] = nfc_data->stored_data[i - 4];
      }
      return SUCCESS;
    } else return UNREGISTERED_TAG;
  } else return TAG_READ_ERROR;
}


// WRITE TAG DATA FUNCTION
NFC_Status_t NFC_writeTag(Tag_Data_t *data, NFC_t tag_sign, uint16_t tag_value, uint16_t tag_num, uint8_t tag_Xpos, uint8_t tag_Ypos) {
  uint8_t tag_data[16] = { DATA_AUTH_HEADER, DATA_AUTH_HEADER, tag_sign };

  tag_data[3] = (tag_value >> 8) & 0xFF;
  tag_data[4] = tag_value & 0xFF;
  tag_data[5] = (tag_num >> 8) & 0xFF;
  tag_data[6] = tag_num & 0xFF;
  tag_data[7] = tag_Xpos;
  tag_data[8] = tag_Ypos;

  if (NFC_writeData(tag_data) == SUCCESS) return SUCCESS;
  else return TAG_WRITE_ERROR;
}


/*--- CHECK NFC DATA ---*/
void Check_TagData(void) {
  if (NFC_readTag(&NFC_Tag) == SUCCESS) {
    Serial.println("(READ DATA SUCCESS)");

    Serial.print("Tag ID: ");
    for (int i = 0; i < 4; i++) {
      Serial.print("0x");
      Serial.print(NFC_Tag.nfc_payload[i], HEX);
      Serial.print(" ");
    }

    Serial.print(" ");

    Serial.print("Tag Data Header: ");
    for (int i = 4; i < 6; i++) {
      Serial.print("0x");
      Serial.print(NFC_Tag.nfc_payload[i], HEX);
      Serial.print(" ");
    }

    Serial.print(" ");

    Serial.print("Tag Data: ");
    for (int i = 6; i < sizeof(NFC_Tag.nfc_payload); i++) {
      Serial.print("0x");
      Serial.print(NFC_Tag.nfc_payload[i], HEX);
      Serial.print(" ");
    }

    Serial.println(" ");
  }

  else {
    Serial.println("(CANT READ DATA FROM CARD)");
  }
}
