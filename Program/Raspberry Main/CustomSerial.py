# Raspberry Serial Communication Handler Program V2
# PT Stechoq Robotika Indonesia
# Date	 : 25 Oktober 2024
# Author : Alfonsus Giovanni Mahendra Putra - Universitas Diponegoro

# USED LIBRARY -----
import serial
import struct
import array as arr
import time
# ------------------

# SERIAL SETUP ---------
baudRate = 1000000
UARTPort = '/dev/serial1'
USBPort = '/dev/ttyUSB0'
ArduinoPort = 'COM9'
timeout = 0
# ----------------------


# Instruction
class Instruction:
    Ping = 0x01
    Read = 0x02
    Write = 0x03

# Command Item
class Item:
    Running_Mode = 0x01
    Running_State = 0x02
    AGV_Status = 0x03
    Sensor_Data = 0x04
    NFC_Data = 0x05
    Joystick_Data = 0x06

# Command Sub Item
class SubItem:
    Sub_item1 = 0x01
    Sub_item2 = 0x02
    Sub_item3 = 0x03

# Command Error
class Error:
    No_err = 0x00
    Instruction_err = 0x01
    Item_err = 0x02
    Length_err = 0x04
    Tag_reading_err = 0x08
    LoRa_com_err = 0x10
    Carrier_hook_err = 0x20
    Battery_overvoltage = 0x40
    Battery_overcurrent = 0x80

# Select Mode
class SelectModeData:
    NOT_SET = 0x00
    LF_MODE = 0x01
    LIDAR_MODE = 0x02

# Select
class SelectStateData:
    START = 0x01
    STOP = 0x02
    PAUSE = 0x03

class SelectDirData:
    FORWARD = 0x01
    BACKWARD = 0x02
    LEFT = 0x03
    RIGHT = 0x04
    ROTATE_LEFT = 0x05
    ROTATE_RIGHT = 0x06
    BRAKE = 0x07

class SensorData:
    DETECTED = 0x01
    NOT_DETECTED = 0x02

class AccelData:
    NORMAL_ACCEL = 0x01
    REGENERATIVE_ACCEL = 0x02

class BrakeData:
    NORMAL_BRAKE = 0x01
    REGENERATIVE_BRAKE = 0x02

class PositionData:
    HOME = 0x01
    ON_STATION = 0x02
    ON_THE_WAY = 0x03

class StationData:
    HOME_STATION = 0x01
    STATION_A = 0x02
    STATION_B = 0x03

class NFCSignData:
    NONE_SIGN = 0x00
    HOME_SIGN = 0x01
    STATION_SIGN = 0x02
    TURN_SIGN = 0x03
    CROSSECTION_SIGN = 0x04
    CARRIER_SIGN = 0x05

class Serial_COM:
    def __init__(self, inputBaudRate, serialPort, inputTimeout):
          self.baud = inputBaudRate
          self.port = serialPort
          self.timeout = inputTimeout

          self.Header = 0xFF
          self.Tail = 0xA5

          # Data Send
          self.Running_Mode = 0
          self.Base_Speed = 0
          self.Running_State = 0
          self.Running_Dir = 0
          self.Running_Accel = 0
          self.Running_Brake = 0
          self.Start_coordinateX = 0
          self.Start_coordinateY = 0
          self.Goal_coordinateX = 0
          self.Goal_coordinateY = 0
          self.To_Tagnum = 0
          self.joystick_X = 0
          self.joystick_Y = 0
          self.Left_speed = 0
          self.Right_speed = 0

          # Data Get
          self.Current_Pos = 0
          self.CurrentPos_value = 0
          self.Left_Counter = 0
          self.Right_Counter = 0
          self.SensorA_Status = 0
          self.SensorB_Status = 0
          self.Tag_sign = 0
          self.Tag_value = 0
          self.Tag_num = 0
          self.Current_coordinateX = 0
          self.CUrrent_coordinateY = 0
          self.Send_Counter = 0
          self.Pickup_Counter = 0
          self.Battery_Level = 0

          self.running_mode = SelectModeData()
          self.running_state = SelectStateData()
          self.running_dir = SelectDirData()
          self.accel_mode = AccelData()
          self.brake_mode = BrakeData()
          self.station_type = StationData()
          self.sign_data = NFCSignData()

          # Serial Initialize
          self.data = serial.Serial(serialPort, inputBaudRate, timeout=inputTimeout)
          self.data.setDTR(False)
          time.sleep(1)
          self.data.flushInput()
          self.data.setDTR(True)
          time.sleep(1)

    def Transmit_Data(self):
        tx_buff = arr.array('B', [
            self.Header,
            self.Header,
            
            self.Running_Mode,
            self.Base_Speed, 
            self.Running_State,
            self.Running_Dir,
            self.Running_Accel,
            self.Running_Brake,

            (self.Start_coordinateX >> 8) & 0xFF,
            self.Start_coordinateX & 0xFF,
            (self.Start_coordinateY >> 8) & 0xFF,
            self.Start_coordinateY & 0xFF,

            (self.Goal_coordinateX >> 8) & 0xFF,
            self.Goal_coordinateX & 0xFF,
            (self.Goal_coordinateY >> 8) & 0xFF,
            self.Goal_coordinateY & 0xFF,

            (self.joystick_X >> 8) & 0xFF,
            self.joystick_X & 0xFF,
            (self.joystick_Y >> 8) & 0xFF,
            self.joystick_Y & 0xFF,

            self.Left_speed,
            self.Right_speed,

            self.Tail
            ])
        self.data.write(tx_buff)

    def Receive_Data(self):
        rx_buff = self.data.read(32)

        if len(rx_buff) == 32 and rx_buff[0] == self.Header and rx_buff[1] == self.Header and rx_buff[31] == self.Tail:
            self.Running_Mode = rx_buff[2]
            self.Running_State = rx_buff[3]
            self.Current_Pos = rx_buff[4]
            self.CurrentPos_value = rx_buff[5] << 8 | rx_buff[6]
            self.Tag_sign = rx_buff[7]
            self.Tag_value = rx_buff[8] << 8 | rx_buff[9]
            self.Tag_num = rx_buff[10] << 8 | rx_buff[11]
            self.Current_coordinateX = rx_buff[12] << 8 | rx_buff[13]
            self.Current_coordinateY = rx_buff[14] << 8 | rx_buff[15]
            self.Left_Counter = rx_buff[16] << 8 | rx_buff[17]
            self.Right_Counter = rx_buff[18] << 8 | rx_buff[19]
            self.SensorA_Status = rx_buff[20]
            self.SensorB_Status = rx_buff[21]
            self.Send_Counter = rx_buff[22] << 8 | rx_buff[23]
            self.Pickup_Counter = rx_buff[24] << 8 | rx_buff[25]

            voltage_array = arr.array('B', [rx_buff[26], rx_buff[27], rx_buff[28], rx_buff[29]])
            voltage_data = bytes(voltage_array)
            self.Battery_Level = struct.unpack('f', voltage_data)[0]

UART_COM = Serial_COM(baudRate, ArduinoPort, timeout)