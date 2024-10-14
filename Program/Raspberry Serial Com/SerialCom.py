# Raspberry Serial Communication Handler Program
# PT Stechoq Robotika Indonesia
# Date	 : 11 Oktober 2024
# Author : Alfonsus Giovanni Mahendra Putra - Universitas Diponegoro


# USED LIBRARY -----
import serial
import struct
import array as arr
import time
import copy
# ------------------

# SERIAL SETUP ---------
baudRate = 500000
MaxMsgsLen = 16
UARTPort = '/dev/serial0'
USBPort = '/dev/ttyACM1'
timeout = 1
# ----------------------

# Instruction 
class Instruction:
    Ping = 0x01
    Read = 0x02
    Write = 0x03

class Item:
    Running_Mode = 0x01
    Running_State = 0x02
    AGV_Status = 0x03
    Sensor_Data = 0x04
    NFC_Data = 0x05
    Joystick_Data = 0x06

class SubItem:
    Sub_item1 = 0x01
    Sub_item2 = 0x02
    Sub_item3 = 0x03

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

class SelectModeData:
    NOT_SET = 0x00
    LF_MODE = 0x01
    LIDAR_MODE = 0x02

class SelectSpeedData:
    NORMAL_SPEED = 0x01
    HIGH_SPEED = 0x02

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


class Serial_COM:
    def __init__(self, inputBaudRate, serialPort, inputTimeout):
          self.baud = inputBaudRate
          self.port = serialPort
          self.timeout = inputTimeout
          
          self.Header = 0xFF

          self.return_data = bytearray(16)
          self.data_length = 0
          self.Instruction_get = 0
          self.Item_get = 0
          self.SubItem_get = 0
          
          self.Select_mode = SelectModeData.NOT_SET
          self.Base_speed = 0
          
          self.Select_state = SelectStateData.STOP
          self.Set_Direction = SelectDirData.FORWARD
          self.Set_Acceleration = AccelData.NORMAL_ACCEL
          self.Set_Breaking = BrakeData.NORMAL_BRAKE
          
          self.Position = PositionData.HOME
          self.Pos_value = 0
          self.Send_counter = 0
          self.Pickup_counter = 0
          self.Tag_position = PositionData.HOME
          self.Tag_value = 0
          
          self.SensorA = SensorData.NOT_DETECTED
          self.SensorB = SensorData.NOT_DETECTED
          
          self.Battery_level = 0.0
          
          self.Xpos = 0
          self.Ypos = 0
          
          self.instruction = Instruction()
          self.error_state = Error()
          self.item = Item()
          self.sub_item = SubItem()

          self.running_mode = SelectModeData()
          self.accel_mode = AccelData()
          self.brake_mode = BrakeData()
          
          self.data = serial.Serial(serialPort, inputBaudRate, timeout=inputTimeout)
          
    def Send_Ping(self):
        msgs_len = 0x01
        tx_buff = [self.Header, self.Header, msgs_len, self.instruction.Ping]
        self.data.write(tx_buff)

        rx_buff = self.data.read(5)
        if len(rx_buff) == 5:
            if rx_buff[0] == self.Header and rx_buff[1] == self.Header:
                self.data_length = rx_buff[2]
            self.return_data = copy.deepcopy(rx_buff)
            self.data.flushInput()
    
    def Set_Running_Mode(self, select_mode):
        msgs_len = 0x04
        tx_buff = [self.Header, self.Header, msgs_len, self.instruction.Write, self.item.Running_Mode, self.sub_item.Sub_item1, select_mode]
        self.data.write(tx_buff)

        rx_buff = self.data.read(4)
        if len(rx_buff) == 4:
            if rx_buff[0] == self.Header and rx_buff[1] == self.Header:
                self.data_length = rx_buff[2]
            self.return_data = copy.deepcopy(rx_buff)
            self.data.flushInput()
    
    def Set_Base_Speed(self, base_speed):
        msgs_len = 0x04
        tx_buff = [self.Header, self.Header, msgs_len, self.instruction.Write, self.item.Running_Mode, self.sub_item.Sub_item2, base_speed]
        self.data.write(tx_buff)

        rx_buff = self.data.read(4)
        if len(rx_buff) == 4:
            if rx_buff[0] == self.Header and rx_buff[1] == self.Header:
                self.data_length = rx_buff[2]
            self.return_data = copy.deepcopy(rx_buff)
            self.data.flushInput()

    def Set_Running_State(self, select_state, direction):
        msgs_len = 0x05
        tx_buff = [self.Header, self.Header, msgs_len, self.instruction.Write, self.item.Running_State, self.sub_item.Sub_item1, select_state, direction]
        self.data.write(tx_buff)

        rx_buff = self.data.read(4)
        if len(rx_buff) == 4:
            if rx_buff[0] == self.Header and rx_buff[1] == self.Header:
                self.data_length = rx_buff[2]
            self.return_data = copy.deepcopy(rx_buff)
            self.data.flushInput()
    
    def Set_Running_Accel(self, set_accel_mode):
        msgs_len = 0x04
        tx_buff = [self.Header, self.Header, msgs_len, self.instruction.Write, self.item.Running_State, self.sub_item.Sub_item2, set_accel_mode]
        self.data.write(tx_buff)

        rx_buff = self.data.read(4)
        if len(rx_buff) == 4:
            if rx_buff[0] == self.Header and rx_buff[1] == self.Header:
                self.data_length = rx_buff[2]
            self.return_data = copy.deepcopy(rx_buff)
            self.data.flushInput()
    
    def Set_Running_Brake(self, set_brake_mode):
        msgs_len = 0x04
        tx_buff = [self.Header, self.Header, msgs_len, self.instruction.Write, self.item.Running_State, self.sub_item.Sub_item3, set_brake_mode]
        self.data.write(tx_buff)

        rx_buff = self.data.read(4)
        if len(rx_buff) == 4:
            if rx_buff[0] == self.Header and rx_buff[1] == self.Header:
                self.data_length = rx_buff[2]
            self.return_data = copy.deepcopy(rx_buff)
            self.data.flushInput()
    
    def Set_Joystick(self, valueX, valueY):
        msgs_len = 0x04
        tx_buff = [self.Header, self.Header, msgs_len, self.instruction.Write, self.item.Running_State, valueX, valueY]
        self.data.write(tx_buff)

        rx_buff = self.data.read(4)
        if len(rx_buff) == 4:
            if rx_buff[0] == self.Header and rx_buff[1] == self.Header:
                self.data_length = rx_buff[2]
                self.return_data = copy.deepcopy(rx_buff)
                self.data.flushInput()
    
    def Read_Running_Mode(self):
        msgs_len = 0x02
        tx_buff = [self.Header, self.Header, msgs_len, self.instruction.Read, self.item.Running_Mode]
        self.data.write(tx_buff)

        rx_buff = self.data.read(6)
        if len(rx_buff) == 6:
            if rx_buff[0] == self.Header and rx_buff[1] == self.Header and rx_buff[3] == 0:
                self.data_length = rx_buff[2]

                if(rx_buff[4] == 0x00):
                    self.Select_mode = SelectModeData.NOT_SET
                elif(rx_buff[4] == 0x01):
                    self.Select_mode = SelectModeData.LF_MODE
                else:
                    self.Select_mode = SelectModeData.LIDAR_MODE
                self.Base_speed = rx_buff[5]

                self.return_data = copy.deepcopy(rx_buff)

            self.return_data = copy.deepcopy(rx_buff)
            self.data.flushInput()

    def Read_Running_State(self):
        msgs_len = 0x02
        tx_buff = [self.Header, self.Header, msgs_len, self.instruction.Read, self.item.Running_State]
        self.data.write(tx_buff)

        rx_buff = self.data.read(8)
        if len(rx_buff) == 8:
            if rx_buff[0] == self.Header and rx_buff[1] == self.Header and rx_buff[3] == 0:
                self.data_length = rx_buff[2]

                if(rx_buff[4] == 0x01):
                    self.Select_state = SelectStateData.START
                elif(rx_buff[4] == 0x02):
                    self.Select_state = SelectStateData.STOP
                else:
                    self.Select_state = SelectStateData.PAUSE

                if(rx_buff[5] == 0x01):
                    self.Set_Direction = SelectDirData.FORWARD
                elif(rx_buff[5] == 0x02):
                    self.Set_Direction = SelectDirData.BACKWARD
                elif(rx_buff[5] == 0x03):
                    self.Set_Direction = SelectDirData.LEFT
                elif(rx_buff[5] == 0x04):
                    self.Set_Direction = SelectDirData.RIGHT
                elif(rx_buff[5] == 0x05):
                    self.Set_Direction = SelectDirData.ROTATE_LEFT
                elif(rx_buff[5] == 0x06):
                    self.Set_Direction = SelectDirData.ROTATE_RIGHT
                elif(rx_buff[5] == 0x07):
                    self.Set_Direction = SelectDirData.BRAKE

                if(rx_buff[6] == 0x01):
                    self.Set_Acceleration = AccelData.NORMAL_ACCEL
                else:
                    self.Set_Acceleration = AccelData.REGENERATIVE_ACCEL

                if(rx_buff[7] == 0x01):
                    self.Set_Breaking = BrakeData.NORMAL_BRAKE
                else:
                    self.Set_Breaking = BrakeData.REGENERATIVE_BRAKE

                self.return_data = copy.deepcopy(8)

            self.return_data = copy.deepcopy(rx_buff)
            self.data.flushInput()
    
    def Read_AGV_Status(self):
        msgs_len = 0x02
        tx_buff = [self.Header, self.Header, msgs_len, self.instruction.Read, self.item.AGV_Status]
        self.data.write(tx_buff)

        rx_buff = self.data.read(15)
        if len(rx_buff) == 15:
            if rx_buff[0] == self.Header and rx_buff[1] == self.Header and rx_buff[3] == 0:
                self.data_length = rx_buff[2]

                if(rx_buff[4] == 0x01):
                    self.Position = PositionData.HOME
                elif(rx_buff[4] == 0x02):
                    self.Position = PositionData.ON_STATION
                else:
                    self.Position = PositionData.ON_THE_WAY

                self.Pos_value = (rx_buff[5] << 8) | rx_buff[6]
                self.Send_counter = (rx_buff[7] << 8) | rx_buff[8]
                self.Pickup_counter = (rx_buff[9] << 8) | rx_buff[10]

                voltage_array = arr.array('B', [rx_buff[11], rx_buff[12], rx_buff[13], rx_buff[14]])
                voltage_data = bytes(voltage_array)
                self.Battery_level = struct.unpack('f', voltage_data)[0]

                self.return_data = copy.deepcopy(rx_buff)

            self.return_data = copy.deepcopy(rx_buff)
            self.data.flushInput()
    
    def Read_Sensor_Data(self):
        msgs_len = 0x02
        tx_buff = [self.Header, self.Header, msgs_len, self.instruction.Read, self.item.Sensor_Data]
        self.data.write(tx_buff)

        rx_buff = self.data.read(6)
        if len(rx_buff) == 6:
            if(rx_buff[0] == self.Header and rx_buff[1] == self.Header and rx_buff[3] == 0):
                self.data_length = rx_buff[2]
                if(rx_buff[4] == 0x01):
                    self.SensorA = SensorData.DETECTED
                else:
                    self.SensorA = SensorData.NOT_DETECTED
                if(rx_buff[5] == 0x01):
                    self.SensorB = SensorData.DETECTED
                else:
                    self.SensorB = SensorData.NOT_DETECTED
                self.return_data = copy.deepcopy(rx_buff)

            self.return_data = copy.deepcopy(rx_buff)
            self.data.flushInput()
    
    def Read_NFC_Data(self):
        msgs_len = 0x02
        tx_buff = [self.Header, self.Header, msgs_len, self.instruction.Read, self.item.NFC_Data]
        self.data.write(tx_buff)

        rx_buff = self.data.read(7)
        if len(rx_buff) == 7:
            if(rx_buff[0] == self.Header and rx_buff[1] == self.Header and rx_buff[3] == 0):
                self.data_length = rx_buff[2]
                if(rx_buff[4] == 0x01):
                    self.Tag_position = PositionData.HOME
                elif(rx_buff[4] == 0x02):
                    self.Tag_position = PositionData.ON_STATION
                else:
                    self.Tag_position = PositionData.ON_THE_WAY
                self.Tag_value = (rx_buff[5] << 8) | rx_buff[6]
                self.return_data = copy.deepcopy(rx_buff)

            self.return_data = copy.deepcopy(rx_buff)
            self.data.flushInput()
    
    def Read_Joystick_Data(self):
        msgs_len = 0x02
        tx_buff = [self.Header, self.Header, msgs_len, self.instruction.Read, self.item.Joystick_Data]
        self.data.write(tx_buff)

        rx_buff = self.data.read(6)
        if len(rx_buff) == 6:
            if(rx_buff[0] == self.Header and rx_buff[1] == self.Header and rx_buff[3] == 0):
                self.data_length = rx_buff[2]
                self.Xpos = rx_buff[4]
                self.Ypos = rx_buff[5]

            self.return_data = copy.deepcopy(rx_buff)
            self.data.flushInput()
            
UART_COM = Serial_COM(baudRate, USBPort, timeout)

# SETUP RUNNING
ping_checked = False
set_mode_complete = False
set_speed_complete = False
set_accel_complete = False
set_brake_complete = False
setup_done = False

while True:
    if not ping_checked:
        UART_COM.Send_Ping()
        if UART_COM.return_data[0] == UART_COM.Header and UART_COM.return_data[1] == UART_COM.Header and UART_COM.return_data[3] == 0:
            print(" ".join(f"0x{byte:02X}" for byte in UART_COM.return_data))
            ping_checked = True
            print("Ping Check Complete")
            time.sleep(1)
            break
        else:
            print("Waiting Ping Response...")
            ping_checked = False

while True:
    if not set_mode_complete:
        UART_COM.Set_Running_Mode(UART_COM.running_mode.LF_MODE)
        if UART_COM.return_data[0] == UART_COM.Header and UART_COM.return_data[1] == UART_COM.Header and UART_COM.return_data[3] == 0:
            print(" ".join(f"0x{byte:02X}" for byte in UART_COM.return_data))
            set_mode_complete = True
            print("Set Mode Complete")
            time.sleep(1)
            break
        else:
            print("Waiting Running Mode Response...")
            set_mode_complete = False

while True:
    if not set_speed_complete:
        UART_COM.Set_Base_Speed(100)
        if UART_COM.return_data[0] == UART_COM.Header and UART_COM.return_data[1] == UART_COM.Header and UART_COM.return_data[3] == 0:
            print(" ".join(f"0x{byte:02X}" for byte in UART_COM.return_data))
            set_speed_complete = True
            print("Set Speed Complete")
            time.sleep(1)
            break
        else:
            print("Waiting Running Speed Response...")
            set_speed_complete = False

while True:
    if not set_accel_complete:
        UART_COM.Set_Running_Accel(UART_COM.accel_mode.REGENERATIVE_ACCEL)
        if UART_COM.return_data[0] == UART_COM.Header and UART_COM.return_data[1] == UART_COM.Header and UART_COM.return_data[3] == 0:
            print(" ".join(f"0x{byte:02X}" for byte in UART_COM.return_data))
            set_accel_complete = True
            print("Set Accel Complete")
            time.sleep(1)
            break
        else:
            print("Waiting Accel Mode Response...")
            set_accel_complete = False

while True:
    if not set_brake_complete:
        UART_COM.Set_Running_Brake(UART_COM.brake_mode.REGENERATIVE_BRAKE)
        if UART_COM.return_data[0] == UART_COM.Header and UART_COM.return_data[1] == UART_COM.Header and UART_COM.return_data[3] == 0:
            print(" ".join(f"0x{byte:02X}" for byte in UART_COM.return_data))
            set_brake_complete = True
            print("Set Brake Complete")
            time.sleep(1)
            break
        else:
            print("Waiting Brake Mode Response...")
            set_brake_complete = False

while True:
    if ping_checked and set_mode_complete and set_speed_complete and set_brake_complete:
        setup_done = True
        print("All Setup Complete")
        time.sleep(1)
        break

    if setup_done:
        break

# TESTING
while True:
    print("Running")