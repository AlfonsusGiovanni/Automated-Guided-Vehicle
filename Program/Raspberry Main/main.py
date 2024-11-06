# Raspberry Main Program
# PT Stechoq Robotika Indonesia
# Date	 : 18 Oktober 2024
# Author : Alfonsus Giovanni Mahendra Putra - Universitas Diponegoro

import CustomSerial as Serial
import threading
import json
import time

class AGV_State:
    AGV_READY = 0x01
    AGV_BUSY = 0x02
    AGV_WAITING = 0x03
    AGV_COMPLETE = 0x04

class Scheduller_Handler:
    def __init__(self):
        self.payload_data = 0
        self.payload_run_state = 0

    def receive_payload(self):
        if Serial.UART_COM.Running_Mode == Serial.UART_COM.running_mode.LF_MODE:
            data = json.loads(self.payload_data)
            AGV.destination_id = data['destination']['rfid_code']
        elif Serial.UART_COM.Select_mode == Serial.UART_COM.running_mode.LIDAR_MODE:
            data = json.loads(self.payload_data)
            AGV.Xpos_dest = data['destination']['lidar_position']['x']
            AGV.Ypos_dest = data['destination']['lidar_position']['y']
            AGV.Zpos_dest = data['destination']['lidar_position']['z']

    def send_payload(self, payload_type, payload_data):
        pass

class AGV:
    # AGV Class Variable
    agv_status = AGV_State.AGV_READY
    Xpos_dest = 0
    Ypos_dest = 0
    Zpos_dest = 0
    destination_id = 0

    def __init__(self):
        self.run_state = 0
        self.run_dir = 0

        # Thread Function Control
        self.SerialTransmit_Thread_On = False
        self.SerialReceive_Thread_On = False
        self.dataTransmitted = False

    # AGV System Setting
    def System_Set(self, runmode, basespeed, accel, brake):
        Serial.UART_COM.Running_Mode = runmode
        Serial.UART_COM.Base_Speed =basespeed
        Serial.UART_COM.Running_Accel = accel
        Serial.UART_COM.Running_Brake = brake

    def Transmitter_Handler(self):
        while self.SerialTransmit_Thread_On:
            # Serial.UART_COM.Send_String()
            Serial.UART_COM.Transmit_Data()
            Serial.UART_COM.data.flushOutput()
            time.sleep(0.1)

    def Receiver_Handler(self):
        while self.SerialReceive_Thread_On:
            if Serial.UART_COM.data.in_waiting > 0:
                # Serial.UART_COM.Receive_String()
                Serial.UART_COM.Receive_Data()
                Serial.UART_COM.data.flushInput()
                time.sleep(1)

    def Set_Destination(self, now_position, destination):
        if Serial.UART_COM.Current_Pos != destination:
            Serial.UART_COM.Start_Pos = now_position
            Serial.UART_COM.Destination = destination
    
# Class Declare
AGV1 = AGV()
Scheduller = Scheduller_Handler()

# Thread Function Declare
Transmitter_Thread = threading.Thread(target=AGV1.Transmitter_Handler)
Receiver_Thread = threading.Thread(target=AGV1.Receiver_Handler)

# Initialize Communication
AGV1.System_Set(Serial.UART_COM.running_mode.LF_MODE, 80, Serial.UART_COM.accel_mode.REGENERATIVE_ACCEL, Serial.UART_COM.brake_mode.REGENERATIVE_BRAKE)
for i in range(0, 100):
    Serial.UART_COM.Start_Pos = Serial.UART_COM.station_type.HOME_STATION
    Serial.UART_COM.Destination = Serial.UART_COM.station_type.STATION_A
    Serial.UART_COM.Running_State = Serial.UART_COM.running_state.START
    Serial.UART_COM.Transmit_Data()
print("Init Done")

time.sleep(1)

# Start All Thread
AGV1.SerialTransmit_Thread_On = True
AGV1.SerialReceive_Thread_On = True

Transmitter_Thread.start()
Receiver_Thread.start()

while True:
    print("Run Start")
    Serial.UART_COM.Running_Mode = Serial.UART_COM.running_mode.LF_MODE
    Serial.UART_COM.Running_Dir = Serial.UART_COM.running_dir.FORWARD
    # print([hex(i) for i in Serial.UART_COM.return_data])

    print("AGV Running Mode: ", Serial.UART_COM.Running_Mode)
    print("AGV Running State: ", Serial.UART_COM.Running_State)
    print("AGV Position: ", Serial.UART_COM.Current_Pos)
    print("AGV Pos Value: ", Serial.UART_COM.CurrentPos_value)
    print("AGV Send Counter: ", Serial.UART_COM.Send_Counter)
    print("AGV Pickup Counter: ", Serial.UART_COM.Pickup_Counter)
    print("AGV Battery Level: ", Serial.UART_COM.Battery_Level)

    # print("Running State: ", Serial.UART_COM.Select_state)

    # print("Sign Type: ", Serial.UART_COM.Tag_sign)
    # print("Sign Value: ", Serial.UART_COM.Tag_value)
    # print("Sign Num: ", Serial.UART_COM.Tag_num)

    time.sleep(1)