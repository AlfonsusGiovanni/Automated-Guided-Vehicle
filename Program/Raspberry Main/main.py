# Raspberry Main Program
# PT Stechoq Robotika Indonesia
# Date	 : 18 Oktober 2024
# Author : Alfonsus Giovanni Mahendra Putra - Universitas Diponegoro

import SerialCom as Serial
import json

class Scheduller_Handler:
    def receive_payload(self, payload_data):
        if Serial.UART_COM.Select_mode == Serial.UART_COM.running_mode.LF_MODE:
            data = json.loads(payload_data)
            AGV.destination = data['destination']['rfid_code']
        elif Serial.UART_COM.Select_mode == Serial.UART_COM.running_mode.LIDAR_MODE:
            data = json.loads(payload_data)
            AGV.Xpos_dest = data['destination']['lidar_position']['x']
            AGV.Ypos_dest = data['destination']['lidar_position']['y']
            AGV.Zpos_dest = data['destination']['lidar_position']['z']

    def send_payload(self, payload_type, payload_data):
        pass

class AGV:
    agv_status = None

    Xpos_dest = None
    Ypos_dest = None
    Zpos_dest = None

    destination = None
    current_position = None

    def System_Set(self, runmode, basespeed, accel, brake):
        Serial.UART_COM.Initialize(runmode, basespeed, accel, brake)
            
    def Run(self, state):
        pass

    def Sensor_Check(self, select_sensor):
        pass

    def RFID_Check(self):
        pass

    def Return_Home(self):
        pass

AGV1 = AGV()