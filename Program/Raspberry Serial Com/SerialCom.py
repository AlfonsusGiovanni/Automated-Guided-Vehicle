# Raspberry Serial Communication Handler Program
# PT Stechoq Robotika Indonesia
# Date   : 27 September 2024
# Author : Alfonsus Giovanni Mahendra Putra - Universitas Diponegoro

# USED LIBRARY
import serial
import struct
import array as arr
import time
# -----------

# SERIAL SETUP ---------
baudRate = 115200
portCom = '/dev/serial0'
timeout = 0.1
# ----------------------

# COM STATUS ---------------
dataReceived_valid = 0x01
dataReceived_invalid = 0x02
dataNotReceived = 0x03
# --------------------------

# MODE --------
LF_mode = 0x01
LID_mode = 0x02
# -------------

# RUN STATE ----
RUN_START = 0x01
RUN_STOP = 0x02
RUN_PAUSE = 0x03
# --------------


class Serial_COM:

        def __init__(self, inputBaudRate, serialPort, inputTimeout):
                self.baud = inputBaudRate
                self.port = serialPort
                self.timeout = inputTimeout

                self.msgHeader = 0xff
                self.msgTail = 0xa5

                self.sendMode = 0x01
                self.sendRun = 0x02
                self.sendStatus = 0x03

                self.command = None
                self.mode = None
                self.run_state = None
                self.position = None
                self.pos_value = None
                self.send_counter = None
                self.pickup_counter = None
                self.battery_value = None

                self.status = None

                self.data = serial.Serial(serialPort, inputBaudRate, timeout=inputTimeout)

        def Send_Mode(self, setMode):
                tx_buff = bytearray([self.msgHeader, self.sendMode, setMode, 0x00, 0x00, 0x00, 0x00, 0x00,
                                     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, self.msgTail])
                self.data.write(tx_buff)

        def Send_Run(self, setRun):
                tx_buff = bytearray([self.msgHeader, self.sendRun, 0x00, setRun, 0x00, 0x00, 0x00, 0x00,
                                     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, self.msgTail])
                self.data.write(tx_buff)

        def Receive_Data(self, byteSize):
                rx_buff = self.data.read(byteSize)

                if len(rx_buff) == byteSize:
                        if rx_buff[0] == self.msgHeader and rx_buff[1] == self.sendStatus and self.command == 0x03:
                                self.mode = rx_buff[2]
                                self.run_state = rx_buff[3]
                                self.position = rx_buff[4]
                                self.pos_value = (rx_buff[5] << 8) | rx_buff[6]
                                self.send_counter = (rx_buff[7] << 8) | rx_buff[8]
                                self.pickup_counter = (rx_buff[9] << 8) | rx_buff[10]

                                voltageArray = arr.array('B', [rx_buff[11], rx_buff[12], rx_buff[13], rx_buff[14]])
                                voltageData = bytes(voltageArray)

                                self.battery_value = struct.unpack('f', voltageData)[0]

                                self.status = dataReceived_valid
                        else:
                                self.status = dataReceived_invalid
                else :
                        self.status = dataNotReceived

                return rx_buff
                self.data.flushInput()

        def Test_Receive(self, byteSize):
                get_data = UART_COM.Receive_Data(byteSize)

                if UART_COM.status == dataReceived_valid:
                        print("Data Valid")
                        print([hex(i) for i in get_data])

                elif UART_COM.status == dataReceived_invalid:
                        print("Data Invalid")

                else:
                        print("Data Not Received")

        def Test_Transmit(self, iteration):
                print("Sending...")
                for a in range(0, iteration):
                        UART_COM.Send_Run(RUN_START)

                for b in range(0, iteration):
                        UART_COM.Send_Run(RUN_STOP)

UART_COM = Serial_COM(baudRate, portCom, timeout)

while True:
        UART_COM.Receive_Data(16)

        print("Position: ", UART_COM.position)
        print("Value: ", UART_COM.pos_value)
        print("Send: ", UART_COM.send_counter)
        print("Pick Up: ", UART_COM.pickup_counter)
        print("Battery: ", UART_COM.battery_value)

#       UART_COM.Test_Transmit(100)
#       UART_COM.Test_Receive(16)