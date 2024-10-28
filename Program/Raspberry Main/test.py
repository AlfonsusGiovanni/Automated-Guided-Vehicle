import serial

ser = serial.Serial('COM17', baudrate=9600, timeout=1)
if not ser.is_open:
    ser.open()
print("Open Success")