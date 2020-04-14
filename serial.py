import serial
ser = serial.Serial('/dev/ttyUSB0', 9600)

while True:
    data = ser.readline()
    if data:
        print(data)
