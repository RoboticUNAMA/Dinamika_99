import serial

#motor = serial.Serial(port='/dev/ttyUSB1', baudrate=9600, timeout=1)
db = serial.Serial(port='/dev/ttyUSB0', baudrate=9600, timeout=1)

#speed_awal = 100

while(True):
    data = db.readline()
    print(data)