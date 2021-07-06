import serial

motor = serial.Serial(port='/dev/ttyUSB0', baudrate=9600, timeout=1)
db = serial.Serial(port='/dev/ttyUSB1', baudrate=9600, timeout=1)

while(True):
    arah = input("Arah = ")

    if arah == 'w':
        motor.write(b"MAJU\n")
    elif arah == 'a':
        motor.write(b"GESER KIRI\n")
    elif arah == 's':
        motor.write(b"MUNDUR\n")
    elif arah == 'd':
        motor.write(b"GESER KANAN\n")
    elif arah == 'q':
        motor.write(b"BELOK KIRI\n")
    elif arah == 'e':
        motor.write(b"BELOK KANAN\n")
    elif arah == 'z':
        motor.write(b"PUTAR KIRI\n")
    elif arah == 'x':
        motor.write(b"PUTAR KIRI\n")
    elif arah == 'r':
        motor.write(b"SERONG KIRI\n")
    elif arah == 't':
        motor.write(b"SERONG KANAN\n")
    elif arah == '1':
        motor.write(b"BERHENTI\n")
    elif arah == '0':
        db.write(b"DB OFF\n")
    elif arah == '9':
        db.write(b"DB ON\n")
