import serial
import keyboard

motor = serial.Serial(port='/dev/ttyUSB0', baudrate=9600, timeout=1)
db = serial.Serial(port='/dev/ttyUSB1', baudrate=9600, timeout=1)

while(True):
    if keyboard.is_pressed('w'):
        motor.write(b"MAJU\n")
        print("MAJU")
    elif keyboard.press('a'):
        motor.write(b"GESER KIRI\n")
    elif keyboard.press('s'):
        motor.write(b"MUNDUR\n")
    elif keyboard.press('d'):
        motor.write(b"GESER KANAN")
    elif keyboard.press('q'):
        motor.write(b"BELOK KIRI\n")
    elif keyboard.press('e'):
        motor.write(b"BELOK KANAN\n")
    elif keyboard.press('z'):
        motor.write(b"PUTAR KIRI\n")
    elif keyboard.press('x'):
        motor.write(b"PUTAR KIRI\n")
    elif keyboard.press('r'):
        motor.write(b"SERONG KIRI\n")
    elif keyboard.press('t'):
        motor.write(b"SERONG KANAN\n")
    elif keyboard.press('1'):
        motor.write(b"BERHENTI\n")
    elif keyboard.press('0'):
        db.write(b"DB OFF\n")
    elif keyboard.press('9'):
        db.write(b"DB ON\n")
