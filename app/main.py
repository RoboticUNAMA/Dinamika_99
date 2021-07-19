import serial
import time
from detection import Camera
from controller import *
motor = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
db = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)

def main():
    green = [0,255,0]
    cam1 = Camera(1, 'ballColor.txt')
    cam2 = Camera(0, 'ballColor1.txt')
    motor = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    db = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
    motor.close()
    db.close()
    time.sleep(1)

    geser(motor, -100)
    time.sleep(1.7)
    maju(motor, 100)
    time.sleep(2.2)

    while True:
        area1, x1, y1, w1, h1, cenX1, cenY1 = cam1.get_object(500)
        area2, x2, y2, w2, h2, cenX2, cenY2 = cam2.get_object(10)
        cam1.display("Front Cam", "Bola", green, 0)
        cam2.display("Omni Cam", "Bola", green, 0)

        if cenX1 > 0 and cenX1 <= 220:
            #pid = PID(cenX1, 240)
            geser(motor, -40)
        elif cenX1 >= 260:
            #pid = PID(cenX1, 240)
            geser(motor, 40)
        elif cenX1 > 220 and cenX1 < 260:
            maju(motor, 80)

        if cenY2 >= 57 and cenY2 < 65:
            db_on(db)
            berhenti(motor)

        if cenX1 == 0:
            berhenti(motor)
            db_off(db)

        print(cenY2)

if __name__ == '__main__':
    # execute main program
    main()
