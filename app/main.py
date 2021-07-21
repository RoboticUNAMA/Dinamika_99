import cv2
import numpy as np
import serial
from time import sleep
from detection import Camera
from controller import *
motor = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
db = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)

def nothing(x):
    pass

def start(a=None,b=None):
    state = "START"
    print(state)

def retry(a=None,b=None):
    state = "RETRY"
    print(state)

def koff_kanan(a=None,b=None):
    state = "KICKOFF KANAN"
    print(state)

def koff_kiri(a=None,b=None):
    state = "KICKOFF KIRI"
    print(state)

def koff_corner(a=None,b=None):
    state = "KICKOFF CORNER"
    print(state)

def main():
    cam1 = Camera(3, 'ballColor.txt')
    cam2 = Camera(0, 'ballColor1.txt')
    motor = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    db = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
    motor.close()
    db.close()
    time.sleep(1)

    # initial variables
    delay = 3
    count = 0
    wait = 0
    state = "IDLE"
    nav = "BERHENTI"

    #geser(motor, -100)
    #time.sleep(1.7)
    #maju(motor, 100)
    #time.sleep(2.2)

    # create ui
    window_name = "Dinamika_99 Main Program"
    cv2.namedWindow(window_name)
    cv2.createTrackbar('Opsi', window_name, 0, 50, nothing)
    cv2.createButton("Start", start, None, cv2.QT_PUSH_BUTTON, 0)
    cv2.createButton("Retry", retry, None, cv2.QT_PUSH_BUTTON, 0)
    cv2.createButton("Kickoff Kanan", koff_kanan, None, cv2.QT_RADIOBOX, 0)
    cv2.createButton("Kickoff Kiri", koff_kiri, None, cv2.QT_RADIOBOX, 0)
    cv2.createButton("Kickoff Corner", koff_corner, None, cv2.QT_RADIOBOX, 0)

    while True:
        # get frames
        area1, x1, y1, w1, h1, cenX1, cenY1 = cam1.get_object(500)
        area2, x2, y2, w2, h2, cenX2, cenY2 = cam2.get_object(10)
        frame1 = cam1.display("Front Cam", "Bola", 0)
        frame2 = cam2.display("Omni Cam", "Bola", 0)

        # show user interface
        window = np.concatenate((frame1, frame2), axis=1)
        cv2.imshow(window_name, window)
        cv2.moveWindow(window_name, 200,100 )

        # conditioning
        opsi = cv2.getTrackbarPos('Opsi', window_name)
        #print("opsi", opsi)
        if opsi == 0 and state == "START":
            state = "CARI BOLA"

        elif opsi == 1 and state == "START":
            geser(motor, -80)
            sleep(delay)
            maju(motor, 80)
            sleep(delay)
            state = "CARI BOLA"

        elif opsi == 2 and state == "START":
            serongKiri(motor, 120)
            sleep(delay)
            maju(motor, 80)
            sleep(delay)
            state = "CARI BOLA"

        elif state == "CARI BOLA":
            db_on(db)
            print(state)

        # keyboard control
        k = cv2.waitKey(1)
        if k == 27:
            cv2.destroyAllWindows
            break
        elif k == ord('e') or k == ord('E'):
            start()
        elif k == ord('r') or k == ord('R'):
            retry()
        elif k == ord('w') or k == ord('W'):
            maju(motor, 80)
        elif k == ord('a') or k == ord('A'):
            geser(motor, -80)
        elif k == ord('s') or k == ord('S'):
            mundur(motor, 80)
        elif k == ord('d') or k == ord('D'):
            geser(motor, 80)
        elif k == ord('j') or k == ord('J'):
            putar(motor, 80)
        elif k == ord('l') or k == ord('L'):
            putar(motor, -80)
        else:
            berhenti(motor)
            db_off()

        #if cenX1 > 0 and cenX1 <= 220:
        #    #pid = PID(cenX1, 240)
        #    geser(motor, -40)
        #elif cenX1 >= 260:
        #    #pid = PID(cenX1, 240)
        #    geser(motor, 40)
        #elif cenX1 > 220 and cenX1 < 260:
        #    maju(motor, 80)

        #if cenY2 >= 57 and cenY2 < 65:
        #    db_on(db)
        #   berhenti(motor)

        #if cenX1 == 0:
        #    berhenti(motor)
        #    db_off(db)

        #print(cenY2)

if __name__ == '__main__':
    # execute main program
    main()
