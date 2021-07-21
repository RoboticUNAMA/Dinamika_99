import cv2
import numpy as np
import serial
from time import sleep
from detection import Camera
from controller import *

# serial
motor = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
db = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)

# index camera
FRONT_CAM = 1
OMNI_CAM = 0

def nothing(x):
    pass

def main():
    cam1 = Camera(FRONT_CAM, 'ballColor.txt')
    cam2 = Camera(OMNI_CAM, 'ballColor1.txt')
    motor = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    db = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
    motor.close()
    db.close()
    sleep(1)

    # initial variables
    delay = 3
    count = 0
    wait = 0
    mode = "-"
    state = "IDLE"
    nav = "BERHENTI"

    #geser(motor, -100)
    #time.sleep(1.7)
    #maju(motor, 100)
    #time.sleep(2.2)

    # create ui
    main_window = "Dinamika_99 Main Program"
    cv2.namedWindow(main_window)

    control_window = "Control Panel"
    cv2.namedWindow(control_window)
    cv2.createTrackbar('State', control_window, 0, 2, nothing)
    cv2.createTrackbar('Engine', control_window, 1, 2, nothing)
    cv2.createTrackbar('Mode', control_window, 0, 2, nothing)
    cv2.createTrackbar('Opsi', control_window, 0, 9, nothing)
    cv2.createTrackbar('Control Mode', control_window, 0, 1, nothing)

    while True:
        # get frames
        area1, x1, y1, w1, h1, cenX1, cenY1 = cam1.get_object(500)
        area2, x2, y2, w2, h2, cenX2, cenY2 = cam2.get_object(10)
        frame1 = cam1.display("Front Cam", "Bola", 0)
        frame2 = cam2.display("Omni Cam", "Bola", 0)
        frmPanel = cv2.imread('bg.png')

        # get trackbar value
        engineVal = cv2.getTrackbarPos('Engine', control_window)
        modeVal = cv2.getTrackbarPos('Mode', control_window)
        stateVal = cv2.getTrackbarPos('State', control_window)
        opsiVal = cv2.getTrackbarPos('Opsi', control_window)
        controlVal = cv2.getTrackbarPos('Control Mode', control_window)

        # display trackbar value to panel frame
        # TEXT STATE
        if stateVal == 0:
            state = "Idle"
        elif stateVal == 1:
            state = "Cari Bola"
        elif stateVal == 2:
            state = "Tunggu Bola"
        cv2.putText(
                frmPanel, "State: "+state, (5,20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, [0,255,0], 2) 

        # TEXT ENGINE
        if engineVal == 2:
            engine = "START"
        elif engineVal == 0:
            engine = "RETRY"
        else:
            engine = "STOP"
        cv2.putText(
                frmPanel, "Engine: "+engine, (5,50),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, [0,255,0], 2) 

        # TEXT MODE
        if modeVal == 1:
            mode = "Kickoff Kiri"
        elif modeVal == 2:
            mode = "Kickoff Corner"
        else:
            mode = "Kickoff Kanan"
        cv2.putText(
                frmPanel, "Mode: "+mode, (5,80),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, [0,255,0], 2)  

        # TEXT OPSI
        cv2.putText(
                frmPanel, "Opsi: "+str(opsiVal), (5,110),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, [0,255,0], 2) 

        # TEXT CONTROL
        if controlVal == 1:
            control = "True"
        else:
            control = "False"
        cv2.putText(
                frmPanel, "Control: "+control, (5,140),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, [0,255,0], 2) 

        # show windows
        window = np.concatenate((frame1, frame2), axis=1)
        cv2.imshow(main_window, window)
        cv2.imshow(control_window, frmPanel)
        #cv2.moveWindow(window_name, 200,100 )

        # conditioning
        if opsiVal == 0 and state == "START":
            state = "CARI BOLA"

        elif opsiVal == 1 and state == "START":
            #geser(motor, -80)
            #sleep(delay)
            #maju(motor, 80)
            #sleep(delay)
            state = "CARI BOLA"

        elif opsiVal == 2 and state == "START":
            #serongKiri(motor, 120)
            #sleep(delay)
            #maju(motor, 80)
            #sleep(delay)
            state = "CARI BOLA"

        elif state == "CARI BOLA":
            #db_on(db)
            print(state)

        # keyboard control
        k = cv2.waitKey(1)
        if k == 27:
            cv2.destroyAllWindows
            break

        if controlVal == 0:
            if k == ord('1'):
                cv2.setTrackbarPos("State", control_window, 0)
            elif k == ord('2'):
                cv2.setTrackbarPos("State", control_window, 1)
            elif k == ord('3'):
                cv2.setTrackbarPos("State", control_window, 2)
            elif k == ord('q') or k == ord('Q'):
                cv2.setTrackbarPos("Engine", control_window, 0)
            elif k == ord('w') or k == ord('W'):
                cv2.setTrackbarPos("Engine", control_window, 1)
            elif k == ord('e') or k == ord('E'):
                cv2.setTrackbarPos("Engine", control_window, 2)
            elif k == ord('a') or k == ord('A'):
                cv2.setTrackbarPos("Mode", control_window, 0)
            elif k == ord('s') or k == ord('S'):
                cv2.setTrackbarPos("Mode", control_window, 1)
            elif k == ord('d') or k == ord('D'):
                cv2.setTrackbarPos("Mode", control_window, 2)
            elif k == ord('x') or k == ord('X'):
                cv2.setTrackbarPos("Control Mode", control_window, 1)

        elif controlVal == 1:
            if k == ord('z') or k == ord('Z'):
                cv2.setTrackbarPos("Control Mode", control_window, 0)
            elif k == ord('w') or k == ord('W'):
                maju(motor, 80)               
            elif k == ord('a') or k == ord('A'):
                geser(motor, -80)
            elif k == ord('s') or k == ord('S'):
                mundur(motor, 80)
            elif k == ord('d') or k == ord('D'):
                geser(motor, 80)
            elif k == ord('q') or k == ord('Q'):
                serongKiri(motor, 80)
            elif k == ord('e') or k == ord('E'):
                serongKanan(motor, 80)
            elif k == ord('j') or k == ord('J'):
                putar(motor, 80)
            elif k == ord('l') or k == ord('L'):
                putar(motor, -80)
            elif k == ord('i') or k == ord('I'):
                tendang(db)
            elif k == ord('k') or k == ord('K'):
                passing(db)
            elif k == ord('9'):
                db_on(db)
            elif k == ord('0'):
                db_off(db)
            else:
                berhenti(motor)
                db_off(db)

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
