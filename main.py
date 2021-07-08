'''
    Filename        : main.py
    Description     : Object Detection Robot KRSBI Beroda UNAMA
    Created By      : Arjuna Panji Prakarsa
    Date            : 06/06/2021
    Python Version  : 3.6.9
'''

import cv2
import numpy as np
import datetime
import serial
from time import sleep


# initialize
font = cv2.FONT_HERSHEY_SIMPLEX
date = str(datetime.datetime.now().strftime("%d-%m-%Y %H:%M:%S"))
FRONT_CAM = 1   # front camera
OMNI_CAM = 0    # omni camera
speed_awal = 100

# serial motor driver
motor = serial.Serial(port='/dev/ttyUSB0', baudrate=9600, timeout=1)

# serial dribble
db = serial.Serial(port='/dev/ttyUSB1', baudrate=9600, timeout=1)

def setMotor(dki, dka, bki, bka):
    motor.write(("#M|RUN|" + str(dki) + "|" + str(dka) + "|" + str(bka) + "|"  + str(bki) + "\n").encode('utf-8'))

def getObjectInfo(param):
    if param == 1:
        source = open('ballColor.txt','r')
    elif param == 2:
        source = open('ballColor1.txt', 'r')
    elif param == 3:
        source = open('goalColor.txt', 'r')

    n = []
    for i in source:
        n.append(int(i))

    lower = np.array([n[0],n[1],n[2]])
    upper = np.array([n[3],n[4],n[5]])
    threshold = n[6]
    return lower, upper, threshold

def imageProcessing(frame, lower, upper, threshold):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # blur the frame
    blur = cv2.medianBlur(hsv, 5)

    # create a mask from blurred frame
    mask = cv2.inRange(blur, lower, upper)

    # convert to black and white image
    _, thresh = cv2.threshold(mask, threshold, 255, 0)

    # refine the image using morphological transformation
    kernal = np.ones((5,5), np.uint8)
    morph = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernal, iterations = 2)

    # find contours
    contours, _ = cv2.findContours(morph, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=lambda x:cv2.contourArea(x), reverse=True)
    return contours

def findObject(frame, contours, min_area):
    area, x, y, w, h, cenX, cenY = 0,0,0,0,0,0,0
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > min_area:
            (x, y, w, h) = cv2.boundingRect(contour)
            #cv2.putText(frame, "X: "+str(x)+" Y: "+str(y), (520, 20), font, 0.5, (0,0,255),2)
            cenX = (x+x+w)/2
            cenY = (y+y+h)/2
            break
    return area, x, y, w, h, cenX, cenY

def PID(centerObj, centerFrame):
    Kp = 5.0
    Kd = 5.0
    Ki = 0.0
    error = 0.0
    error_sebelumnya = 0.0
    selisih_error = 0.0
    jumlah_error = 0.0
    error = centerObj - centerFrame
    error_sebelumnya = error
    selisi_error = error_sebelumnya - error
    jumlah_error += (0.001 * error)
    P = Kp * error
    I = Ki * selisih_error
    D = Kd * jumlah_error
    PID = P + I + D
    PID = PID / 5
    return PID

def main():
    # create opencv video capture object
    cap1 = cv2.VideoCapture(FRONT_CAM) 
    cap2= cv2.VideoCapture(OMNI_CAM) 
    # set frame size
    cap1.set(cv2.CAP_PROP_FRAME_WIDTH, 480)
    cap1.set(cv2.CAP_PROP_FRAME_HEIGHT, 270)
    cap2.set(cv2.CAP_PROP_FRAME_WIDTH, 480)
    cap2.set(cv2.CAP_PROP_FRAME_HEIGHT, 270)
    # get center of the frame
    _, frame1 = cap1.read()
    rows1, cols1, _ = frame1.shape
    cenX_frame1 = int(cols1/2)
    cenY_frame1 = int(rows1/2)
    _, frame2 = cap2.read()
    rows2, cols2, _ = frame2.shape
    cenX_frame2 = int(cols2/2)
    cenY_frame2 = int(rows2/2)
    # define center area of the frame 1
    inner_left = cenX_frame1 - 100
    outer_left = cenX_frame1 - 225 
    inner_right = cenX_frame1 + 100
    outer_right = cenX_frame1 + 225
    inner_top = cenY_frame1 - 100
    outer_top = cenY_frame1 - 150
    inner_bottom = cenY_frame1 + 100
    outer_bottom = cenY_frame1 + 150

    wait = 0
    state = "CARI BOLA"

    #serongKiri()
    #sleep(1)
    #serongKiri()
    #sleep(3.5)
    #db.write(b"DB ON\n")
    #berhenti()

    while(True):
        # read frame
        _, frame1 = cap1.read()
        _, frame2 = cap2.read()

        # read object 
        lBall1, uBall1, thBall1 = getObjectInfo(1) # param 1 = ball, param 2 = goal
        lBall2, uBall2, thBall2 = getObjectInfo(2) # param 1 = ball, param 2 = goal

        ballContours1 = imageProcessing(frame1, lBall1, uBall1, thBall1)
        ballContours2 = imageProcessing(frame2, lBall2, uBall2, thBall2)

        ball_area1, x_ball1, y_ball1, w_ball1, h_ball1, cenX_ball1, cenY_ball1 = findObject(frame1, ballContours1, 500)
        ball_area2, x_ball2, y_ball2, w_ball2, h_ball2, cenX_ball2, cenY_ball2 = findObject(frame2, ballContours2, 10)
       
        # draw actual coordinate from segmentation
        if cenX_ball1 > 0 and cenY_ball1 > 0:
            cv2.circle(frame1, (int(cenX_ball1), int(cenY_ball1)), 20, [0,255,0], 2, 8)
            cv2.line(frame1, (int(cenX_ball1), int(cenY_ball1 + 20)), (int(cenX_ball1 + 50), int(cenY_ball1 + 20)), [0,255,0], 2, 8)
            cv2.putText(frame1, "Actual", (int(cenX_ball1 + 50), int(cenY_ball1 + 20)), font, 0.5, [0,255,0], 2)
        if cenX_ball2 > 0 and cenY_ball2 > 0:
            cv2.circle(frame2, (int(cenX_ball2), int(cenY_ball2)), 20, [0,255,0], 2, 8)
            cv2.line(frame2, (int(cenX_ball2), int(cenY_ball2 + 20)), (int(cenX_ball2 + 50), int(cenY_ball2 + 20)), [0,255,0], 2, 8)
            cv2.putText(frame2, "Actual", (int(cenX_ball2 + 50), int(cenY_ball2 + 20)), font, 0.5, [0,255,0], 2)

        # navigasi
        if state == "CARI BOLA" and cenX_ball2 > 0 and cenY_ball2 > 0:
            #print(cenY_ball2)
            if cenY_ball2 < 120:
                wait = 10
                if cenX_ball2 <= 180:
                    # bola di kanan robot
                    pid = PID(cenX_ball2, 190)
                    print("Kanan: "+str(pid))
                    if pid > 0 and pid < 35 or pid > -35 and pid < 0:
                        pid = 0
                        state = "ADA BOLA"
                    setMotor(pid,pid,pid,pid)
                elif cenX_ball2 >= 200:
                    # bola di kiri robot
                    pid = PID(cenX_ball2, 190)
                    print("Kiri: "+str(pid))
                    if pid > 0 and pid < 35 or pid > -35 and pid < 0:
                        pid = 0
                        state = "ADA BOLA"
                    setMotor(pid,pid,pid,pid)
                elif cenX_ball2 > 180 and cenX_ball2 < 200:
                    # motor maju
                    db.write(b"DB ON\n")
                    setMotor(-100,100,-100,100)

        if state == "ADA BOLA" and cenX_ball1 > 0 and cenY_ball1 > 0:
            if cenY_ball1 > 0 and cenY_ball1 < 210:
                # motor maju
                setMotor(-100,100,-100,100)
                wait = 10
            else:
                setMotor(0,0,0,0)
                state = "CARI BOLA"

        print(state)

        wait -= 1
        if wait <= 0:
            wait = 0
            state = "CARI BOLA"
            # bola hilang
            # kirim serial matikan dribble dan motor
            db.write(b"DB OFF\n")
            setMotor(0,0,0,0)

        # displays

        ## uncomment this to show center area of the frame 1
        #cv2.rectangle(frame1, (inner_left, inner_top), (inner_right, inner_bottom), (0,255,0), 2)
        #cv2.rectangle(frame1, (outer_left, outer_top), (outer_right, outer_bottom), (0,255,255), 2)

        cv2.imshow("Cam2", frame2)
        cv2.imshow("Cam1", frame1)
        cv2.moveWindow("Cam1", 0,0)
        cv2.moveWindow("Cam2", 500,0)


        k = cv2.waitKey(1) & 0xFF
        if k == 27:
            setMotor(0,0,0,0)
            db.write(b"DB OFF")
            frame1.release()
            cv2.destroyAllWindows()
            break

if __name__ == "__main__":
    # execute main program
    main()
