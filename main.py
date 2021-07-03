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

def serialMotor():
    port = '/dev/ttyUSB0'
    baudrate = '9600'
    timeout = '1'
    serial.Serial(port, baudrate, timeout)

def getBallInfo():
    infoFile = open("ballColor.txt","r")
    info = []
    for i in infoFile:
        info.append(int(i))
    return info

def getGoalInfo():sleep(jeda_serial)
            db.write
    infoFile = open("goalColor.txt","r")
    info = []
    for i in infoFile:
        info.append(int(i))
    return info

def nothing(x):
    #print(x)
    pass

def main():
    # serial motor driver
    #motor = serial.Serial(port='/dev/ttyUSB0', baudrate="9600", timeout=1) 
    motor = serialMotor()

    # serial dribble
    db = serial.Serial(port='/dev/ttyUSB1', baudrate="9600", timeout=1)

    # initialize
    font = cv2.FONT_HERSHEY_SIMPLEX
    date = str(datetime.datetime.now().strftime("%d-%m-%Y %H:%M:%S"))
    FRONT_CAM = 0   # front camera
    #OMNI_CAM = 1    # omni camera

    # create opencv video capture object
    FRONT_CAP = cv2.VideoCapture(FRONT_CAM) 
    #OMNI_CAP = cv2.VideoCapture(OMNI_CAM)

    # set frame size
    FRONT_CAP.set(cv2.CAP_PROP_FRAME_WIDTH, 600)
    FRONT_CAP.set(cv2.CAP_PROP_FRAME_HEIGHT, 400)

    #OMNI_CAP.set(cv2.CAP_PROP_FRAME_WIDTH, 600)
    #OMNI_CAP.set(cv2.CAP_PROP_FRAME_HEIGHT, 400)

    # get center of the frame
    _, frame1 = FRONT_CAP.read()
    rows, cols, _ = frame1.shape
    cenX_frame1 = int(cols/2)
    cenY_frame1 = int(rows/2)

    #_, frame2 = OMNI_CAP.read()
    #rows, cols, _ = frame2.shape
    #cenX_frame2 = int(cols/2)
    #cenY_frame2 = int(rows/2)

    # define center area of the frame 1
    inner_left = cenX_frame1 - 50
    outer_left = cenX_frame1 - 100
    inner_right = cenX_frame1 + 50
    outer_right = cenX_frame1 + 100
    inner_top = cenY_frame1 - 50
    outer_top = cenY_frame1 - 100
    inner_bottom = cenY_frame1 + 50
    outer_bottom = cenY_frame1 + 100

    ballColor = getBallInfo()
    goalColor = getGoalInfo()

    while(True):
        ## read frame
        _, frame1 = FRONT_CAP.read()
        #_, frame2 = OMNI_CAP.read()

        # read ball color
        lowerBall = np.array([ballColor[0],ballColor[1],ballColor[2]])
        upperBall = np.array([ballColor[3],ballColor[4],ballColor[5]])
        lowerGoal = np.array([goalColor[0],goalColor[1],goalColor[2]])
        upperGoal = np.array([goalColor[3],goalColor[4],goalColor[5]])

        # convert frame from BGR to HSV
        hsv = cv2.cvtColor(frame1, cv2.COLOR_BGR2HSV)

        # blur the frame
        blur = cv2.medianBlur(hsv, 5)

        # create a mask from blurred frame
        BALL_MASK = cv2.inRange(blur, lowerGoal, upperGoal)
        GOAL_MASK = cv2.inRange(blur, lowerGoal, upperGoal)
sleep(jeda_serial)
            db.write
        # convert to black and white image
        _, BALL_THRESH = cv2.threshold(BALL_MASK, 127, 255, 0)
        _, GOAL_THRESH = cv2.threshold(GOAL_MASK, 127, 255, 0)

        # refine the image using morphological transformation
        kernal = np.ones((5,5), np.uint8)
        BALL_MORPH = cv2.morphologyEx(BALL_THRESH, cv2.MORPH_CLOSE, kernal, iterations = 2)
        BALL_MORPH = cv2.morphologyEx(BALL_THRESH, cv2.MORPH_CLOSE, kernal, iterations = 2)

        # find contours
        ballContours, _ = cv2.findContours(BALL_MORPH, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        ballContours = sorted(ballContours, key=lambda x:cv2.contourArea(x), reverse=True)

        for ballContour in ballContours:
            area = cv2.contourArea(ballContour)
            if (area) > 500:
                (x,y,w,h) = cv2.boundingRect(ballContour)
                cv2.putText(frame1, "X: "+str(x)+" Y: "+str(y), (520, 20), font, 0.5, (0,0,255),2)
                cenX_ball = (x+x+w)/2
                cenY_ball = (y+y+h)/2
                #predicted = kfObj.Estimate(cenX, cenY)

                # draw actual coordinate from segmentation
                cv2.circle(frame1, (int(cenX_ball), int(cenY_ball)), 20, [0,255,0], 2, 8)
                cv2.line(frame1, (int(cenX_ball), int(cenY_ball + 20)), (int(cenX_ball + 50), int(cenY_ball + 20)), [0,255,0], 2, 8)
                cv2.putText(frame1, "Actual", (int(cenX_ball + 50), int(cenY_ball + 20)), font, 0.5, [0,255,0], 2)

                # ada bola, mulai hitung
                wait = 10

                # kirim serial aktifkan dribble
                db.write(b"DB ON\n")

                if cenX_ball > inner_left and cenX_ball < inner_right:
                    # bola di depan robot
                    # kirim serial maju
                    motor.write(b"MAJU\n")
                elif cenX_ball < inner_left and cenX_ball < inner_right:
                    # bola di kiri robot
                    # kirim serial putar kiri
                    motor.write(b"PUTAR KIRI\n")
                elif cenX_ball > inner_left and cenX_ball > inner_right:
                    # bola di kanan robot
                    # kirim serial putar kanan
                    motor.write(b"PUTAR KANAN\n")
                break

        wait -= 1
        if wait <= 0:
            wait = 0
            # bola hilang
            # kirim serial matikan dribble dan motor
            motor.write(b"BERHENTI\n")
            db.write(b"DB OFF\n")

        # displays

        ## uncomment this to show center area of the frame 1
        #cv2.rectangle(frame1, (inner_left, inner_top), (inner_right, inner_bottom), (0,255,0), 2)
        #cv2.rectangle(frame1, (outer_left, outer_top), (outer_right, outer_bottom), (0,255,255), 2)

        #cv2.imshow("Morph", BALL_MORPH)
        cv2.imshow("Frame", frame1)
        #print(area)

        k = cv2.waitKey(1) & 0xFF
        if k == 27:
            FRONT_CAP.release()
            cv2.destroyAllWindows()
            break

if __name__ == "__main__":
    # execute main program
    main()