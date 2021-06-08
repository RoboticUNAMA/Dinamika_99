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

def nothing(x):
    #print(x)
    pass

def getBallInfo():
    infoFile = open("ballColor.txt","r")
    info = []
    for i in infoFile:
        info.append(int(i))
    return info

## initialize
font = cv2.FONT_HERSEY_SIMPLEX
date = str(datetime.datetime.now().strftime("%d-%m-%Y %H:%M:%S"))
#FRONT_CAM = 0   # front camera
#OMNI_CAM = 1    # omni camera

## create opencv video capture object
#FRONT_CAP = cv2.VideoCapture(FRONT_CAM) 
#OMNI_CAP = cv2.VideoCapture(OMNI_CAM)

## set frame size
#FRONT_CAP.set(cv2.CAP_PROP_FRAME_WIDTH, 600)
#FRONT_CAP.set(cv2.CAP_PROP_FRAME_WIDTH, 400)

#OMNI_CAP.set(cv2.CAP_PROP_FRAME_WIDTH, 600)
#OMNI_CAP.set(cv2.CAP_PROP_FRAME_WIDTH, 400)

## get center of the frame
#_, frame1 = FRONT_CAP.read()
#rows,cols = frame1.shape
#cenX_frame1 = int(cols/2)
#cenY_frame1 = int(rows/2)

#_, frame2 = OMNI_CAP.read()
#rows,cols = frame2.shape
#cenX_frame2 = int(cols/2)
#cenY_frame2 = int(rows/2)

ballColor = getBallInfo()

while(True):
    ## read frame
    #_, frame1 = FRONT_CAP.read()
    #_, frame2 = OMNI_CAP.read()

    lowerBall = np.array([ballColor[0],ballColor[1],ballColor[2]])
    upperBall = np.array([ballColor[3],ballColor[4],ballColor[5]])

    # convert frame from BGR to HSV
    hsv = cv2.cvtColor(frame1, cv2.COLOR_BGR2HSV)

    # blur the frame
    blur = cv2.medianBlur(hsv, 5)

    # create a mask from blurred frame
    BALL_MASK = cv2.inRange(blue, lowerBall, upperBall)

    # convert to black and white image
    _, BALL_THRESH = cv2.threshold(BALL_MASK, 127, 255, 0)

    # refine the image using morphological transformation
    kernal = np.ones((5,5), np.uint8)
    BALL_MORPH = cv2.morphologyEx(BALL_THRESH, cv2.MORPH_CLOSE, kernal, iterations = 2)

    # find contours
    _, ballContours, _ = cv2.findContours(BALL_MORPH, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    ballContours = sorted(contours, key=lambda x:cv2.contourArea(x), reverse=True)

    for ballContour in ballContours:
        area = cv2.contourArea(ballContour)
        if area > 1000:
            (x,y,w,h) = cv2.boundingRect(ballContour)
            cv2.putText(frame, "X: "+str(x)+" Y: "+str(y), (520, 20), font, 0.5, (0,0,255),2)
            cenX_ball = (x+x+w)/2
            cenY_ball = (y+y+h)/2
            predicted = kfObj.Estimate(cenX, cenY)

            # draw actual coordinate from segmentation
            cv2.circle(frame, (int(cenX_ball), int(cenY_ball)), 20, [0,255,0], 2, 8)
            cv2.line(frame, (int(cenX_ball), int(cenY_ball + 20)), (int(cenX_ball + 50), int(cenY_ball + 20)), [0,255,0], 2, 8)
            cv2.putText(frame, "Actual", (int(cenX_ball + 50), int(cenY_ball + 20)))
